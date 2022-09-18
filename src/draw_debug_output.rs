use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::{DrawTarget, RgbColor},
};
use embedded_hal::digital::v2::OutputPin;
use fugit::Instant;
use ili9341::Ili9341;
use rp_pico::hal::{spi, Spi, Timer};

pub type Lcd = Ili9341<
    SPIInterface<
        Spi<spi::Enabled, rp_pico::pac::SPI0, 8>,
        rp_pico::hal::gpio::Pin<
            rp_pico::hal::gpio::bank0::Gpio20,
            rp_pico::hal::gpio::Output<rp_pico::hal::gpio::PushPull>,
        >,
        rp_pico::hal::gpio::Pin<
            rp_pico::hal::gpio::bank0::Gpio17,
            rp_pico::hal::gpio::Output<rp_pico::hal::gpio::PushPull>,
        >,
    >,
    rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio22,
        rp_pico::hal::gpio::Output<rp_pico::hal::gpio::PushPull>,
    >,
>;

fn handle_drawing_output(
    mut lcd: Lcd,
    mut sio: rp_pico::hal::Sio,
    timer: Timer,
    mut debug_pin: rp_pico::hal::gpio::Pin<
        rp_pico::hal::gpio::bank0::Gpio26,
        rp_pico::hal::gpio::Output<rp_pico::hal::gpio::PushPull>,
    >,
) -> ! {
    lcd.clear(Rgb565::BLACK).unwrap();
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::WHITE);
    let mut lut = [0u16; 256];
    let mut frame_count = 0;
    let mut prev_focus_metric = 0;
    let mut motion_histogram = [0u32; 240];
    // for (i, x) in motion_histogram.iter_mut().enumerate() {
    //     *x = i as u32 * 1000;
    // }
    for i in 0..255u16 {
        lut[i as usize] = ((i & 0b11111000) << 8) | ((i & 0b11111100) << 3) | (i >> 3);
    }
    loop {
        let mut draw_time: u32 = 0;
        // Get something from the FIFO, which triggers us to wake up and render the frame.
        let ready = sio.fifo.read_blocking();
        if ready == 1 {
            let _ = debug_pin.set_high();
            // Get ticks from clock?
            let mut frame_telemetry: Option<crate::FrameTelemetry> = None;

            // TODO: Also need an accumulation buffer, and need to do it only with fixed point maths.

            critical_section::with(|cs| {
                frame_count += 1;
                crate::FRAME_BUFFER.swap();
                let mut current_frame_ref = crate::FRAME_BUFFER.frame_1.borrow_ref_mut(cs);
                let mut prev_frame_ref = crate::FRAME_BUFFER.frame_2.borrow_ref_mut(cs);
                // Okay, so the "scene stats" min/max intensity have nothing to do with the actual values we see.  Huh.

                let min = current_frame_ref.telemetry.min_value;
                let max = current_frame_ref.telemetry.max_value;
                motion_histogram[frame_count % 240] = current_frame_ref
                    .telemetry
                    .focus_metric
                    .abs_diff(prev_focus_metric);
                prev_focus_metric = current_frame_ref.telemetry.focus_metric;
                frame_telemetry.replace(current_frame_ref.telemetry);
                let current_frame = &mut current_frame_ref.frame;

                let prev_frame = &mut prev_frame_ref.frame;

                let range = max - min;
                let div = u16::MAX / range;

                // TODO - Pull in some cptv code here

                // info!("Start compression");
                // let mut compressor = miniz_oxide::deflate::core::CompressorOxide::new(0);
                // compressor.set_compression_level(CompressionLevel::BestSpeed);
                // let mut out = [0u8; 100];
                // let (status, in_pos, out_pos) = miniz_oxide::deflate::core::compress(
                //     &mut compressor,
                //     current_frame.as_byte_slice(),
                //     &mut out,
                //     TDEFLFlush::Finish,
                // );
                // info!("Compressed in pos {}, out pos {}", in_pos, out_pos);

                /*
                let diff_iter =
                    current_frame
                        .iter()
                        .zip(prev_frame.iter())
                        .map(|(&c_px, &p_px)| {
                            lut[((((c_px.abs_diff(p_px)) - min) * div) / 256u16) as usize]
                        });
                */
                // ~25ms
                let now = Instant::<u32, 1, 1_000_000>::from_ticks(timer.get_counter_low());
                lcd.draw_raw_iter(
                    0,
                    0,
                    159,
                    119,
                    current_frame
                        .iter()
                        .map(|&px| lut[(((px - min) * div) / 256u16) as usize]),
                )
                .unwrap();
                let later = Instant::<u32, 1, 1_000_000>::from_ticks(timer.get_counter_low());
                draw_time = later.checked_duration_since(now).unwrap().ticks();
            });

            // ~25ms
            // if let Some(frame_telemetry) = frame_telemetry {
            //     let range = frame_telemetry.max_value - frame_telemetry.min_value;
            //     let mut static_string = StaticString::new();

            //     // Setup text buffer by lines
            //     static_string.push_str("Frame");
            //     static_string.push_space();
            //     static_string.push_str("#");
            //     static_string.push_u32(frame_telemetry.count);
            //     static_string.push_space();
            //     static_string.push_str("Focus");
            //     static_string.push_space();
            //     static_string.push_u32(frame_telemetry.focus_metric);

            //     // static_string.newline();
            //     // static_string.push_str("SceneMin");
            //     // static_string.push_space();
            //     // static_string.push_u16(frame_telemetry.min_value);
            //     // static_string.push_space();
            //     // static_string.push_str("SceneMax");
            //     // static_string.push_space();
            //     // static_string.push_u16(frame_telemetry.max_value);

            //     static_string.newline();
            //     static_string.push_str("Min");
            //     static_string.push_space();
            //     static_string.push_u16(frame_telemetry.min_value);

            //     static_string.push_space();
            //     static_string.push_str("Max");
            //     static_string.push_space();
            //     static_string.push_u16(frame_telemetry.max_value);

            //     static_string.push_space();
            //     static_string.push_str("Range");
            //     static_string.push_space();
            //     static_string.push_u16(range);

            //     static_string.newline();
            //     static_string.push_str("T(Frame)");
            //     static_string.push_space();
            //     static_string.push_u32(frame_telemetry.process_time);

            //     static_string.newline();
            //     static_string.push_str("T(draw)");
            //     static_string.push_space();
            //     static_string.push_u32(draw_time);

            //     lcd.fill_solid(
            //         &Rectangle::new(
            //             Point::new(0, 132),
            //             Size::new(240, static_string.num_lines() * 10),
            //         ),
            //         Rgb565::BLACK,
            //     )
            //     .unwrap();

            //     Text::with_alignment(
            //         static_string.get(),
            //         Point::new(10, 140),
            //         style,
            //         Alignment::Left,
            //     )
            //     .draw(&mut lcd)
            //     .unwrap();

            //     {
            //         // Draw a little motion histogram based on focus metric
            //         let max = *motion_histogram.iter().max().unwrap();
            //         let min = *motion_histogram.iter().min().unwrap();

            //         let range = (max - min) as f32;
            //         //info!("min {}, max {}, range {}", min, max, range);
            //         // Scale to

            //         let hist_y = 132 + ((static_string.num_lines() * 10) + 20) as i32;
            //         // FIXME - Do we need to adjust our start offset into the iterator?
            //         let mut hist: [u16; 240 * 20] = [0u16; 240 * 20];
            //         let red = ((255 & 0b11111000) << 8) | ((0 & 0b11111100) << 3) | (0 >> 3);
            //         for (x, &val) in motion_histogram
            //             .iter()
            //             .cycle()
            //             .skip(frame_count % 240)
            //             .enumerate()
            //             .take(240)
            //         {
            //             let scale_y = 20 - (((val - min) as f32 / range) * 20.0) as usize;
            //             for y in scale_y..20 {
            //                 hist[y * 240 + x] = red;
            //             }
            //         }
            //         lcd.draw_raw_slice(0, hist_y as u16, 239, hist_y as u16 + 20, &hist)
            //             .unwrap();
            //     }
            // }
            loop {
                // Clear out the FIFO so we don't drop frames.
                if sio.fifo.read().is_none() {
                    break;
                }
            }
            let _ = debug_pin.set_low();
            let later = Instant::<u32, 1, 1_000_000>::from_ticks(timer.get_counter_low());
            // info!(
            //     "Rendering {} µs",
            //     later.checked_duration_since(now).unwrap().ticks()
            // );
        }
    }
}
