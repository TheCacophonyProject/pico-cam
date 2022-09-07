//! Blin&ks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod lepton;
mod static_string;
use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock, ValidSrc},
        gpio::{
            bank0::{Gpio17, Gpio18, Gpio20, Gpio22},
            bank0::{Gpio19, Gpio25},
            FunctionSpi, Interrupt, Output, Pin, PushPull, PushPullOutput,
        },
        multicore::{Multicore, Stack},
        pac,
        pac::interrupt,
        sio::{Sio, SioFifo},
        spi::Enabled,
        watchdog::Watchdog,
        Spi, Timer,
    },
    pac::SPI0,
};
use byte_slice_cast::AsByteSlice;
use byteorder::{ByteOrder, LittleEndian};
use lepton::Lepton;
use miniz_oxide::deflate::{core::TDEFLFlush, CompressionLevel};
use numtoa::NumToA;
use rp_pico as bsp;

use cortex_m::delay::Delay;
use defmt::*;
use defmt_rtt as _;

use embedded_sdmmc::{TimeSource, Timestamp};
use ili9341::{DisplaySize240x320, Ili9341, Orientation};

use panic_probe as _;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::Rectangle,
    text::{Alignment, Text},
};

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

// use sparkfun_pro_micro_rp2040 as bsp;
use fugit::{Instant, MicrosDurationU32, MillisDurationU32, RateExtU32};

// Some traits we need
use embedded_hal::digital::v2::ToggleableOutputPin;
use static_string::StaticString;

use core::{
    cell::{RefCell, UnsafeCell},
    time::Duration,
};
use critical_section::Mutex;

use crate::lepton::TelemetryLocation;

static mut CORE1_STACK: Stack<4096> = Stack::new();

const SEGMENT_LENGTH: usize = (160 * 122) / 4;

struct ClockA;

#[derive(Clone, Copy)]
enum FfcState {
    NeverCommanded,
    Imminent,
    InProgress,
    Complete,
}

#[derive(Clone, Copy)]
struct FrameTelemetry {
    count: u32,
    time_on: u32,
    ffc_desired: bool,
    ffc_state: FfcState,
    focus_metric: u32,
    min_value: u16,
    max_value: u16,
    avg_value: u16,
    spot_min: u16,
    spot_max: u16,
    spot_avg: u16,
    process_time: u32,
}

impl FrameTelemetry {
    const fn new() -> Self {
        FrameTelemetry {
            count: 0,
            time_on: 0,
            ffc_desired: false,
            ffc_state: FfcState::NeverCommanded,
            focus_metric: 0,
            min_value: u16::MAX,
            max_value: u16::MIN,
            avg_value: 0,
            spot_avg: 0,
            spot_min: u16::MAX,
            spot_max: u16::MIN,
            process_time: 0,
        }
    }
}
struct Frame {
    frame: [u16; 160 * 120],
    telemetry: FrameTelemetry,
}

impl Frame {
    pub const fn new() -> Frame {
        Frame {
            frame: [0u16; 160 * 120],
            telemetry: FrameTelemetry::new(),
        }
    }
}

impl TimeSource for ClockA {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct TripleBuffer {
    frame_0: Mutex<RefCell<Frame>>,
    frame_1: Mutex<RefCell<Frame>>,
    frame_2: Mutex<RefCell<Frame>>,
}

impl TripleBuffer {
    pub fn swap(&self) {
        critical_section::with(|cs| {
            self.frame_0.borrow(cs).swap(self.frame_2.borrow(cs));
            self.frame_1.borrow(cs).swap(self.frame_2.borrow(cs));
        });
    }
}

type InterruptState = Option<(Lepton, LedPin, Delay, SioFifo, VoSpiState, Timer)>;
type Lcd = Ili9341<
    SPIInterface<
        Spi<Enabled, SPI0, 8>,
        Pin<Gpio20, Output<PushPull>>,
        Pin<Gpio17, Output<PushPull>>,
    >,
    Pin<Gpio22, Output<PushPull>>,
>;
type LedPin = Pin<Gpio25, PushPullOutput>;
static GLOBAL_LEPTON_STATE: Mutex<RefCell<InterruptState>> = Mutex::new(RefCell::new(None));

static FRAME_BUFFER: TripleBuffer = TripleBuffer {
    frame_0: Mutex::new(RefCell::new(Frame::new())),
    frame_1: Mutex::new(RefCell::new(Frame::new())),
    frame_2: Mutex::new(RefCell::new(Frame::new())),
};

fn handle_drawing_output(mut lcd: Lcd, mut sio: Sio, timer: Timer) -> ! {
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
            // Get ticks from clock?
            let mut frame_telemetry: Option<FrameTelemetry> = None;

            // TODO: Also need an accumulation buffer, and need to do it only with fixed point maths.

            critical_section::with(|cs| {
                frame_count += 1;
                FRAME_BUFFER.swap();
                let mut current_frame_ref = FRAME_BUFFER.frame_1.borrow_ref_mut(cs);
                let mut prev_frame_ref = FRAME_BUFFER.frame_2.borrow_ref_mut(cs);
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
            if let Some(frame_telemetry) = frame_telemetry {
                let range = frame_telemetry.max_value - frame_telemetry.min_value;
                let mut static_string = StaticString::new();

                // Setup text buffer by lines
                static_string.push_str("Frame");
                static_string.push_space();
                static_string.push_str("#");
                static_string.push_u32(frame_telemetry.count);
                static_string.push_space();
                static_string.push_str("Focus");
                static_string.push_space();
                static_string.push_u32(frame_telemetry.focus_metric);

                // static_string.newline();
                // static_string.push_str("SceneMin");
                // static_string.push_space();
                // static_string.push_u16(frame_telemetry.min_value);
                // static_string.push_space();
                // static_string.push_str("SceneMax");
                // static_string.push_space();
                // static_string.push_u16(frame_telemetry.max_value);

                static_string.newline();
                static_string.push_str("Min");
                static_string.push_space();
                static_string.push_u16(frame_telemetry.min_value);

                static_string.push_space();
                static_string.push_str("Max");
                static_string.push_space();
                static_string.push_u16(frame_telemetry.max_value);

                static_string.push_space();
                static_string.push_str("Range");
                static_string.push_space();
                static_string.push_u16(range);

                static_string.newline();
                static_string.push_str("T(Frame)");
                static_string.push_space();
                static_string.push_u32(frame_telemetry.process_time);

                static_string.newline();
                static_string.push_str("T(draw)");
                static_string.push_space();
                static_string.push_u32(draw_time);

                lcd.fill_solid(
                    &Rectangle::new(
                        Point::new(0, 132),
                        Size::new(240, static_string.num_lines() * 10),
                    ),
                    Rgb565::BLACK,
                )
                .unwrap();

                Text::with_alignment(
                    static_string.get(),
                    Point::new(10, 140),
                    style,
                    Alignment::Left,
                )
                .draw(&mut lcd)
                .unwrap();

                {
                    // Draw a little motion histogram based on focus metric
                    let max = *motion_histogram.iter().max().unwrap();
                    let min = *motion_histogram.iter().min().unwrap();

                    let range = (max - min) as f32;
                    //info!("min {}, max {}, range {}", min, max, range);
                    // Scale to

                    let hist_y = 132 + ((static_string.num_lines() * 10) + 20) as i32;
                    // FIXME - Do we need to adjust our start offset into the iterator?
                    let mut hist: [u16; 240 * 20] = [0u16; 240 * 20];
                    let red = ((255 & 0b11111000) << 8) | ((0 & 0b11111100) << 3) | (0 >> 3);
                    for (x, &val) in motion_histogram
                        .iter()
                        .cycle()
                        .skip(frame_count % 240)
                        .enumerate()
                        .take(240)
                    {
                        let scale_y = 20 - (((val - min) as f32 / range) * 20.0) as usize;
                        for y in scale_y..20 {
                            hist[y * 240 + x] = red;
                        }
                    }
                    lcd.draw_raw_slice(0, hist_y as u16, 239, hist_y as u16 + 20, &hist)
                        .unwrap();
                }
            }
            loop {
                // Clear out the FIFO so we don't drop frames.
                if sio.fifo.read().is_none() {
                    break;
                }
            }
            let later = Instant::<u32, 1, 1_000_000>::from_ticks(timer.get_counter_low());
            // info!(
            //     "Rendering {} µs",
            //     later.checked_duration_since(now).unwrap().ticks()
            // );
        }
    }
}

struct VoSpiState {
    prev_packet_id: isize,
    started_segment: bool,
    current_segment_num: u16,
    packet_id_mismatches: u8,
    reboots: u32,
    last_reboot_time: u32,
}

impl VoSpiState {
    pub const fn new() -> VoSpiState {
        VoSpiState {
            prev_packet_id: -1,
            started_segment: false,
            current_segment_num: 0,
            packet_id_mismatches: 0,
            reboots: 0,
            last_reboot_time: 0,
        }
    }

    pub fn reset(&mut self) {
        self.prev_packet_id = -1;
        self.started_segment = false;
        self.current_segment_num = 0;
    }
}

type MicroSecondsInstant = Instant<u32, 1, 1_000_000>;

#[interrupt]
fn IO_IRQ_BANK0() {
    // Vsync interrupt
    static mut LOCAL_LEPTON_STATE: InterruptState = None;

    if LOCAL_LEPTON_STATE.is_none() {
        critical_section::with(|cs| {
            *LOCAL_LEPTON_STATE = GLOBAL_LEPTON_STATE.borrow(cs).take();
        });
    }

    if let Some((lepton, led_pin, delay, sio, vo_spi_state, timer)) = LOCAL_LEPTON_STATE {
        let start = MicroSecondsInstant::from_ticks(timer.get_counter_low());
        if lepton.spi.vsync.interrupt_status(Interrupt::EdgeHigh) {
            if (lepton.is_resetting()) {
                warn!("Got vsync during reset");
            }
            // Our interrupt doesn't clear itself.
            // Do that now so we don't immediately jump back to this interrupt handler.
            lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);

            critical_section::with(|cs| {
                let mut frame = FRAME_BUFFER.frame_0.borrow(cs).borrow_mut();

                vo_spi_state.started_segment = false;
                loop {
                    let mut buffer = [0u16; 82];
                    let packet = lepton.transfer(&mut buffer).unwrap();

                    let packet_header = packet[0];
                    let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
                    if is_discard_packet {
                        continue;
                    }
                    let packet_id = (packet_header & 0x0fff) as isize;
                    if packet_id == 0 {
                        vo_spi_state.prev_packet_id = -1;
                        vo_spi_state.started_segment = true;
                        frame.telemetry.process_time = 0;
                    } else if packet_id == 20 {
                        // Packet 20 is the only one that contains a meaningful segment number
                        let segment_num = packet_header >> 12;
                        // See if we're starting a frame, or ending it.
                        if vo_spi_state.current_segment_num > 0
                            && vo_spi_state.current_segment_num < 4
                            && segment_num != vo_spi_state.current_segment_num
                        {
                            // Segment order mismatch.
                            warn!(
                                "Segment order mismatch error: stored {}, this {}",
                                vo_spi_state.current_segment_num, segment_num
                            );
                            vo_spi_state.reset();
                            // Reset and resync
                            lepton.wait_for_ready(delay);
                            lepton.reset(delay);

                            let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                            let elapsed = end.checked_duration_since(start).unwrap().ticks();
                            frame.telemetry.process_time += elapsed;

                            break;
                        }
                        vo_spi_state.current_segment_num = segment_num;
                        if vo_spi_state.current_segment_num == 0 {
                            vo_spi_state.started_segment = false;
                        }
                    }
                    if vo_spi_state.started_segment {
                        if packet_id != vo_spi_state.prev_packet_id + 1 {
                            // Packet order mismatch
                            warn!(
                                "Packet order mismatch current: {}, prev: {}, seg {}",
                                packet_id,
                                vo_spi_state.prev_packet_id,
                                vo_spi_state.current_segment_num
                            );
                            vo_spi_state.reset();

                            if vo_spi_state.packet_id_mismatches > 3 {
                                vo_spi_state.packet_id_mismatches = 0;
                                let now = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                                let last =
                                    MicroSecondsInstant::from_ticks(vo_spi_state.last_reboot_time);
                                let elapsed = now.checked_duration_since(last).unwrap().ticks();
                                warn!(
                                    "{} prior reboots, last was {} mins ago, {} frames ago",
                                    vo_spi_state.reboots,
                                    elapsed / 1000 / 1000 / 60,
                                    frame.telemetry.count
                                );
                                vo_spi_state.reboots += 1;
                                vo_spi_state.last_reboot_time = timer.get_counter_low();
                                lepton.reboot(delay, false);
                                break;
                            }
                            // Reset and resync - do we need to disable vsync in here so we don't keep entering this function?
                            lepton.reset(delay);
                            vo_spi_state.packet_id_mismatches += 1;

                            let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                            let elapsed = end.checked_duration_since(start).unwrap().ticks();
                            frame.telemetry.process_time += elapsed;

                            break;
                        }
                        let is_last_segment = vo_spi_state.current_segment_num == 4;
                        if packet_id < 61 && !is_last_segment || packet_id < 57 && is_last_segment {
                            // Write packet out to frame buffer
                            let packet_id = packet_id as usize;

                            let segment_offset = (u16::max(vo_spi_state.current_segment_num, 1) - 1)
                                as usize
                                * SEGMENT_LENGTH;
                            let side = packet_id % 2; // + (*current_segment_num - 1) as usize % 2) % 2;
                            let y = (packet_id - side) >> 1;
                            let offset = segment_offset + (y * 160) + (80 * side);
                            frame.frame[offset..offset + 80].copy_from_slice(&packet[2..]);
                        }
                        if packet_id == 60 && !is_last_segment {
                            // Increment in good faith if we're on the last packet of a valid segment
                            vo_spi_state.current_segment_num += 1;

                            let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                            let elapsed = end.checked_duration_since(start).unwrap().ticks();
                            frame.telemetry.process_time += elapsed;

                            break;
                        }
                        if packet_id == 57 && is_last_segment {
                            let _ = led_pin.toggle();

                            // FIXME: These numbers seem to be junk.  Confirm that we actually have TLinear on, and they're
                            // not actually a scaled 14bit value!  Could they be values from before image processing?
                            // If we turn TLinear off, do they match up?
                            let frame_stats = lepton.scene_stats(delay);
                            frame.telemetry.min_value = frame_stats.min;
                            frame.telemetry.max_value = frame_stats.max;
                            frame.telemetry.avg_value = frame_stats.avg;

                            // TODO - Maybe also get the spot-meter roi here?

                            // Read out four telemetry packets (including the current one) and parse them.

                            // frame counter
                            let frame_counter =
                                LittleEndian::read_u32(&packet[2..][20..=21].as_byte_slice());
                            frame.telemetry.count = frame_counter;

                            // Grab two more telemetry rows, and discard
                            lepton.transfer(&mut buffer).unwrap(); // packet 58
                            lepton.transfer(&mut buffer).unwrap(); // packet 59
                            let spot_meter_avg = buffer[2..][50];
                            let spot_meter_max = buffer[2..][51];
                            let spot_meter_min = buffer[2..][52];
                            frame.telemetry.spot_avg = spot_meter_avg;
                            frame.telemetry.spot_min = spot_meter_min;
                            frame.telemetry.spot_max = spot_meter_max;

                            let spot_start_y = buffer[2..][54];
                            let spot_start_x = buffer[2..][55];
                            let spot_end_y = buffer[2..][56];
                            let spot_end_x = buffer[2..][57];
                            // info!(
                            //     "spot {}, {}, {}, {}",
                            //     spot_start_x, spot_end_x, spot_start_y, spot_end_y
                            // );

                            // Not sure if we need this last one?
                            //lepton.transfer(&mut buffer).unwrap(); // packet 60

                            // Get additional frame stats from CCI.
                            let focus_metric = lepton.get_focus_metric(delay);
                            frame.telemetry.focus_metric = focus_metric;

                            let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                            let elapsed = end.checked_duration_since(start).unwrap().ticks();
                            frame.telemetry.process_time += elapsed;
                            // Pass control of frame to other core for further processing.
                            sio.write(1);
                            break;
                        }
                    }
                    vo_spi_state.prev_packet_id = packet_id;
                }
            });
        }
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let mut sio = Sio::new(pac.SIO);
    let mut mc = Multicore::new(&mut pac.PSM, &mut pac.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("=== Init LCD ===");
    let lcd: Lcd = {
        let _lcd_mosi: Pin<Gpio19, FunctionSpi> = pins.gpio19.into_mode();
        let _lcd_clk: Pin<Gpio18, FunctionSpi> = pins.gpio18.into_mode();
        let lcd_dc = pins.gpio20.into_push_pull_output(); // spi tx
        let lcd_rst = pins.gpio22.into_push_pull_output(); // spi sck
        let lcd_cs = pins.gpio17.into_push_pull_output(); // spi cs
        let lcd_spi: Spi<_, _, 8> = Spi::new(pac.SPI0).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(), // 125_000_000
            48_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        );
        let spi_iface = SPIInterface::new(lcd_spi, lcd_dc, lcd_cs);
        let lcd = Ili9341::new(
            spi_iface,
            lcd_rst,
            &mut delay,
            Orientation::PortraitFlipped,
            DisplaySize240x320,
        )
        .unwrap();
        lcd
    };
    let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
        // Get the second core's copy of the `CorePeripherals`, which are per-core.
        // Unfortunately, `cortex-m` doesn't support this properly right now,
        // so we have to use `steal`.
        info!("Init Core 1");
        let mut pac = unsafe { pac::Peripherals::steal() };
        let _core = unsafe { pac::CorePeripherals::steal() };
        let sio = Sio::new(pac.SIO);
        // Set up the delay for the second core.
        //let delay = Delay::new(core.SYST, sys_freq);
        let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
        handle_drawing_output(lcd, sio, timer);
    });

    let mut lepton = lepton::Lepton::new(
        rp_pico::hal::I2C::i2c0(
            pac.I2C0,
            pins.gpio12.into_mode(),
            pins.gpio13.into_mode(),
            400.kHz(),
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
        ),
        Spi::new(pac.SPI1).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(), // 125_000_000
            20_000_000.Hz(),
            &embedded_hal::spi::MODE_3,
        ),
        pins.gpio9.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio8.into_mode(),
        pins.gpio10.into_mode(),
        pins.gpio28.into_mode(),
    );
    let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
    // Reboot every time we start up the program to get to a known state.

    lepton.reboot(&mut delay, true);

    let led_pin: LedPin = pins.led.into_push_pull_output();
    critical_section::with(|cs| {
        GLOBAL_LEPTON_STATE.borrow(cs).replace(Some((
            lepton,
            led_pin,
            delay,
            sio.fifo,
            VoSpiState::new(),
            timer,
        )));
    });

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }

    /*
    {
        // SDCARD stuff
        let _spi0_rx: Pin<Gpio16, FunctionSpi> = pins.gpio16.into_mode();
        let _spi0_tx: Pin<Gpio19, FunctionSpi> = pins.gpio19.into_mode();
        let _spi0_sck: Pin<Gpio18, FunctionSpi> = pins.gpio18.into_mode();
        let spi0_cs = pins.gpio17.into_push_pull_output();
        let sdmmc_spi: Spi<_, _, 8> = Spi::new(pac.SPI0).init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(), // 125_000_000
            25_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        );

        // Need to make an i2c interface that supports 16bit wide read/writes?

        let mut sd_controller = embedded_sdmmc::Controller::new(
            embedded_sdmmc::SdMmcSpi::new(sdmmc_spi, spi0_cs),
            ClockA,
        );
        let vol = match sd_controller.device().init() {
            Ok(_) => {
                match sd_controller.device().card_size_bytes() {
                    Ok(size) => {
                        info!("{}", size)
                    }
                    Err(_e) => info!("Error"),
                }
                match sd_controller.get_volume(VolumeIdx(0)) {
                    Ok(v) => Some(v),
                    Err(_e) => None,
                }
            }
            Err(_e) => None,
        };

        if let Some(mut volume) = vol {
            match sd_controller.open_root_dir(&volume) {
                Ok(root_dir) => {
                    match sd_controller.open_file_in_dir(
                        &mut volume,
                        &root_dir,
                        "PACKETS.DAT",
                        embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
                    ) {
                        Ok(mut file) => {
                            sd_controller
                                .write(&mut volume, &mut file, &frame.as_byte_slice())
                                .unwrap();
                            info!("File size {}", file.length());
                        }
                        Err(e) => {
                            warn!("Failed to open or create file");
                        }
                    }
                }
                Err(e) => warn!("Error opening root dir on volume"),
            }
        }
    }
    */
    loop {
        cortex_m::asm::wfi();
    }
    // It's probably fine to hold onto the volume, but we'll want to shutdown the sdcard peripheral when we're not using it.
}
