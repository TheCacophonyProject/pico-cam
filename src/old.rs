{
// info!("=== Init LCD ===");
// let lcd: Lcd = {
//     let _lcd_mosi: Pin<Gpio19, FunctionSpi> = pins.gpio19.into_mode();
//     let _lcd_clk: Pin<Gpio18, FunctionSpi> = pins.gpio18.into_mode();
//     let lcd_dc = pins.gpio20.into_push_pull_output(); // spi tx
//     let lcd_rst = pins.gpio22.into_push_pull_output(); // spi sck
//     let lcd_cs = pins.gpio17.into_push_pull_output(); // spi cs
//     let lcd_spi: Spi<_, _, 8> = Spi::new(pac.SPI0).init(
//         &mut pac.RESETS,
//         clocks.peripheral_clock.freq(), // 125_000_000
//         48_000_000u32.Hz(),
//         &embedded_hal::spi::MODE_0,
//     );
//     let spi_iface = SPIInterface::new(lcd_spi, lcd_dc, lcd_cs);
//     let lcd = Ili9341::new(
//         spi_iface,
//         lcd_rst,
//         &mut delay,
//         Orientation::PortraitFlipped,
//         DisplaySize240x320,
//     )
//     .unwrap();
//     lcd
// };
//
// let debug_render_pin: Pin<Gpio26, Output<PushPull>> = pins.gpio26.into_push_pull_output();
//
// let _ = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || {
//     // Get the second core's copy of the `CorePeripherals`, which are per-core.
//     // Unfortunately, `cortex-m` doesn't support this properly right now,
//     // so we have to use `steal`.
//     info!("Init Core 1");
//     let mut pac = unsafe { pac::Peripherals::steal() };
//     let _core = unsafe { pac::CorePeripherals::steal() };
//     let sio = Sio::new(pac.SIO);
//     // Set up the delay for the second core.
//     //let delay = Delay::new(core.SYST, sys_freq);
//     let timer = Timer::new(pac.TIMER, &mut pac.RESETS);
//     handle_drawing_output(lcd, sio, timer, debug_render_pin);
// });
}
