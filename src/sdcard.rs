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
// It's probably fine to hold onto the volume, but we'll want to shutdown the sdcard peripheral when we're not using it.
