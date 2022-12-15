pub fn rosc_frequency_count_hz() -> u32 {
    // Use the reference xosc while enabled to measure the speed of the rosc, apparently.
    let peripherals = unsafe { pac::Peripherals::steal() };
    let clocks = peripherals.CLOCKS;
    while clocks.fc0_status.read().bits() & 0x00000100 != 0 {
        cortex_m::asm::nop();
    }

    clocks
        .fc0_ref_khz
        .write(|w| unsafe { w.fc0_ref_khz().bits(XOSC_CRYSTAL_FREQ / 1000) });
    clocks
        .fc0_interval
        .write(|w| unsafe { w.fc0_interval().bits(10) });
    clocks
        .fc0_min_khz
        .write(|w| unsafe { w.fc0_min_khz().bits(0) });
    clocks
        .fc0_max_khz
        .write(|w| unsafe { w.fc0_max_khz().bits(0xffffffff) });
    clocks.fc0_src.write(|w| unsafe { w.fc0_src().bits(0x09) });

    while clocks.fc0_status.read().bits() & 0x00000010 == 0 {
        cortex_m::asm::nop();
    }

    clocks.fc0_result.read().khz().bits() * 1000
}
