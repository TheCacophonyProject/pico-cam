//! Blin&ks the LED on a Pico board
//!
//! This will blink an LED attached to GP25, which is the pin the Pico uses for the on-board LED.
#![no_std]
#![no_main]

mod draw_debug_output;
mod lepton;
mod static_string;
use bsp::{
    entry,
    hal::{
        clocks::{
            init_clocks_and_plls, Clock, ClockSource, ClocksManager, InitError, StoppableClock,
            ValidSrc,
        },
        gpio::{
            bank0::{Gpio17, Gpio18, Gpio20, Gpio22, Gpio26, Gpio27},
            bank0::{Gpio19, Gpio25},
            FunctionSpi, Interrupt, Output, Pin, PushPull, PushPullOutput,
        },
        multicore::{Multicore, Stack},
        pac,
        pac::interrupt,
        pll::{
            common_configs::{PLL_SYS_125MHZ, PLL_USB_48MHZ},
            setup_pll_blocking, Disabled, Locked, PhaseLockedLoop,
        },
        rosc::{self, Enabled, RingOscillator},
        sio::{Sio, SioFifo},
        spi::{self},
        watchdog::Watchdog,
        xosc::{setup_xosc_blocking, CrystalOscillator, Dormant, Initialized, Stable},
        Spi, Timer,
    },
    pac::{
        clocks::sleep_en0,
        io_bank0::{dormant_wake_inte::DORMANT_WAKE_INTE_SPEC, DORMANT_WAKE_INTE},
        Peripherals, PLL_SYS, PLL_USB, SPI0,
    },
    XOSC_CRYSTAL_FREQ,
};
use byte_slice_cast::AsByteSlice;
use byteorder::{ByteOrder, LittleEndian};
use lepton::Lepton;
use miniz_oxide::deflate::{core::TDEFLFlush, CompressionLevel};
use numtoa::NumToA;
use rp_pico as bsp;

use cortex_m::{delay::Delay, peripheral::scb};
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
use fugit::{HertzU32, Instant, MicrosDurationU32, MillisDurationU32, RateExtU32};

// Some traits we need
use embedded_hal::{
    can::nb,
    digital::v2::{OutputPin, ToggleableOutputPin},
};
use static_string::StaticString;

use core::{
    cell::{RefCell, UnsafeCell},
    fmt::Error,
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

type InterruptState = Option<(
    Lepton,
    LedPin,
    Delay,
    SioFifo,
    VoSpiState,
    Timer,
    Pin<Gpio27, Output<PushPull>>,
    ClocksManager,
    (
        Option<CrystalOscillator<Stable>>,
        Option<CrystalOscillator<Dormant>>,
    ),
    (
        Option<RingOscillator<Enabled>>,
        Option<RingOscillator<rosc::Dormant>>,
    ),
    (
        Option<PhaseLockedLoop<Locked, PLL_SYS>>,
        Option<PhaseLockedLoop<Disabled, PLL_SYS>>,
    ),
    (
        Option<PhaseLockedLoop<Locked, PLL_USB>>,
        Option<PhaseLockedLoop<Disabled, PLL_USB>>,
    ),
    Watchdog,
)>;

type LedPin = Pin<Gpio25, PushPullOutput>;
static GLOBAL_LEPTON_STATE: Mutex<RefCell<InterruptState>> = Mutex::new(RefCell::new(None));

static FRAME_BUFFER: TripleBuffer = TripleBuffer {
    frame_0: Mutex::new(RefCell::new(Frame::new())),
    frame_1: Mutex::new(RefCell::new(Frame::new())),
    frame_2: Mutex::new(RefCell::new(Frame::new())),
};

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
    static mut LOCAL_LEPTON_STATE: InterruptState = None;

    if LOCAL_LEPTON_STATE.is_none() {
        critical_section::with(|cs| {
            *LOCAL_LEPTON_STATE = GLOBAL_LEPTON_STATE.borrow(cs).take();
        });
    }

    if let Some((
        lepton,
        led_pin,
        delay,
        sio,
        vo_spi_state,
        timer,
        debug_pin,
        clocks,
        xosc,
        rosc,
        pll_sys,
        pll_usb,
        watchdog,
    )) = LOCAL_LEPTON_STATE
    {
        if lepton.spi.vsync.interrupt_status(Interrupt::EdgeHigh) {
            lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);

            // TODO: Try this with ROSC to see if it is fast enough to sleep and wake.

            if let Some(pll_sys_enabled) = pll_sys.0.take() {
                // Switch clock sources away from plls
                debug_pin.set_high().unwrap();
                clocks.peripheral_clock.disable();

                while clocks.system_clock.reset_source_await().is_err() {}
                while clocks.reference_clock.reset_source_await().is_err() {}

                let pll_sys_disabled = pll_sys_enabled.free();
                pll_sys_disabled.pwr.reset();
                //pll_sys_disabled.fbdiv_int.reset();

                let pll_disabled = match PhaseLockedLoop::new(
                    pll_sys_disabled,
                    XOSC_CRYSTAL_FREQ.Hz(),
                    PLL_SYS_125MHZ,
                ) {
                    Ok(p) => p,
                    Err(_) => crate::panic!("Couldn't get disabled PLL"),
                };
                pll_sys.1 = Some(pll_disabled);
                debug_pin.set_low().unwrap();
            }
            if let Some(xosc_enabled) = xosc.0.take() {
                debug_pin.set_low().unwrap();
                lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
                lepton
                    .spi
                    .vsync
                    .set_interrupt_enabled(Interrupt::EdgeHigh, false);
                lepton
                    .spi
                    .vsync
                    .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true); // 0x00080000 - IO_BANK0_DORMANT_WAKE_INTE3_GPIO28_EDGE_HIGH
                lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
                unsafe { xosc_enabled.go_dormant() };
                lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
                let initialized_xosc = unsafe { xosc_enabled.get_initialized() };

                // if initialized_xosc.is_stable() {
                //     debug_pin.set_low().unwrap();
                // }

                // let initialized_xosc =
                //     unsafe { dormant_xosc.initialize_from_dormant_wake(XOSC_CRYSTAL_FREQ.Hz()) };

                // let disabled_xosc = CrystalOscillator::new(dormant_xosc.free());
                // let initialized_xosc;
                // match disabled_xosc.initialize(XOSC_CRYSTAL_FREQ.Hz()) {
                //     Ok(x) => initialized_xosc = x,
                //     Err(_) => crate::panic!("Foo"),
                // };
                let xosc_stable_token;
                //debug_pin.set_low().unwrap();
                loop {
                    let res = initialized_xosc.await_stabilization();
                    if let Ok(res) = res {
                        xosc_stable_token = res;
                        break;
                    }
                }

                xosc.0 = Some(initialized_xosc.get_stable(xosc_stable_token));
                lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
                lepton
                    .spi
                    .vsync
                    .set_interrupt_enabled(Interrupt::EdgeHigh, true);
                lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
                lepton
                    .spi
                    .vsync
                    .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, false);
                debug_pin.set_high().unwrap();
            }

            //info!("Waking up");
            // if let Some(pll_disabled) = pll_sys.1.take() {
            //     let mut peripherals = unsafe { pac::Peripherals::steal() };
            //     let initialized_pll = pll_disabled.initialize(&mut peripherals.RESETS);
            //     let locked_pll_token;
            //     loop {
            //         match initialized_pll.await_lock() {
            //             Ok(l) => {
            //                 locked_pll_token = l;
            //                 break;
            //             }
            //             Err(_) => {}
            //         }
            //     }

            //     let pll_sys_locked = initialized_pll.get_locked(locked_pll_token);

            //     clocks
            //         .system_clock
            //         .configure_clock(&pll_sys_locked, pll_sys_locked.get_freq())
            //         .unwrap();

            //     // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
            //     // Normally choose clk_sys or clk_usb
            //     clocks
            //         .peripheral_clock
            //         .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
            //         .unwrap();
            //     clocks.peripheral_clock.enable();
            //     pll_sys.0 = Some(pll_sys_locked);
            //     debug_pin.set_low().unwrap();
            // }
        }
    }
}

//#[interrupt]
//fn IO_IRQ_BANK0() {
/*
// TODO: Once we're synced, we should be able to immediately ignore certain interrupts.

// Vsync interrupt
static mut LOCAL_LEPTON_STATE: InterruptState = None;

if LOCAL_LEPTON_STATE.is_none() {
    critical_section::with(|cs| {
        *LOCAL_LEPTON_STATE = GLOBAL_LEPTON_STATE.borrow(cs).take();
    });
}

if let Some((
    lepton,
    led_pin,
    delay,
    sio,
    vo_spi_state,
    timer,
    debug_pin,
    clocks,
    xosc,
    rosc,
    pll_sys,
    pll_usb,
    watchdog,
)) = LOCAL_LEPTON_STATE
{
    // We need to recover from dormant here:
    led_pin.set_low().unwrap();
    if let Some(pll_sys_disabled) = pll_sys.1.take() {
        let mut peripherals = unsafe { pac::Peripherals::steal() };
        //info!("Waking up");
        let initialized_pll = pll_sys_disabled.initialize(&mut peripherals.RESETS);
        let locked_pll_token;
        loop {
            match initialized_pll.await_lock() {
                Ok(l) => {
                    locked_pll_token = l;
                    break;
                }
                Err(_) => {}
            }
        }
        led_pin.set_low().unwrap();

        let pll_sys_locked = initialized_pll.get_locked(locked_pll_token);

        clocks
            .system_clock
            .configure_clock(&pll_sys_locked, pll_sys_locked.get_freq())
            .unwrap();

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        clocks
            .peripheral_clock
            .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
            .unwrap();

        pll_sys.0 = Some(pll_sys_locked);
    }

    //if lepton.spi.vsync.interrupt_status(Interrupt::EdgeHigh) {
    let start = MicroSecondsInstant::from_ticks(timer.get_counter_low());
    //lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
    if lepton.is_resetting() {
        warn!("Got vsync during reset");
    }
    // Our interrupt doesn't clear itself.
    // Do that now so we don't immediately jump back to this interrupt handler.

    critical_section::with(|cs| {
        let mut frame = FRAME_BUFFER.frame_0.borrow(cs).borrow_mut();
        let _ = debug_pin.set_high();

        vo_spi_state.started_segment = false;
        let mut discards_before_first_good = 0;
        loop {
            let mut buffer = [0u16; 82];
            let packet = lepton.transfer(&mut buffer).unwrap();

            let packet_header = packet[0];
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            if is_discard_packet {
                discards_before_first_good += 1;
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
                    let _ = debug_pin.set_low();
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
                    //let _ = led_pin.toggle();
                    // let _ = debug_pin.toggle();

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

                    //sio.write(1);

                    break;
                }
            }
            vo_spi_state.prev_packet_id = packet_id;
        }
        let _ = debug_pin.set_low();
    });
    //}

    // NOTE: We really want to do this on the other core, once we're finished.

    // NOTE: Unfortunately going into dormant terminates our debugger session prematurely.

    clocks.peripheral_clock.disable();
    let xosc_stable = xosc.0.take().unwrap();
    //let rosc_stable = rosc.0.take().unwrap();

    // FIXME We may need to be able to set *no* auxsrc here.
    // clocks
    //     .reference_clock
    //     .configure_clock(&rosc_stable, XOSC_CRYSTAL_FREQ.Hz())
    //     .unwrap();

    // clocks
    //     .system_clock
    //     .configure_clock(&rosc_stable, XOSC_CRYSTAL_FREQ.Hz())
    //     .unwrap();
    while clocks.system_clock.reset_source_await().is_err() {}
    while clocks.reference_clock.reset_source_await().is_err() {}
    let pll_sys_taken = pll_sys.0.take().unwrap();

    //info!("{:?}", pll_sys_taken.);
    let pll_sys_disabled = pll_sys_taken.free();

    {
        //info!("Disable PLL_SYS");
        // Actually disable

        pll_sys_disabled.pwr.reset();

        pll_sys_disabled.fbdiv_int.reset();

        // pll_sys_disabled.cs.write(|w| unsafe {
        //     w.refdiv().bits(1);
        //     w
        // });

        // let ref_freq_hz: HertzU32 = XOSC_CRYSTAL_FREQ.Hz();
        // let vco_freq = PLL_SYS_125MHZ.vco_freq;

        // let fbdiv: u16 = vco_freq.to_Hz().checked_div(ref_freq_hz.to_Hz()).unwrap() as u16;

        // pll_sys_disabled.fbdiv_int.write(|w| unsafe {
        //     w.fbdiv_int().bits(fbdiv);
        //     w
        // });
    }

    let pll_disabled =
        match PhaseLockedLoop::new(pll_sys_disabled, XOSC_CRYSTAL_FREQ.Hz(), PLL_SYS_125MHZ) {
            Ok(p) => p,
            Err(_) => crate::panic!("Couldn't get disabled PLL"),
        };
    pll_sys.1 = Some(pll_disabled);

    //info!("Going dormant");
    //xosc.0 = Some(xosc_stable);
    //delay.delay_ms(10);
    //led_pin.set_high().unwrap();
    xosc.1 = Some(unsafe { xosc_stable.dormant() });

    if let Some(dormant_xosc) = xosc.1.take() {
        let disabled_xosc = CrystalOscillator::new(dormant_xosc.free());
        let initialized_xosc;
        match disabled_xosc.initialize(XOSC_CRYSTAL_FREQ.Hz()) {
            Ok(x) => initialized_xosc = x,
            Err(_) => crate::panic!("Foo"),
        };
        // let initialized_xosc =
        //     unsafe { dormant_xosc.initialize_from_dormant_wake(XOSC_CRYSTAL_FREQ.Hz()) };

        // if initialized_xosc.has_bad_bit() {
        //     led_pin.set_low().unwrap();
        // }
        if initialized_xosc.enabled() {
            led_pin.set_high().unwrap();
        }
        let xosc_stable_token;
        loop {
            let res = initialized_xosc.await_stabilization();
            if let Ok(res) = res {
                xosc_stable_token = res;
                break;
            }
        }
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
        let xosc_stable = initialized_xosc.get_stable(xosc_stable_token);
        xosc.0 = Some(xosc_stable);
    }

    // clocks
    //     .system_clock
    //     .configure_clock(pll_sys, 125_000_000.Hz())
    //     .unwrap();
    // Wake up again
    // When we return here, we will be awake, and have a CrystalOscillator<Enabled>, and we need to transition it to stable.
}
*/
//}

#[entry]
fn main() -> ! {
    info!("Program start");

    ///
    let mut peripherals = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(peripherals.WATCHDOG);
    const XOSC_CRYSTAL_FREQ: u32 = 12_000_000; // Typically found in BSP crates

    // let rosc = RingOscillator::new(peripherals.ROSC);
    // let rosc = rosc.initialize();
    // let rosc = rosc.disable();
    // Enable the xosc
    let mut xosc = match setup_xosc_blocking(peripherals.XOSC, XOSC_CRYSTAL_FREQ.Hz()) {
        Ok(xosc) => xosc,
        Err(_) => crate::panic!("xosc"),
    };

    // Start tick in watchdog
    watchdog.enable_tick_generation((XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

    let mut clocks = ClocksManager::new(peripherals.CLOCKS);

    // Configure PLLs
    //                   REF     FBDIV VCO            POSTDIV
    // PLL SYS: 12 / 1 = 12MHz * 125 = 1500MHZ / 6 / 2 = 125MHz
    // PLL USB: 12 / 1 = 12MHz * 40  = 480 MHz / 5 / 2 =  48MHz
    let mut pll_sys = match setup_pll_blocking(
        peripherals.PLL_SYS,
        xosc.operating_frequency().into(),
        PLL_SYS_125MHZ,
        &mut clocks,
        &mut peripherals.RESETS,
    ) {
        Ok(pll_sys) => pll_sys,
        Err(_) => crate::panic!("pll_sys"),
    };

    // Configure clocks
    // CLK_REF = XOSC (12MHz) / 1 = 12MHz
    // CLK SYS = PLL SYS (125MHz) / 1 = 125MHz
    clocks.usb_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();

    clocks
        .system_clock
        .configure_clock(&pll_sys, pll_sys.get_freq())
        .unwrap();

    // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
    // Normally choose clk_sys or clk_usb
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    ////

    let core = pac::CorePeripherals::take().unwrap();

    let mut sio = Sio::new(peripherals.SIO);
    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];

    let sys_freq = clocks.system_clock.freq().to_Hz();
    let mut delay = Delay::new(core.SYST, sys_freq);

    let pins = bsp::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );

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

        // let debug_render_pin: Pin<Gpio26, Output<PushPull>> = pins.gpio26.into_push_pull_output();

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
    let mut enable_pin = pins.gpio2.into_push_pull_output();
    enable_pin.set_high().unwrap();
    let mut lepton = lepton::Lepton::new(
        rp_pico::hal::I2C::i2c0(
            peripherals.I2C0,
            pins.gpio12.into_mode(),
            pins.gpio13.into_mode(),
            400.kHz(),
            &mut peripherals.RESETS,
            clocks.peripheral_clock.freq(),
        ),
        Spi::new(peripherals.SPI1).init(
            &mut peripherals.RESETS,
            clocks.peripheral_clock.freq(), // 125_000_000
            20_000_000.Hz(),
            &embedded_hal::spi::MODE_3,
        ),
        pins.gpio9.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio8.into_mode(),
        pins.gpio10.into_mode(),
        pins.gpio3.into_mode(),
    );
    let timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
    // Reboot every time we start up the program to get to a known state.

    lepton.reboot(&mut delay, true);

    let mut alt_pin = pins.gpio26.into_push_pull_output();
    let mut led_pin: LedPin = pins.led.into_push_pull_output();
    //let _ = led_pin.set_low();
    let debug_pin: Pin<Gpio27, Output<PushPull>> = pins.gpio27.into_push_pull_output();
    lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
    //peripherals.IO_BANK0.dormant_wake_inte
    // FIXME - Need to set this
    //DORMANT_WAKE_INTE.
    critical_section::with(|cs| {
        GLOBAL_LEPTON_STATE.borrow(cs).replace(Some((
            lepton,
            led_pin,
            delay,
            sio.fifo,
            VoSpiState::new(),
            timer,
            debug_pin,
            clocks,
            (Some(xosc), None),
            (None, None),
            (Some(pll_sys), None),
            (None, None),
            watchdog,
        )));
    });
    alt_pin.set_high().unwrap();
    unsafe {
        pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
    }
    // led_pin.set_high().unwrap();
    // info!("Sleeping");
    // delay.delay_ms(50);
    /*
    loop {
        let pll_sys_disabled = pll_sys.free();
        pll_sys_disabled.pwr.reset();
        //pll_sys_disabled.fbdiv_int.reset();

        let pll_disabled =
            match PhaseLockedLoop::new(pll_sys_disabled, XOSC_CRYSTAL_FREQ.Hz(), PLL_SYS_125MHZ) {
                Ok(p) => p,
                Err(_) => crate::panic!("Couldn't get disabled PLL"),
            };

        let dormant_xosc = unsafe { xosc.dormant() };

        led_pin.set_low().unwrap();
        let disabled_xosc = CrystalOscillator::new(dormant_xosc.free());
        let initialized_xosc;
        match disabled_xosc.initialize(XOSC_CRYSTAL_FREQ.Hz()) {
            Ok(x) => initialized_xosc = x,
            Err(_) => crate::panic!("Foo"),
        };
        let xosc_stable_token;
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);

        loop {
            let res = initialized_xosc.await_stabilization();
            if let Ok(res) = res {
                xosc_stable_token = res;
                break;
            }
        }

        //lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
        xosc = initialized_xosc.get_stable(xosc_stable_token);
        delay.delay_ms(5);

        //info!("Waking up");
        let initialized_pll = pll_disabled.initialize(&mut peripherals.RESETS);
        let locked_pll_token;
        loop {
            match initialized_pll.await_lock() {
                Ok(l) => {
                    locked_pll_token = l;
                    break;
                }
                Err(_) => {}
            }
        }

        let pll_sys_locked = initialized_pll.get_locked(locked_pll_token);

        clocks
            .system_clock
            .configure_clock(&pll_sys_locked, pll_sys_locked.get_freq())
            .unwrap();

        // CLK PERI = clk_sys. Used as reference clock for Peripherals. No dividers so just select and enable
        // Normally choose clk_sys or clk_usb
        clocks
            .peripheral_clock
            .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
            .unwrap();
        pll_sys = pll_sys_locked;

        //cortex_m::asm::wfi();
        // We just got woken by the interrupt, now recover.
    }
    */
    loop {
        cortex_m::asm::wfi();
    }
}
