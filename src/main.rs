#![no_std]
#![no_main]

mod draw_debug_output;
mod lepton;
mod static_string;
mod utils;
use bsp::{
    entry,
    hal::{
        clocks::{
            Clock, ClockSource, ClocksManager, StoppableClock,
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
        Peripherals, CLOCKS, PLL_SYS, PLL_USB, SPI0,
    },
    XOSC_CRYSTAL_FREQ,
};

use lepton::Lepton;
use rp_pico as bsp;

use cortex_m::{delay::Delay};
use defmt::*;
use defmt_rtt as _;

use embedded_sdmmc::{TimeSource, Timestamp};
use ili9341::{DisplaySize240x320, Ili9341, Orientation};

use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

// use sparkfun_pro_micro_rp2040 as bsp;
use fugit::{HertzU32, Instant, RateExtU32};

// Some traits we need
use embedded_hal::{
    digital::v2::{OutputPin},
};

use core::{
    cell::{RefCell},
    time::Duration,
};
use critical_section::Mutex;

static mut CORE1_STACK: Stack<4096> = Stack::new();

pub const SEGMENT_LENGTH: usize = (160 * 122) / 4;

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

type LedPin = Pin<Gpio25, PushPullOutput>;

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
    current_frame_num: u32,
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
            current_frame_num: 0,
        }
    }

    pub fn reset(&mut self) {
        self.prev_packet_id = -1;
        self.started_segment = false;
        self.current_segment_num = 0;
    }
}

type MicroSecondsInstant = Instant<u32, 1, 1_000_000>;

#[entry]
fn main() -> ! {
    info!("Program start");

    // Rosc measured speed seems to be 145_332_000 - see how this varies between boards.
    let rosc_clock_freq: HertzU32 = 133_000_000.Hz();

    // Seems to give a reported baud-rate of 22Mhz, in practice it may be lower, since we're seeing 2MB/s transfers,
    // and at 20Mhz we'd expect a maximum of 2.5MB/s
    let spi_clock_freq: HertzU32 = 25_000_000.Hz();

    let mut peripherals = Peripherals::take().unwrap();
    //Enable the xosc, so that we can measure the rosc clock speed
    let xosc = match setup_xosc_blocking(peripherals.XOSC, XOSC_CRYSTAL_FREQ.Hz()) {
        Ok(xosc) => xosc,
        Err(_) => crate::panic!("xosc"),
    };
    let mut clocks = ClocksManager::new(peripherals.CLOCKS);

    clocks
        .reference_clock
        .configure_clock(&xosc, XOSC_CRYSTAL_FREQ.Hz())
        .unwrap();

    let rosc = RingOscillator::new(peripherals.ROSC);
    let mut rosc = rosc.initialize();

    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();
    info!("Rosc speed {}", utils::rosc_frequency_count_hz());
    // Now raise the clock speed of rosc.
    rosc.set_operating_frequency(rosc_clock_freq);

    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    let measured_speed = utils::rosc_frequency_count_hz();
    info!("Measured {}, asked for {}", measured_speed, rosc_clock_freq.to_Hz());
    // Do this a few more times until the rosc speed is close to the actual speed measured.

    info!("System clock speed {}", clocks.system_clock.freq().to_MHz());
    let _xosc_disabled = xosc.disable();
    clocks.usb_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();
    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

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
    let mut enable_pin = pins.gpio2.into_push_pull_output();
    let mut debug_awake_pin: Pin<Gpio27, Output<PushPull>> = pins.gpio27.into_push_pull_output();
    let mut debug_frame_read_pin = pins.gpio26.into_push_pull_output();
    enable_pin.set_high().unwrap();

    info!("Freq {:?}", clocks.peripheral_clock.freq().to_Hz());
    let (spi, speed) = Spi::new(peripherals.SPI1).init(
        &mut peripherals.RESETS,
        clocks.peripheral_clock.freq(), // 125_000_000
        spi_clock_freq,
        &embedded_hal::spi::MODE_3,
    );
    let mut lepton = Lepton::new(
        rp_pico::hal::I2C::i2c0(
            peripherals.I2C0,
            pins.gpio12.into_mode(),
            pins.gpio13.into_mode(),
            400.kHz(),
            &mut peripherals.RESETS,
            clocks.peripheral_clock.freq(),
        ),
        spi,
        pins.gpio9.into_push_pull_output(),
        pins.gpio11.into_push_pull_output(),
        pins.gpio8.into_mode(),
        pins.gpio10.into_mode(),
        pins.gpio3.into_mode(),
    );
    info!("Baud-rate {:?}", speed.to_MHz());
    let timer = Timer::new(peripherals.TIMER, &mut peripherals.RESETS);
    // Reboot every time we start up the program to get to a known state.
    lepton.reboot(&mut delay, true);

    lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
    // Set wake from dormant on vsync
    lepton
        .spi
        .vsync
        .set_interrupt_enabled_dormant_wake(Interrupt::EdgeHigh, true);
    debug_frame_read_pin.set_high().unwrap();
    let mut vo_spi_state = VoSpiState::new();
    let start = MicroSecondsInstant::from_ticks(timer.get_counter_low());
    let mut last_segment_was_4 = false;
    let mut got_sync = false;
    let mut skipped_segs = 0;
    warn!("Syncing");
    while !got_sync {
        lepton.get_frame_sync(&mut vo_spi_state, &mut delay, &timer, start, &mut got_sync);
    }
    warn!("Got sync, entering frame/sleep loop");

    let mut buffer = [0u16; 82];
    let mut n_frames = 0;

    let mut prev_packet_id = -1;

    'frame_loop: loop {
        debug_awake_pin.set_low().unwrap();
        // NOTE: Go into dormant state
        let dormant_rosc = unsafe { rosc.dormant() };

        // NOTE: We got an interrupt on Vsync pin, now we're waking up again!
        //  Re-enable the rosc so we can resume operation.
        let disabled_rosc = RingOscillator::new(dormant_rosc.free());
        let initialized_rosc = disabled_rosc.initialize();
        rosc = initialized_rosc;
        debug_awake_pin.set_high().unwrap();
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);

        let mut discards_before_first_good = 0;
        let mut segment_num = 0;

        if last_segment_was_4 {
            skipped_segs += 1;
        }
        if last_segment_was_4 && skipped_segs < 8 {
            // NOTE: After segment 4 of a frame, there are 7 segments that can be skipped
            //  completely before the next real frame starts.  Contrary to the lepton documentation,
            //  sync is not lost if we decide to just ignore these rather than clocking out their
            //  data packets, and doing this saves power.
            //  On the 8th segment, which is the last junk segment before we start getting real ones,
            //  we wake up and start getting rid of the discard buffer.  If we skipped the 8th segment,
            //  we wouldn't have time to catch up after discarding all the junk packets, due to the
            //  limited SPI bandwidth available.
            continue 'frame_loop;
        }
        skipped_segs = 0;
        if last_segment_was_4 {
            // NOTE: We've successfully read out at least one frame, and have skipped 7 of the 8
            //  blank segments.  Now we still have a bunch of discards that are buffered up, since
            //  the lepton buffers N (where N is some number in the low hundreds) packets so that
            //  reads can be delayed somewhat and still catch up and maintain sync.  Here we just
            //  wait a little bit longer before starting to read out packets so that the number
            //  we have to read out only to discard is lower (around 25), which saves some power.
            delay.delay_us(5100);

            // Indicate that we're starting to read out a new frame
            debug_frame_read_pin.set_high().unwrap();
        }
        // NOTE: - it's possible to greedily clock out all 4 segments in the first vsync pulse, but
        //  the lepton module seems to not be able to feed all 4 segments out cleanly, so it outputs
        //  a couple of dozen discard packets before each frame segment.  This uses slightly more
        //  power to read out sequentially than just waiting for the vsync pulse for each segment,
        //  processing it with no discards, and then going dormant again.
        rosc.set_operating_frequency(rosc_clock_freq);
        lepton.cs_low();
        lepton.start_clock(clocks.peripheral_clock.freq(), spi_clock_freq);
        'scanline: loop {
            // Read out 1 scanline of the video frame
            let packet = lepton.transfer_block(&mut buffer).unwrap();
            let packet_header = packet[0];
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            if is_discard_packet {
                discards_before_first_good += 1;
                continue 'scanline;
            }
            let packet_id = (packet_header & 0x0fff) as isize;
            if packet_id != prev_packet_id + 1 && segment_num != 0 {
                warn!("Expected {}, got {}", prev_packet_id + 1, packet_id);
            }
            prev_packet_id = packet_id;
            if packet_id == 20 {
                // NOTE: Packet 20 is the only packet that has a valid segment id in its header.
                segment_num = packet_header >> 12;
                if segment_num == 1 {
                    got_sync = true;
                }
            }
            if packet_id == 60 {
                prev_packet_id = -1;
                last_segment_was_4 = segment_num == 4;
            }
            if packet_id == 60  { // NOTE: To clock out all segments at once: && (segment_num == 4 || !started)
                if segment_num == 4 {
                    // We've finished reading out the frame.
                    debug_frame_read_pin.set_low().unwrap();

                    // TODO: Work out how to successfully come out of power-off mode for the lepton
                    //  - This may require a board with access to the reset and pwr pins, rather
                    //  than just "enable"
                }
                break 'scanline;
            }
        }
        // Setting the lepton baud-rate to 0 while dormant seems to help with power usage a tiny bit.
        lepton.stop_clock();
        lepton.cs_high();
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
    }
}
