#![no_std]
#![no_main]

//mod draw_debug_output;
mod lepton;
mod static_string;
mod utils;
use bsp::{
    entry,
    hal::{
        clocks::{Clock, ClockSource, ClocksManager, StoppableClock},
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

use byteorder::{BigEndian, ByteOrder, LittleEndian};
use crc::{Crc, CRC_16_XMODEM};
use lepton::Lepton;
use pio::{Instruction, InstructionOperands};
use rp2040_hal::{
    dma::{bidirectional, single_buffer, DMAExt},
    gpio::{BusKeepInput, FloatingInput, Input, PullDownInput, PullUpInput},
    pio::{PIOBuilder, ShiftDirection},
    prelude::_rphal_pio_PIOExt,
};
use rp_pico as bsp;

use cortex_m::{
    asm::wfi,
    delay::Delay,
    prelude::{_embedded_hal_blocking_spi_Transfer, _embedded_hal_spi_FullDuplex},
};
use defmt::*;
use defmt_rtt as _;

use embedded_sdmmc::{TimeSource, Timestamp};
use fugit::RateExtU32;
use ili9341::{DisplaySize240x320, Ili9341, Orientation};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.

// use sparkfun_pro_micro_rp2040 as bsp;
use fugit::{HertzU32, Instant};

// Some traits we need
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use core::{borrow::BorrowMut, cell::RefCell, time::Duration};
use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use critical_section::{CriticalSection, Mutex};

use crate::core1::{any_as_u8_slice, core1_task};

static mut CORE1_STACK: Stack<4096> = Stack::new();

mod core1;

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

pub type FramePacketData = [u8; 160];
pub struct FrameSeg((u8, [FramePacketData; 61]));

impl FrameSeg {
    pub const fn new() -> FrameSeg {
        FrameSeg((0, [[0u8; 160]; 61]))
    }
}

struct FrameSegs {
    frame: [FrameSeg; 4],
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
    frame_0: Mutex<RefCell<FrameSeg>>,
    frame_1: Mutex<RefCell<FrameSeg>>,
    frame_2: Mutex<RefCell<FrameSeg>>,
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

// static FRAME_BUFFER: TripleBuffer = TripleBuffer {
//     frame_0: Mutex::new(RefCell::new(FrameSeg::new())),
//     frame_1: Mutex::new(RefCell::new(FrameSeg::new())),
//     frame_2: Mutex::new(RefCell::new(FrameSeg::new())),
// };

// struct CorePriority(Mutex<RefCell<usize>>);

// static FIFO: CorePriority = CorePriority(Mutex::new(RefCell::new(0)));

// impl CorePriority {
//     fn aquire(&self) {
//         critical_section::with(|cs| {
//             *self.0.borrow_mut(cs) = 1;
//         });
//     }

//     fn release(&self) {
//         critical_section::with(|cs| *self.0.borrow_mut(cs) = 0);
//     }
// }

pub static FRAME_BUFFER: DoubleBuffer = DoubleBuffer {
    front: Mutex::new(RefCell::new(FrameSeg::new())),
    back: Mutex::new(RefCell::new(FrameSeg::new())),
};

pub struct DoubleBuffer {
    pub front: Mutex<RefCell<FrameSeg>>,
    pub back: Mutex<RefCell<FrameSeg>>,
}

impl DoubleBuffer {
    pub fn swap(&self) {
        critical_section::with(|cs| {
            self.front.borrow(cs).swap(self.back.borrow(cs));
        });
    }
}

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
static mut tx_buf: [u32; 1024] = [0u32; 1024];

#[entry]
fn main() -> ! {
    info!("Program start");
    let handoff_to_core_1 = true;

    // Rosc measured speed seems to be 145_332_000 - see how this varies between boards.
    let rosc_clock_freq: HertzU32 = 133_000_000.Hz();
    //let rosc_clock_freq: HertzU32 = 75_000_000.Hz();
    //let rosc_clock_freq: HertzU32 = 250_000_000.Hz();

    // Seems to give a reported baud-rate of 22Mhz, in practice it may be lower, since we're seeing 2MB/s transfers,
    // and at 20Mhz we'd expect a maximum of 2.5MB/s
    let spi_clock_freq: HertzU32 = 30_000_000.Hz();

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

    let measured_rosc_speed = utils::rosc_frequency_count_hz();
    info!("Rosc speed {}", measured_rosc_speed);
    // Now raise the clock speed of rosc.
    //if measured_rosc_speed > 133_000_000 {
    rosc.set_operating_frequency(rosc_clock_freq);

    clocks
        .system_clock
        .configure_clock(&rosc, rosc.get_freq())
        .unwrap();

    let measured_speed = utils::rosc_frequency_count_hz();
    info!(
        "Measured {}, asked for {}",
        measured_speed,
        rosc_clock_freq.to_Hz()
    );
    //}
    // Do this a few more times until the rosc speed is close to the actual speed measured.

    info!("System clock speed {}", clocks.system_clock.freq().to_MHz());
    let _xosc_disabled = xosc.disable();
    clocks.usb_clock.disable();
    clocks.gpio_output0_clock.disable();
    clocks.gpio_output1_clock.disable();
    clocks.gpio_output2_clock.disable();
    clocks.gpio_output3_clock.disable();
    clocks.adc_clock.disable();
    clocks.rtc_clock.disable();

    // TODO: Are PLLs disabled by default?

    clocks
        .peripheral_clock
        .configure_clock(&clocks.system_clock, clocks.system_clock.freq())
        .unwrap();

    let core = pac::CorePeripherals::take().unwrap();
    let mut sio = Sio::new(peripherals.SIO);

    let mut mc = Multicore::new(&mut peripherals.PSM, &mut peripherals.PPB, &mut sio.fifo);
    let cores = mc.cores();
    let core1 = &mut cores[1];
    if handoff_to_core_1 {
        let _test = core1.spawn(unsafe { &mut CORE1_STACK.mem }, move || core1_task());
    }

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

    let mut packet_pin = pins.gpio22.into_push_pull_output();

    //let mut side_pin = pins.gpio6.into_mode::<rp2040_hal::gpio::FunctionPio0>();

    enable_pin.set_high().unwrap();

    /*
    {
        let dma = peripherals.DMA.split(&mut peripherals.RESETS);
        let (mut pio0, sm0, _, _, _) = peripherals.PIO0.split(&mut peripherals.RESETS);
        let program_with_defines = pio_proc::pio_file!("./src/soft_spi_slave.pio");
        let installed = pio0.install(&program_with_defines.program).unwrap();
        let val = 1u32 << 24 | 2u32 << 16 | 3u32 << 8 | 255u32;
        let use_pio = false;

        if use_pio {
            let _spi_clk = pins.gpio18.into_mode::<rp2040_hal::gpio::FunctionPio0>();
            let _spi_miso = pins.gpio19.into_mode::<rp2040_hal::gpio::FunctionPio0>();
            //let _spi_miso = pins.gpio19.into_mode::<rp2040_hal::gpio::FunctionPio0>();
            let _spi_cs = pins.gpio17.into_mode::<rp2040_hal::gpio::FunctionPio0>();
            let sclk_id = 18;
            let miso_id = 19;

            // Build the pio program and set pin both for set and side set!
            // We are running with the default divider which is 1 (max speed)
            let (mut sm, _, mut tx) = PIOBuilder::from_program(installed)
                //.side_set_pin_base(miso_id)
                .out_pins(miso_id, 1)
                .out_shift_direction(ShiftDirection::Left)
                .pull_threshold(32)
                .autopush(true)
                .autopull(true)
                .build(sm0);

            // TODO: See if I can get any faster with SPI?
            sm.set_pindirs([(miso_id, rp2040_hal::pio::PinDir::Output)]);

            let mut sm = sm.start();
            unsafe {
                for i in 0..tx_buf.len() {
                    tx_buf[i] = val;
                }
                info!("{:#034b}", tx_buf[1]);
                info!("Begin transfer");
                let mut dma_ch0 = dma.ch0;
                loop {
                    let tx_transfer = single_buffer::Config::new(dma_ch0, &tx_buf, tx).start();
                    let (ch0, tx_buf_2, tx_ret) = tx_transfer.wait();
                    tx_buf = *tx_buf_2;
                    tx = tx_ret;
                    dma_ch0 = ch0;
                }
                info!("End transfer");
            }
        } else {
            let _spi_mosi = pins.gpio16.into_mode::<FunctionSpi>();
            let _spi_miso = pins.gpio19.into_mode::<FunctionSpi>();
            let _spi_sck = pins.gpio18.into_mode::<FunctionSpi>();
            let _spi_cs = pins.gpio17.into_mode::<FunctionSpi>();

            let spi: Spi<_, SPI0, 8> = Spi::new(peripherals.SPI0);
            let mut spi = spi.init(
                &mut peripherals.RESETS,
                133_000_000.Hz(),
                12_000_000.Hz(),
                &embedded_hal::spi::MODE_3,
                true,
            );
            unsafe {
                for i in 0..tx_buf.len() {
                    tx_buf[i] = val;
                }
                let mut buf_u8 = unsafe { any_as_u8_slice(&tx_buf) };
                info!("{:#034b}", tx_buf[1]);
                info!("Begin transfer");

                let mut dma_ch0 = dma.ch0;
                loop {
                    let tx_transfer = single_buffer::Config::new(dma_ch0, &*buf_u8, spi).start();
                    let (ch0, tx_buf_2, tx_ret) = tx_transfer.wait();
                    buf_u8 = tx_buf_2;
                    spi = tx_ret;
                    dma_ch0 = ch0;
                }

                //let tx_transfer = single_buffer::Config::new(dma.ch0, &*buf_u8, spi).start();
                //let (ch0, tx_buf_2, spi) = tx_transfer.wait();
                info!("End transfer");
            }
            }

        loop {
            //tx.write(val);
            wfi();
        }
    }*/

    info!("Freq {:?}", clocks.peripheral_clock.freq().to_Hz());
    let spi = Spi::new(peripherals.SPI1).init(
        &mut peripherals.RESETS,
        clocks.peripheral_clock.freq(), // 125_000_000
        spi_clock_freq,
        &embedded_hal::spi::MODE_3,
        false,
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
    let crc_check = Crc::<u16>::new(&CRC_16_XMODEM);

    let do_crc_check = false;
    let mut crc_buffer = [0u8; 164];
    'frame_loop: loop {
        debug_awake_pin.set_low().unwrap();
        // NOTE: Go into dormant state
        let dormant_rosc = unsafe { rosc.dormant() };

        // NOTE: We got an interrupt on Vsync pin, now we're waking up again!
        //  Re-enable the rosc so we can resume operation.
        let disabled_rosc = RingOscillator::new(dormant_rosc.free());
        let initialized_rosc = disabled_rosc.initialize();
        rosc = initialized_rosc;
        // info!("Woke up");
        debug_awake_pin.set_high().unwrap();
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);

        // NOTE: - it's possible to greedily clock out all 4 segments in the first vsync pulse, but
        //  the lepton module seems to not be able to feed all 4 segments out cleanly, so it outputs
        //  a couple of dozen discard packets before each frame segment.  This uses slightly more
        //  power to read out sequentially than just waiting for the vsync pulse for each segment,
        //  processing it with no discards, and then going dormant again.
        rosc.set_operating_frequency(rosc_clock_freq);

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
        //lepton.cs_low();
        //lepton.start_clock(clocks.peripheral_clock.freq(), spi_clock_freq);

        //{
        //    let cs = unsafe { CriticalSection::new() };
        if handoff_to_core_1 {
            if !sio.fifo.is_write_ready() {
                info!("fifo full");
            }
            sio.fifo.write(1);
        }
        //critical_section::with(|_cs| {
        // Tell core1 we're awake, and that it can start processing the previous segment.

        'scanline: loop {
            // Read out 1 scanline of the video frame

            // TODO: If we want to do CRC checks, ideally we'd interleave that work with the transfer.
            // Also, it would be good to have this be a DMA driven thing.
            packet_pin.set_high().unwrap();
            lepton.transfer(&mut buffer).unwrap();
            packet_pin.set_low().unwrap();
            let packet_header = buffer[0];
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            if is_discard_packet {
                discards_before_first_good += 1;
                continue 'scanline;
            }
            let packet_id = (packet_header & 0x0fff) as isize;
            if packet_id != prev_packet_id + 1 && segment_num != 0 {
                warn!("Expected {}, got {}", prev_packet_id + 1, packet_id);
            }

            if do_crc_check {
                // Check crc
                let crc = buffer[1].to_le();
                BigEndian::write_u16_into(&buffer, &mut crc_buffer);
                crc_buffer[0] = crc_buffer[0] & 0x0f;
                crc_buffer[2] = 0;
                crc_buffer[3] = 0;
                crate::assert_eq!(crc_check.checksum(&crc_buffer), crc);
            }

            prev_packet_id = packet_id;
            if packet_id == 20 {
                // NOTE: Packet 20 is the only packet that has a valid segment id in its header.
                segment_num = packet_header >> 12;
                if segment_num == 1 {
                    got_sync = true;
                }
                if segment_num == 0 {
                    // TODO - entire segment should be discarded.
                }
            }

            // Copy the line out to the appropriate place in the current segment buffer.
            if handoff_to_core_1 {
                critical_section::with(|cs| {
                    let mut frame_seg = FRAME_BUFFER.front.borrow_ref_mut(cs);
                    (frame_seg.0).0 = segment_num as u8;
                    if (packet_id as usize) < (frame_seg.0).1.len() {
                        LittleEndian::write_u16_into(
                            &buffer[2..],
                            &mut (frame_seg.0).1[packet_id as usize][..],
                        );
                        //(frame_seg.0).1[packet_id as usize].copy_from_slice(&buffer[2..]);
                    } else {
                        warn!("Invalid packet id {}", packet_id);
                    }
                });
            }

            if packet_id == 60 {
                prev_packet_id = -1;
                last_segment_was_4 = segment_num == 4;
            }
            if packet_id == 60 {
                // NOTE: To clock out all segments at once: && (segment_num == 4 || !started)
                if segment_num == 4 {
                    // We've finished reading out the frame.
                    debug_frame_read_pin.set_low().unwrap();
                    // We want to hand-off to core1, which can send out our debug frame over spi to the
                    // raspberry pi.

                    // TODO: Work out how to successfully come out of power-off mode for the lepton
                    //  - This may require a board with access to the reset and pwr pins, rather
                    //  than just "enable"
                }
                break 'scanline;
            }
        }
        //}

        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
        // Wait for core1 to finish processing.
        //});
        //info!("Waiting on core1");
        if handoff_to_core_1 {
            let core_1_completed = sio.fifo.read_blocking();
            // Swap buffer
            FRAME_BUFFER.swap();
        }
        // Setting the lepton baud-rate to 0 while dormant seems to help with power usage a tiny bit.
        //lepton.stop_clock();
        //lepton.cs_high();
        lepton.spi.vsync.clear_interrupt(Interrupt::EdgeHigh);
    }
}
