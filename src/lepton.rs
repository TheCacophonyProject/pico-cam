use core::convert::Infallible;

use byte_slice_cast::AsByteSlice;
use byteorder::{BigEndian, ByteOrder, LittleEndian};
use cortex_m::prelude::{_embedded_hal_blocking_i2c_Write, _embedded_hal_blocking_spi_Transfer};
use cortex_m::{delay::Delay, prelude::_embedded_hal_blocking_i2c_WriteRead};
use cortex_m::asm::nop;
use defmt::{info, warn};
use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};
use fugit::{HertzU32, RateExtU32};

use rp2040_hal::gpio::bank0::{Gpio3, Gpio5};
use rp2040_hal::{pac, Sio, Timer};
use rp2040_hal::gpio::Pins;
use rp_pico::hal::gpio::bank0::{Gpio10, Gpio11, Gpio28, Gpio8, Gpio9};
use rp_pico::hal::gpio::{FloatingInput, FunctionSpi, Interrupt, Output, PushPull};
use rp_pico::hal::spi::Enabled;
use rp_pico::hal::{Spi, I2C as I2CInterface};
use rp_pico::pac::SPI1;
use rp_pico::{
    hal::gpio::{
        bank0::{Gpio12, Gpio13},
        Function, Pin, I2C,
    },
    pac::I2C0,
};

use crate::{Frame, MicroSecondsInstant, VoSpiState, SEGMENT_LENGTH};

pub enum LeptonCommandType {
    Get = 0b0000_0000_0000_00_00,
    Set = 0b0000_0000_0000_00_01,
    Run = 0b0000_0000_0000_00_10,
}

pub type LeptonRegister = [u8; 2];
pub type LeptonSynthesizedCommand = ([u8; 2], u16);
pub type LeptonCommand = (u16, u16);
const LEPTON_BOOTED: u16 = 0b100; // If the camera successfully boots up, this bit is set to 1. If this bit is 0, then the camera has not booted. A host can monitor this bit to learn when the camera has booted.
const LEPTON_BUSY: u16 = 0b000;
const LEPTON_BOOT_MODE: u16 = 0b010; // For normal operation, this bit will be set to 1, indicating successful boot from internal ROM.
const LEPTON_ADDRESS: u8 = 0x2a;

const LEPTON_POWER_ON_REGISTER: LeptonRegister = lepton_register_val(0x0000);
const LEPTON_STATUS_REGISTER: LeptonRegister = lepton_register_val(0x0002);
const LEPTON_COMMAND_ID_REGISTER: LeptonRegister = lepton_register_val(0x0004);
const LEPTON_DATA_LENGTH_REGISTER: LeptonRegister = lepton_register_val(0x0006);
const LEPTON_DATA_0_REGISTER: LeptonRegister = lepton_register_val(0x0008);

const LEPTON_COMMAND_OEM_BIT: u16 = 0b0100_0000_0000_0000;

const LEPTON_SUB_SYSTEM_VID: u16 = 0b0000_0011_0000_0000;
const LEPTON_SUB_SYSTEM_OEM: u16 = 0b0000_1000_0000_0000; // | LEPTON_COMMAND_OEM_BIT; // Requires protection bit set
const LEPTON_SUB_SYSTEM_RAD: u16 = 0b0000_1110_0000_0000; // | LEPTON_COMMAND_OEM_BIT; // Requires protection bit set
const LEPTON_SUB_SYSTEM_AGC: u16 = 0b0000_0001_0000_0000;
const LEPTON_SUB_SYSTEM_SYS: u16 = 0b0000_0010_0000_0000;

/*
    4.3.1 AGC, VID, and SYS Module Command ID Generation
    AGC, VID, and SYS modules no not require a protection bit to be set before the camera will
    recognize it as a valid command so the protection bit value is 0x0000. For example, the AGC
    Module ID is 0x0100; the ACG Enable command ID Base is 0x00. To retrieve the current AGC
    enable state, issue a Get command specifying command type of 0x0.
    The AGC module protection bit not defined so the value is 0x0000.
    The Command ID is synthesized as follows: Module ID + Command ID Base + Type + Protection Bit value= Command ID.
    So in this example, 0x0100 + 0x00 + 0x0 + 0x0000 = 0x0100 and this is the Get AGC Enable State Command ID.
    To set the AGC enable state to enabled, the command type is 0x1 and
    thus the Command ID is 0x100 + 0x00 + 0x1 + 0x0000 = 0x0101.
*/

const LEPTON_SYS_PING_CAMERA: LeptonCommand = (0x0000, 0);
const LEPTON_SYS_STATUS: LeptonCommand = (0x0004, 4);
const LEPTON_SYS_GET_SERIAL: LeptonCommand = (0x0008, 4);
const LEPTON_SYS_NUM_FRAMES_TO_AVERAGE: LeptonCommand = (0x0024, 2);
const LEPTON_SYS_FFC_STATUS: LeptonCommand = (0x0044, 2);
const LEPTON_SYS_STATS: LeptonCommand = (0x002C, 4);
const LEPTON_SYS_TELEMETRY_ENABLE_STATE: LeptonCommand = (0x0018, 2);
const LEPTON_SYS_TELEMETRY_LOCATION: LeptonCommand = (0x001C, 2);
const LEPTON_VID_OUTPUT_FORMAT: LeptonCommand = (0x0030, 2);
const LEPTON_VID_FOCUS_METRIC: LeptonCommand = (0x0018, 2);
const LEPTON_VID_FOCUS_ENABLE_STATE: LeptonCommand = (0x000C, 2);
const LEPTON_AGC_ENABLE_STATE: LeptonCommand = (0x0000, 2);
const LEPTON_OEM_GPIO_MODE: LeptonCommand = (0x0054, 2);
const LEPTON_OEM_GPIO_VSYNC_PHASE_DELAY: LeptonCommand = (0x0058, 2);
const LEPTON_RAD_SPOT_METER_ROI: LeptonCommand = (0x00CC, 4);
const LEPTON_RAD_TLINEAR_ENABLE_STATE: LeptonCommand = (0x00C0, 2);
const LEPTON_OEM_REBOOT: LeptonCommand = (0x0040, 0);
const LEPTON_OEM_BAD_PIXEL_REPLACEMENT: LeptonCommand = (0x006C, 2);
const LEPTON_OEM_TEMPORAL_FILTER: LeptonCommand = (0x0070, 2);
const LEPTON_OEM_COLUMN_NOISE_FILTER: LeptonCommand = (0x0074, 2);
const LEPTON_OEM_PIXEL_NOISE_FILTER: LeptonCommand = (0x0078, 2);
const LEP_OEM_VIDEO_OUTPUT_SOURCE: LeptonCommand = (0x002c, 2);
const LEPTON_OEM_POWER_DOWN: LeptonCommand = (0x0000, 0);
const LEPTON_OEM_VIDEO_OUTPUT_ENABLE: LeptonCommand = (0x0024, 2);

// Should be able to just read into these automatically.
const LEPTON_DATA_1_REGISTER: LeptonRegister = lepton_register_val(0x000A);
// const LEPTON_DATA_2_REGISTER: LeptonRegister = lepton_register_val(0x000C);
// const LEPTON_DATA_3_REGISTER: LeptonRegister = lepton_register_val(0x000E);
// const LEPTON_DATA_4_REGISTER: LeptonRegister = lepton_register_val(0x0010);
// const LEPTON_DATA_5_REGISTER: LeptonRegister = lepton_register_val(0x0012);
// const LEPTON_DATA_6_REGISTER: LeptonRegister = lepton_register_val(0x0014);
// const LEPTON_DATA_7_REGISTER: LeptonRegister = lepton_register_val(0x0016);
// const LEPTON_DATA_8_REGISTER: LeptonRegister = lepton_register_val(0x0018);
// const LEPTON_DATA_9_REGISTER: LeptonRegister = lepton_register_val(0x001A);
// const LEPTON_DATA_10_REGISTER: LeptonRegister = lepton_register_val(0x001C);
// const LEPTON_DATA_11_REGISTER: LeptonRegister = lepton_register_val(0x001E);
// const LEPTON_DATA_12_REGISTER: LeptonRegister = lepton_register_val(0x0020);
// const LEPTON_DATA_13_REGISTER: LeptonRegister = lepton_register_val(0x0022);
// const LEPTON_DATA_14_REGISTER: LeptonRegister = lepton_register_val(0x0024);
// const LEPTON_DATA_15_REGISTER: LeptonRegister = lepton_register_val(0x0026);

enum LeptonError {
    Ok = 0,                      // Camera ok
    Error = -1,                  // Camera general error
    NotReady = -2,               // Camera not ready error
    RangeError = -3,             // Camera range error
    ChecksumError = -4,          // Camera checksum error
    BadArgPointerError = -5,     // Camera Bad argument error
    DataSizeError = -6,          // Camera byte count error
    UndefinedFunctionError = -7, // Camera undefined function error
    FunctionNotSupported = -8,   // Camera function not yet supported error
    DataOutOfRange = -9,         // Camera input DATA is out of valid range error
    CommandNotAllowed = -11,     // Camera unable to execute command due to current camera state

    OtpWriteError = -15,         // Camera OTP write error
    OtpReadError = -16,          // double bit error detected (uncorrectible)
    OtpNotProgrammedError = -18, // Flag read as non-zero

    ErrorI2CBusNotReady = -20,     // I2C Bus Error - Bus Not Avaialble
    ErrorI2CBufferOverflow = -22,  // I2C Bus Error - Buffer Overflow
    ErrorI2CArbitrationLost = -23, // I2C Bus Error - Bus Arbitration Lost
    ErrorI2CBusError = -24,        // I2C Bus Error - General Bus Error
    ErrorI2CNackReceived = -25,    // I2C Bus Error - NACK Received
    ErrorI2CFail = -26,            // I2C Bus Error - General Failure

    DivZeroError = -80, // Attempted div by zero

    CommPortNotOpen = -101,      // Comm port not open
    CommInvalidPortError = -102, // Comm port no such port error
    CommRangeError = -103,       // Comm port range error
    ErrorCreatingComm = -104,    // Error creating comm
    ErrorStartingComm = -105,    // Error starting comm
    ErrorClosingComm = -106,     // Error closing comm
    CommChecksumError = -107,    // Comm checksum error
    CommNoDev = -108,            // No comm device
    TimoutError = -109,          // Comm timeout error
    CommErrorWaitingComm = -110, // Error writing comm
    CommErrorReadingComm = -111, // Error reading comm
    CommCountError = -112,       // Comm byte count error

    OperationCanceled = -126,  // Camera operation canceled
    UndefinedErrorCode = -127, // Undefined error
}

pub type LeptonCCI_I2C =
    I2CInterface<I2C0, (Pin<Gpio12, Function<I2C>>, Pin<Gpio13, Function<I2C>>)>;

type VsyncPin = Pin<Gpio3, FloatingInput>;
pub struct LeptonCCI {
    i2c: LeptonCCI_I2C,
}

pub struct LeptonSpi {
    spi: Spi<Enabled, SPI1, 16>,
    cs: Pin<Gpio9, Output<PushPull>>,
    sck: Pin<Gpio11, Output<PushPull>>,
    _rx: Pin<Gpio8, FunctionSpi>,
    _tx: Pin<Gpio10, FunctionSpi>,
    pub vsync: VsyncPin,
}

pub struct Lepton {
    pub spi: LeptonSpi,
    cci: LeptonCCI,
    resetting: bool,
    clock_running: bool
}

impl Lepton {
    pub fn new(
        i2c: LeptonCCI_I2C,
        spi: Spi<Enabled, SPI1, 16>,
        cs: Pin<Gpio9, Output<PushPull>>,
        sck: Pin<Gpio11, Output<PushPull>>,
        rx: Pin<Gpio8, FunctionSpi>,
        tx: Pin<Gpio10, FunctionSpi>,
        vsync: VsyncPin,
    ) -> Lepton {
        Lepton {
            spi: LeptonSpi {
                spi,
                cs,
                sck,
                _rx: rx,
                _tx: tx,
                vsync,
            },
            cci: LeptonCCI { i2c },
            resetting: false,
            clock_running: true
        }
    }

    pub fn init(&mut self, delay: &mut Delay) {
        /*
            a. Wait 950 milliseconds minimum after power on, clocks applied, and RESET_L de-asserted
                • If the camera has an attached shutter, the minimum wait period should be
                extended to 5 seconds to allow the camera’s automatic execution of a flat-field
                correction (Auto FFC mode).
            b. Read the STATUS register (register address 0x0002) bit 2
                • If Bit 2 is 0, then the camera has not booted yet, extend the wait period.
                • If Bit 2 is 1, then the camera has booted, I2C interface is available
            c. Read the STATUS register (register address 0x0002) bit 0
                • If Bit 0 is 1, then the interface is busy, poll until Bit 0 becomes 0
                • If Bit 0 is 0, then the interface is ready for receiving commands.
        */

        self.reset(delay);
        self.ping();

        self.enable_vsync();
        self.enable_focus_metric();
        self.enable_telemetry();
        self.setup_spot_meter_roi();
        self.disable_post_processing();
        //self.disable_t_linear();

        self.spi
            .vsync
            .set_interrupt_enabled(Interrupt::EdgeHigh, true);

        info!("=== Init lepton module ===");
        info!("Got ping? {}", self.ping());
        info!("AGC enabled? {}", self.acg_enabled());
        info!("Focus metric enabled? {}", self.focus_metric_enabled());
        info!("Vsync enabled? {}", self.vsync_enabled());
        info!("Telemetry enabled? {}", self.telemetry_enabled());
        info!(
            "Telemetry in footer? {}",
            self.telemetry_location() == TelemetryLocation::Footer
        );
        info!("Output format RAW14? {}", self.output_raw14());
    }

    fn setup_spot_meter_roi(&mut self) {
        // Change spot meter ROI so we can get scene stats per frame
        self.set_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_RAD,
                LEPTON_RAD_SPOT_METER_ROI,
                LeptonCommandType::Set,
                true,
            ),
            //  [startRow, startCol, endRow, endCol]
            //&[0, 0, 119, 159],
            &[79, 59, 80, 60], // If this is set to anything but the default 2x2 pixels, we get visual glitches in the video feed :-/
        );
    }

    fn disable_post_processing(&mut self) {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_BAD_PIXEL_REPLACEMENT,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );

        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_TEMPORAL_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            0,
        );

        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_COLUMN_NOISE_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );

        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_PIXEL_NOISE_FILTER,
                LeptonCommandType::Set,
                true,
            ),
            1,
        );

        // self.set_attribute_i32(
        //     lepton_command(
        //         LEPTON_SUB_SYSTEM_OEM,
        //         LEP_OEM_VIDEO_OUTPUT_SOURCE,
        //         LeptonCommandType::Set,
        //         true,
        //     ),
        //     3, // Output raw, without post-processing
        // );
    }

    pub fn enable_video_output(&mut self, enabled: bool) {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_VIDEO_OUTPUT_ENABLE,
                LeptonCommandType::Set,
                true,
            ),
            if enabled { 1 } else { 0 },
        );
    }

    pub fn power_down(&mut self) {
        self.execute_command(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_POWER_DOWN,
                LeptonCommandType::Run,
                true,
            ),
        );
    }

    pub fn power_on(&mut self) {
        // Set SDA low
        let mut peripherals = unsafe { pac::Peripherals::steal() };
        let sio = Sio::new(peripherals.SIO);
        let pins = Pins::new(
            peripherals.IO_BANK0,
            peripherals.PADS_BANK0,
            sio.gpio_bank0,
            &mut peripherals.RESETS,
        );
        pins.gpio12.into_push_pull_output().set_low().unwrap();
        //  Issue a single clock pulse.
        let mut scl = pins.gpio13.into_push_pull_output();
        scl.toggle().unwrap();
        for _ in 0..332 {
            nop();
        }
        scl.toggle().unwrap();

        let _ = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                0,
                0,
                0,
                0,
            ],
        );
        //self.wait_for_ready(true);
    }

    fn disable_t_linear(&mut self) {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_RAD,
                LEPTON_RAD_TLINEAR_ENABLE_STATE,
                LeptonCommandType::Set,
                true,
            ),
            0,
        );
    }

    pub fn reset(&mut self, delay: &mut Delay) {
        info!("Resetting");
        self.resetting = true;
        self.spi.cs.set_low().unwrap();
        self.spi.sck.set_high().unwrap(); // SCK is high when idle
        delay.delay_ms(300);
        self.spi.cs.set_high().unwrap();
        self.spi.sck.set_low().unwrap();
        self.resetting = false;
        //info!("Finished resetting pins");
    }

    pub fn is_resetting(&self) -> bool {
        self.resetting
    }

    pub fn transfer<'w>(&mut self, words: &'w mut [u16]) -> Result<&'w [u16], Infallible> {
        self.spi.cs.set_low().unwrap();
        let result = self.spi.spi.transfer(words);
        self.spi.cs.set_high().unwrap();
        result
    }

    pub fn transfer_block<'w>(&mut self, words: &'w mut [u16]) -> Result<&'w [u16], Infallible> {
        self.spi.spi.transfer(words)
    }

    fn wait_for_ffc_status_ready(&mut self) -> bool {
        // loop {
        //     let (ffc_state, length) = self.get_attribute(
        //         delay,
        //         lepton_command(
        //             LEPTON_SUB_SYSTEM_SYS,
        //             LEPTON_SYS_FFC_STATUS,
        //             LeptonCommandType::Get,
        //             false,
        //         ),
        //         false,
        //     );
        //     let ffc_state = BigEndian::read_i32(&ffc_state[..((length * 2) as usize)]);
        //     //info!("FFC State {}", ffc_state);
        //     if ffc_state == 0 {
        //         return true;
        //     }
        // }
        true
    }

    pub fn telemetry_location(&mut self) -> TelemetryLocation {
        let (telemetry_location, length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_STATS,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        let telemetry_location_state =
            LittleEndian::read_u32(&telemetry_location[..((length * 2) as usize)]);
        if telemetry_location_state == 0 {
            TelemetryLocation::Header
        } else {
            TelemetryLocation::Footer
        }
    }

    pub fn enable_telemetry(&mut self) {
        // Enable telemetry
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_TELEMETRY_ENABLE_STATE,
                LeptonCommandType::Set,
                false,
            ),
            1,
        );
        // Set location to footer
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_TELEMETRY_LOCATION,
                LeptonCommandType::Set,
                false,
            ),
            1,
        );
    }

    pub fn telemetry_enabled(&mut self) -> bool {
        let (telemetry_enabled, length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_TELEMETRY_LOCATION,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        LittleEndian::read_u32(&telemetry_enabled[..((length * 2) as usize)]) == 1
    }

    pub fn output_raw14(&mut self) -> bool {
        let (output_state, length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_VID,
                LEPTON_VID_OUTPUT_FORMAT,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        let output_state = LittleEndian::read_u32(&output_state[..((length * 2) as usize)]);
        output_state == 7
    }

    pub fn stop_clock(&mut self) {
        if self.clock_running {
            self.clock_running = false;
            self.spi.spi.set_baudrate(0.Hz(), 0.Hz());
        }
    }

    pub fn start_clock(&mut self, peri_freq: HertzU32, baudrate: HertzU32) {
        if !self.clock_running {
            self.clock_running = true;
            self.spi.spi.set_baudrate(peri_freq, baudrate);
        }
    }

    pub fn cs_low(&mut self) {
        self.spi.cs.set_low().unwrap();
    }

    pub fn cs_high(&mut self) {
        self.spi.cs.set_high().unwrap();
    }

    pub fn status(&mut self) -> u32 {
        let (output_state, _length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_STATUS,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        LittleEndian::read_u32(&output_state[..4usize])
    }

    pub fn enable_focus_metric(&mut self) {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_VID,
                LEPTON_VID_FOCUS_ENABLE_STATE,
                LeptonCommandType::Set,
                false,
            ),
            1,
        );
    }

    pub fn focus_metric_enabled(&mut self) -> bool {
        let (enabled_focus_state, l) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_VID,
                LEPTON_VID_FOCUS_ENABLE_STATE,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        LittleEndian::read_u32(&enabled_focus_state[..((l * 2) as usize)]) == 1
    }

    pub fn get_focus_metric(&mut self) -> u32 {
        let (focus_metric, l) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_VID,
                LEPTON_VID_FOCUS_METRIC,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        LittleEndian::read_u32(&focus_metric[..((l * 2) as usize)])
    }

    pub fn acg_enabled(&mut self) -> bool {
        let (agc_state, length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_AGC,
                LEPTON_AGC_ENABLE_STATE,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        let acg_enabled_state = LittleEndian::read_u32(&agc_state[..((length * 2) as usize)]);
        acg_enabled_state == 1
    }

    pub fn ping(&mut self) -> bool {
        self.execute_command(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_PING_CAMERA,
                LeptonCommandType::Run,
                false,
            ),
        )
    }

    pub fn reboot(&mut self, delay: &mut Delay, wait: bool) -> bool {
        warn!("Rebooting lepton module");
        let ok = self.execute_command(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_REBOOT,
                LeptonCommandType::Run,
                true,
            ),
        );
        info!("Waiting 5 seconds");
        delay.delay_ms(5000);
        self.init(delay);
        ok
    }

    pub fn enable_vsync(&mut self) {
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_GPIO_VSYNC_PHASE_DELAY,
                LeptonCommandType::Set,
                true,
            ),
            3,
        );

        // Set the phase to -1 line, and see if we have less reboots
        self.set_attribute_i32(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_GPIO_MODE,
                LeptonCommandType::Set,
                true,
            ),
            5,
        );
    }

    pub fn vsync_enabled(&mut self) -> bool {
        let (vsync_state, length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_OEM,
                LEPTON_OEM_GPIO_MODE,
                LeptonCommandType::Get,
                true,
            ),
            true,
        );
        LittleEndian::read_u32(&vsync_state[..((length * 2) as usize)]) == 5
    }

    pub fn scene_stats(&mut self) -> SceneStats {
        let (scene_stats, _length) = self.get_attribute(
            lepton_command(
                LEPTON_SUB_SYSTEM_SYS,
                LEPTON_SYS_STATS,
                LeptonCommandType::Get,
                false,
            ),
            true,
        );
        SceneStats {
            avg: LittleEndian::read_u16(&scene_stats[0..2]),
            max: LittleEndian::read_u16(&scene_stats[2..4]),
            min: LittleEndian::read_u16(&scene_stats[4..6]),
            num_pixels: LittleEndian::read_u16(&scene_stats[6..8]),
        }
    }

    pub fn wait_for_ready(&mut self, print: bool) -> u16 {
        let mut readbuf: [u8; 2];
        let mut camera_status = 0u16;
        loop {
            readbuf = [0; 2];
            if self
                .cci
                .i2c
                .write_read(LEPTON_ADDRESS, &LEPTON_STATUS_REGISTER, &mut readbuf)
                .map_or(false, |_| {
                    camera_status = BigEndian::read_u16(&readbuf);
                    if print {
                        info!("Camera status {:#018b}", camera_status);
                    }
                    camera_status & (LEPTON_BOOTED | LEPTON_BOOT_MODE | LEPTON_BUSY) != 0
                })
            {
                // Return the status bits of the status
                return camera_status;
            }
        }
    }

    fn execute_command(
        &mut self,
        (command, _length): LeptonSynthesizedCommand,
    ) -> bool {
        self.wait_for_ready(false);
        self.wait_for_ffc_status_ready();
        let result = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                LEPTON_COMMAND_ID_REGISTER[0],
                LEPTON_COMMAND_ID_REGISTER[1],
                command[0],
                command[1],
                // TODO
            ],
        );
        self.wait_for_ready(false);

        // TODO Process errors
        // Now read the status register to see if it worked?
        result.is_ok()
    }


    fn get_attribute(
        &mut self,
        (command, length): LeptonSynthesizedCommand,
        wait_for_ffc: bool,
    ) -> ([u8; 32], u16) {
        self.wait_for_ready(false);
        if wait_for_ffc {
            self.wait_for_ffc_status_ready();
        }
        let len = lepton_register_val(length);
        let _ = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                LEPTON_DATA_LENGTH_REGISTER[0],
                LEPTON_DATA_LENGTH_REGISTER[1],
                len[0],
                len[1],
            ],
        );

        let _ = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                LEPTON_COMMAND_ID_REGISTER[0],
                LEPTON_COMMAND_ID_REGISTER[1],
                command[0],
                command[1],
            ],
        );

        self.wait_for_ready(false);

        // Read command error?

        // Now read out length registers, and assemble them into the return type that we want.
        let mut buf = [0u8; 32];
        self.cci
            .i2c
            .write_read(
                LEPTON_ADDRESS,
                &LEPTON_DATA_0_REGISTER,
                &mut buf[..(length as usize * 2)],
            )
            .unwrap();
        // Each u16 needs to be read in order before we can read out a u32 properly
        for chunk in buf.chunks_exact_mut(2) {
            let tmp = chunk[0];
            chunk[0] = chunk[1];
            chunk[1] = tmp;
        }
        return (buf, length);
    }

    fn set_attribute_i32(
        &mut self,
        command: LeptonSynthesizedCommand,
        val: i32,
    ) -> bool {
        self.set_attribute(
            command,
            &[(val & 0xffff) as u16, (val >> 16 & 0xffff) as u16],
        )
    }

    fn set_attribute(
        &mut self,
        (command, length): LeptonSynthesizedCommand,
        words: &[u16],
    ) -> bool {
        self.wait_for_ready(false);
        self.wait_for_ffc_status_ready();
        let len = lepton_register_val(length);

        let _ = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                LEPTON_DATA_LENGTH_REGISTER[0],
                LEPTON_DATA_LENGTH_REGISTER[1],
                len[0],
                len[1],
            ],
        );

        for (index, word) in words.iter().enumerate() {
            // Data reg 0
            let register_address = (0x0008u16 + (index * 2) as u16).to_be_bytes();
            let word = word.to_be_bytes();
            let _ = self.cci.i2c.write(
                LEPTON_ADDRESS,
                &[register_address[0], register_address[1], word[0], word[1]],
            );
        }

        let _ = self.cci.i2c.write(
            LEPTON_ADDRESS,
            &[
                LEPTON_COMMAND_ID_REGISTER[0],
                LEPTON_COMMAND_ID_REGISTER[1],
                command[0],
                command[1],
                // TODO
            ],
        );

        let success = self.wait_for_ready(false);
        if (success >> 8) as i8 != 0 {
            warn!("Set attribute error {}", (success >> 8) as i8);
        }
        // Now read out length registers, and assemble them into the return type that we want.

        // Now read the status register to see if it worked?
        return success == 0;
    }

    pub fn get_frame_sync(
        &mut self,
        vo_spi_state: &mut VoSpiState,
        delay: &mut Delay,
        timer: &Timer,
        start: MicroSecondsInstant,
        completed_frame: &mut bool,
    ) {
        let mut discards_before_first_good = 0;
        let mut print_first_packet = true;
        //let mut vo_spi_state = VoSpiState::new();
        loop {
            // This is one scanline
            let mut buffer = [0u16; 82];
            let packet = self.transfer(&mut buffer).unwrap();

            let packet_header = packet[0];
            let is_discard_packet = packet_header & 0x0f00 == 0x0f00;
            if is_discard_packet {
                discards_before_first_good += 1;
                continue;
            }
            let packet_id = (packet_header & 0x0fff) as isize;
            let segment_num = packet_header >> 12;
            // if segment_num == 0 && packet_id > 20 {
            //     continue;
            // }
            // if print_first_packet {
            //     info!("First packet #{} in segment {}", packet_id, segment_num);
            //     print_first_packet = false;
            // }

            if packet_id == 0 {
                vo_spi_state.prev_packet_id = -1;
                vo_spi_state.started_segment = true;
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
                    self.wait_for_ready(false);
                    self.reset(delay);

                    let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                    let elapsed = end.checked_duration_since(start).unwrap().ticks();
                }
                vo_spi_state.current_segment_num = segment_num;
                if vo_spi_state.current_segment_num == 0 {
                    vo_spi_state.started_segment = false;
                }
            }

            // We only mark a segment as started if the packet num was 0 or 20.
            if vo_spi_state.started_segment {
                if packet_id != vo_spi_state.prev_packet_id + 1 {
                    // Packet order mismatch
                    warn!(
                        "Packet order mismatch current: {}, prev: {}, seg {}",
                        packet_id, vo_spi_state.prev_packet_id, vo_spi_state.current_segment_num
                    );
                    vo_spi_state.reset();

                    if vo_spi_state.packet_id_mismatches > 3 {
                        vo_spi_state.packet_id_mismatches = 0;
                        let now = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                        let last = MicroSecondsInstant::from_ticks(vo_spi_state.last_reboot_time);
                        let elapsed = now.checked_duration_since(last).unwrap().ticks();
                        warn!(
                            "{} prior reboots, last was {} mins ago",
                            vo_spi_state.reboots,
                            elapsed / 1000 / 1000 / 60,
                        );
                        vo_spi_state.reboots += 1;
                        vo_spi_state.last_reboot_time = timer.get_counter_low();
                        self.reboot(delay, false);
                    }
                    // Reset and resync - do we need to disable vsync in here so we don't keep entering this function?
                    self.reset(delay);
                    vo_spi_state.packet_id_mismatches += 1;

                    let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                    let elapsed = end.checked_duration_since(start).unwrap().ticks();
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
                }

                let segment_ended = packet_id == 60;
                if segment_ended {}

                if packet_id == 60 && !is_last_segment {
                    // Increment in good faith if we're on the last packet of a valid segment
                    vo_spi_state.current_segment_num += 1;

                    let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                    let elapsed = end.checked_duration_since(start).unwrap().ticks();
                    //break;
                }
                if packet_id == 60 && segment_num == 0 {
                    // This is an invalid frame segment, so we should return until the next vsync
                    break;
                }
                if packet_id == 57 && is_last_segment {
                    // frame counter
                    let frame_counter =
                        LittleEndian::read_u32(&packet[2..][20..=21].as_byte_slice());

                    // Grab two more telemetry rows, and discard
                    self.transfer(&mut buffer).unwrap(); // packet 58
                    self.transfer(&mut buffer).unwrap(); // packet 59
                    let end = MicroSecondsInstant::from_ticks(timer.get_counter_low());
                    let elapsed = end.checked_duration_since(start).unwrap().ticks();
                    // Pass control of frame to other core for further processing.
                    info!("Read out full frame {}", frame_counter);
                    vo_spi_state.current_frame_num = frame_counter;
                    *completed_frame = true;
                    vo_spi_state.reset();
                    break;
                }
            }
            vo_spi_state.prev_packet_id = packet_id;
        }
    }
}
// TODO - CRC?

const fn lepton_register_val(cmd: u16) -> LeptonRegister {
    let mut buf = [0; 2];
    buf[0] = (cmd >> 8) as u8;
    buf[1] = cmd as u8;
    buf
}

pub const fn lepton_command(
    sub_system: u16,
    command: LeptonCommand,
    command_type: LeptonCommandType,
    is_oem: bool,
) -> LeptonSynthesizedCommand {
    let mut synthesized_command = sub_system | command.0 | command_type as u16;
    if is_oem {
        synthesized_command |= synthesized_command | LEPTON_COMMAND_OEM_BIT;
    }
    (lepton_register_val(synthesized_command), command.1)
}

pub struct SceneStats {
    pub min: u16,
    pub max: u16,
    pub avg: u16,
    pub num_pixels: u16,
}

#[derive(PartialEq)]
pub enum TelemetryLocation {
    Header,
    Footer,
}
