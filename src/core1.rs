use bsp::pac::SPI0;
use cortex_m::prelude::_embedded_hal_blocking_spi_Write;
use defmt::info;
use embedded_hal::digital::v2::OutputPin;
use fugit::HertzU32;
use fugit::RateExtU32;
use rp2040_hal::dma::single_buffer;
use rp2040_hal::dma::DMAExt;
use rp2040_hal::gpio::FunctionSpi;
use rp2040_hal::pio::PIOBuilder;
use rp2040_hal::prelude::_rphal_pio_PIOExt;
use rp2040_hal::Sio;
use rp2040_hal::Spi;
use rp_pico as bsp;
use rp_pico::pac::Peripherals;

use pio_proc;

use crate::FRAME_BUFFER;
pub unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
    core::slice::from_raw_parts((p as *const T) as *const u8, core::mem::size_of::<T>())
}

pub const CORE1_TASK_COMPLETE: u32 = 0xEE;

const SEGMENT_LENGTH: usize = 9764;
const FULL_CHUNK_LENGTH: usize = SEGMENT_LENGTH / 4;
const CHUNK_LENGTH: usize = (SEGMENT_LENGTH - 4) / 4;
static mut b: [u8; SEGMENT_LENGTH] = [0u8; SEGMENT_LENGTH];
pub fn core1_task() -> ! {
    let mut pac = unsafe { Peripherals::steal() };
    //let core = unsafe { bsp::hal::pac::CorePeripherals::steal() };

    let mut sio = Sio::new(pac.SIO);
    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let dma = pac.DMA.split(&mut pac.RESETS);
    // TODO: Setup an SPI interface to send bytes to the raspberry pi
    let mut signal = pins.gpio28.into_push_pull_output();

    let _spi_mosi = pins.gpio16.into_mode::<FunctionSpi>();
    let _spi_miso = pins.gpio19.into_mode::<FunctionSpi>();
    let _spi_sck = pins.gpio18.into_mode::<FunctionSpi>();
    let _spi_cs = pins.gpio17.into_mode::<FunctionSpi>();

    let spi: Spi<_, SPI0, 8> = Spi::new(pac.SPI0);
    let mut spi = spi.init(
        &mut pac.RESETS,
        133_000_000.Hz(),
        30_000_000.Hz(),
        &embedded_hal::spi::MODE_3,
        true,
    );

    info!("Started core 1 task");
    // TODO: Can we investigate async spi calls, rather than blocking?
    // In theory we're handing off to dedicated SPI hardware, and the CPU shouldn't need
    // to wait around for the transfer to finish before going on to do other things.
    // Possibly a triple buffer if we're recording to SDCARD, and then we can be doing some
    // tracking at the same time?  I think DMA is the answer here

    // TODO: Okay, looks like SPI in slave mode is limited to 1/12 of the core clock, which is a lot slower than we thought.
    // Maybe use PIO based SPI to increase the speed to something decent, and bring up our idle time?

    // Also, if we're not running SPI at full clock speed, can we interleave transfers with other useful operations?
    // (This goes for sdcard writes)

    //let mut i = 0;

    // let mut test_b = [0u8; 256];
    // for i in 0..test_b.len() {
    //     test_b[i] = i as u8;
    // }
    // let mut test_slice = [0u8; 488];
    // test_slice[0..256].copy_from_slice(&test_b);
    // test_slice[256..].copy_from_slice(&test_b[0..232]);
    let mut dma_ch0 = dma.ch0;
    info!("Start Core1 loop");

    // Make sure we start low here.
    signal.set_low().unwrap();
    loop {
        let input = sio.fifo.read_blocking();
        //info!("got fifo read on core1 {}", input);
        //debug_core_1_pin.set_high().unwrap();
        let mut seg = 0;
        critical_section::with(|cs| {
            let frame_seg = FRAME_BUFFER.back.borrow_ref(cs);
            seg = (frame_seg.0).0;
            //info!("Seg {}", seg);
            let slice = unsafe { any_as_u8_slice(&(frame_seg.0).1) };
            for (index, chunk) in slice.chunks_exact(CHUNK_LENGTH).enumerate() {
                let start = index * (CHUNK_LENGTH + 1);
                unsafe {
                    b[start] = seg;
                    b[start + 1..(start + 1) + CHUNK_LENGTH].copy_from_slice(chunk);
                }
            }
        });

        // If we're slave, maybe we need to signal on a gpio pin that we want the master to start reading?
        signal.set_high().unwrap();
        unsafe {
            for i in 0..4 {
                let tx_transfer = single_buffer::Config::new(
                    dma_ch0,
                    &b[i * FULL_CHUNK_LENGTH..i * FULL_CHUNK_LENGTH + FULL_CHUNK_LENGTH],
                    spi,
                )
                .start();
                let (ch0, _, spi_ret) = tx_transfer.wait();
                spi = spi_ret;
                dma_ch0 = ch0;
            }
        }
        signal.set_low().unwrap();

        //info!("End write");
        //debug_core_1_pin.set_low().unwrap();
        sio.fifo.write_blocking(CORE1_TASK_COMPLETE);
        //});
        // Instead of calling this
        //};
    }
}
