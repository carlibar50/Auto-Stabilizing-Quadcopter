#![no_std]
#![no_main]

use cortex_m::asm::delay;
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

const T1H_NS: u32 = 2500;
const T1L_NS: u32 = 833;
const T0H_NS: u32 = 1250;
const T0L_NS: u32 = 2083;
const DSHOT_ARM: u16 = 0;
const DSHOT_MIN_THROTTLE: u16 = 100;

static USB_CHANNEL: Channel<ThreadModeRawMutex, [u8; 64], 4> = Channel::new();

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn usb_task(driver: Driver<'static, USB>) -> ! {
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL: StaticCell<[u8; 64]> = StaticCell::new();
    static STATE: StaticCell<State> = StaticCell::new();

    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("SkyRust");
    config.product = Some("DShot Controller");

    let mut builder = Builder::new(
        driver, config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL.init([0; 64]),
    );
    let state = STATE.init(State::new());
    let mut class = CdcAcmClass::new(&mut builder, state, 64);
    let mut usb = builder.build();

    loop {
        let usb_fut = usb.run();
        let io_fut = async {
            class.wait_connection().await;
            loop {
                let msg = USB_CHANNEL.receive().await;
                let len = msg.iter().position(|&b| b == 0).unwrap_or(64);
                let _ = class.write_packet(&msg[..len]).await;
                let _ = class.write_packet(b"\r\n").await;
            }
        };
        embassy_futures::select::select(usb_fut, io_fut).await;
    }
}

async fn usb_log(s: &[u8]) {
    let mut msg = [0u8; 64];
    let n = s.len().min(63);
    msg[..n].copy_from_slice(&s[..n]);
    USB_CHANNEL.send(msg).await;
}

fn dshot_frame(throttle: u16, telemetry: bool) -> u16 {
    let t = if telemetry { 1u16 } else { 0u16 };
    let packet = (throttle << 1) | t;
    let crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
    (packet << 4) | crc
}

fn delay_ns(ns: u32) {
    let freq = embassy_rp::clocks::clk_sys_freq();
    let cycles = (freq as u64 * ns as u64) / 1_000_000_000;
    delay(cycles as u32);
}

fn dshot_send_sync(m1: &mut Output<'_>, m2: &mut Output<'_>, m3: &mut Output<'_>, m4: &mut Output<'_>, frame: u16) {
    for i in (0..16).rev() {
        let bit = (frame >> i) & 1;
        
        m1.set_high();
        m2.set_high();
        m3.set_high();
        m4.set_high();
        
        if bit == 1 {
            delay_ns(T1H_NS);
            m1.set_low();
            m2.set_low();
            m3.set_low();
            m4.set_low();
            delay_ns(T1L_NS);
        } else {
            delay_ns(T0H_NS);
            m1.set_low();
            m2.set_low();
            m3.set_low();
            m4.set_low();
            delay_ns(T0L_NS);
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    spawner.spawn(usb_task(Driver::new(p.USB, Irqs))).unwrap();
    Timer::after(Duration::from_millis(2000)).await;

    usb_log(b"SkyRust - 4 Motors Sync").await;
    usb_log(b"REMOVE PROPELLERS!").await;
    info!("Test 4 motors initialized");

    let mut m1 = Output::new(p.PIN_0, Level::Low);
    let mut m2 = Output::new(p.PIN_1, Level::Low);
    let mut m3 = Output::new(p.PIN_2, Level::Low);
    let mut m4 = Output::new(p.PIN_3, Level::Low);

    usb_log(b"Arming ESCs...").await;
    info!("Arming sequence...");
    let t_end = embassy_time::Instant::now() + Duration::from_millis(4000);
    while embassy_time::Instant::now() < t_end {
        let frame = dshot_frame(DSHOT_ARM, false);
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, frame);
        Timer::after(Duration::from_micros(50)).await;
    }

    usb_log(b"Throttle ramp...").await;
    info!("Ramping throttle...");
    let mut throttle: u16 = 48;
    let t_end = embassy_time::Instant::now() + Duration::from_millis(3000);
    while embassy_time::Instant::now() < t_end {
        let frame = dshot_frame(throttle, false);
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, frame);
        Timer::after(Duration::from_micros(50)).await;
        if throttle < DSHOT_MIN_THROTTLE {
            throttle += 1;
        }
    }

    usb_log(b"Throttle hold...").await;
    info!("Holding throttle...");
    let t_end = embassy_time::Instant::now() + Duration::from_millis(3000);
    while embassy_time::Instant::now() < t_end {
        let frame = dshot_frame(DSHOT_MIN_THROTTLE, false);
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, frame);
        Timer::after(Duration::from_micros(50)).await;
    }

    usb_log(b"Stopping motors").await;
    info!("Stopping...");
    let t_end = embassy_time::Instant::now() + Duration::from_millis(500);
    while embassy_time::Instant::now() < t_end {
        let frame = dshot_frame(DSHOT_ARM, false);
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, frame);
        Timer::after(Duration::from_micros(50)).await;
    }

    usb_log(b"Test completed.").await;
    info!("Test completed OK");

    loop { Timer::after(Duration::from_secs(60)).await; }
}