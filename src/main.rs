#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::i2c::{self, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{I2C1, USB};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use embedded_hal_async::i2c::I2c as I2cTrait;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

const MPU_ADDR: u8         = 0x68;
const REG_PWR_MGMT_1: u8   = 0x6B;
const REG_ACCEL_XOUT_H: u8 = 0x3B;
const REG_GYRO_XOUT_H: u8  = 0x43;
const REG_WHO_AM_I: u8     = 0x75;
const ACCEL_SCALE: f32     = 16384.0;
const GYRO_SCALE: f32      = 131.0;

const CALIB_SAMPLES: usize = 200;

static USB_CHANNEL: Channel<ThreadModeRawMutex, [u8; 128], 4> = Channel::new();

bind_interrupts!(struct Irqs {
    I2C1_IRQ    => I2cInterruptHandler<I2C1>;
    USBCTRL_IRQ => UsbInterruptHandler<USB>;
});

#[embassy_executor::task]
async fn usb_task(driver: Driver<'static, USB>) -> ! {
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC:    StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC:   StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL:     StaticCell<[u8; 64]>  = StaticCell::new();
    static STATE:       StaticCell<State>      = StaticCell::new();

    let mut config      = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("SkyRust");
    config.product      = Some("MPU-6050 Logger");

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        MSOS_DESC.init([0; 256]),
        CONTROL.init([0; 64]),
    );

    let state     = STATE.init(State::new());
    let mut class = CdcAcmClass::new(&mut builder, state, 64);
    let mut usb   = builder.build();

    loop {
        let usb_fut = usb.run();
        let io_fut  = async {
            class.wait_connection().await;
            loop {
                let msg = USB_CHANNEL.receive().await;
                let len = msg.iter().position(|&b| b == 0).unwrap_or(128);
                let _ = class.write_packet(&msg[..len]).await;
                let _ = class.write_packet(b"\r\n").await;
            }
        };
        embassy_futures::select::select(usb_fut, io_fut).await;
    }
}

async fn mpu_write(i2c: &mut i2c::I2c<'_, I2C1, i2c::Async>, reg: u8, val: u8) {
    I2cTrait::write(i2c, MPU_ADDR, &[reg, val]).await.unwrap();
}

async fn mpu_read(i2c: &mut i2c::I2c<'_, I2C1, i2c::Async>, reg: u8, buf: &mut [u8]) {
    I2cTrait::write_read(i2c, MPU_ADDR, &[reg], buf).await.unwrap();
}

fn i16_be(h: u8, l: u8) -> i16 {
    ((h as i16) << 8) | l as i16
}

async fn calibrate(i2c: &mut i2c::I2c<'_, I2C1, i2c::Async>) -> (f32, f32, f32, f32, f32, f32) {
    let mut sum_ax = 0f32;
    let mut sum_ay = 0f32;
    let mut sum_az = 0f32;
    let mut sum_gx = 0f32;
    let mut sum_gy = 0f32;
    let mut sum_gz = 0f32;

    let mut raw = [0u8; 6];

    for _ in 0..CALIB_SAMPLES {
        mpu_read(i2c, REG_ACCEL_XOUT_H, &mut raw).await;
        sum_ax += i16_be(raw[0], raw[1]) as f32 / ACCEL_SCALE;
        sum_ay += i16_be(raw[2], raw[3]) as f32 / ACCEL_SCALE;
        sum_az += i16_be(raw[4], raw[5]) as f32 / ACCEL_SCALE;

        mpu_read(i2c, REG_GYRO_XOUT_H, &mut raw).await;
        sum_gx += i16_be(raw[0], raw[1]) as f32 / GYRO_SCALE;
        sum_gy += i16_be(raw[2], raw[3]) as f32 / GYRO_SCALE;
        sum_gz += i16_be(raw[4], raw[5]) as f32 / GYRO_SCALE;

        Timer::after(Duration::from_millis(10)).await;
    }

    let n = CALIB_SAMPLES as f32;
    let ax_bias = sum_ax / n;
    let ay_bias = sum_ay / n;
    let az_bias = sum_az / n - 1.0; // restar 1g de gravedad
    let gx_bias = sum_gx / n;
    let gy_bias = sum_gy / n;
    let gz_bias = sum_gz / n;

    (ax_bias, ay_bias, az_bias, gx_bias, gy_bias, gz_bias)
}

fn fmt_f32(val: f32, buf: &mut [u8]) -> usize {
    let neg  = val < 0.0;
    let abs  = if neg { -val } else { val };
    let int  = abs as u32;
    let frac = ((abs - int as f32) * 1000.0) as u32;

    let mut tmp = [0u8; 16];
    let mut pos = 0usize;

    if neg { tmp[pos] = b'-'; pos += 1; }

    if int == 0 {
        tmp[pos] = b'0'; pos += 1;
    } else {
        let mut n = int;
        let start = pos;
        while n > 0 { tmp[pos] = b'0' + (n % 10) as u8; pos += 1; n /= 10; }
        tmp[start..pos].reverse();
    }

    tmp[pos] = b'.';                             pos += 1;
    tmp[pos] = b'0' + (frac / 100) as u8;       pos += 1;
    tmp[pos] = b'0' + ((frac / 10) % 10) as u8; pos += 1;
    tmp[pos] = b'0' + (frac % 10) as u8;        pos += 1;

    let n = pos.min(buf.len());
    buf[..n].copy_from_slice(&tmp[..n]);
    n
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    spawner.spawn(usb_task(Driver::new(p.USB, Irqs))).unwrap();

    let mut i2c = i2c::I2c::new_async(
        p.I2C1,
        p.PIN_7,  // SCL
        p.PIN_6,  // SDA
        Irqs,
        I2cConfig::default(),
    );

    Timer::after(Duration::from_millis(2000)).await;

    let mut who = [0u8; 1];
    mpu_read(&mut i2c, REG_WHO_AM_I, &mut who).await;
    if who[0] != 0x68 {
        error!("MPU-6050 not found (WHO_AM_I=0x{:02X})", who[0]);
        loop { Timer::after(Duration::from_secs(1)).await; }
    }
    info!("MPU-6050 OK");

    // Despertar
    mpu_write(&mut i2c, REG_PWR_MGMT_1, 0x00).await;
    Timer::after(Duration::from_millis(100)).await;

    {
        let mut msg = [0u8; 128];
        let s = b"=== CALIBRACION: manten la Pico inmovil 2 segundos ===";
        msg[..s.len()].copy_from_slice(s);
        USB_CHANNEL.send(msg).await;
    }
    info!("Calibrating — dont move the dron...");

    let (ax_b, ay_b, az_b, gx_b, gy_b, gz_b) = calibrate(&mut i2c).await;

    info!("Bias accel: X={} Y={} Z={}", ax_b, ay_b, az_b);
    info!("Bias gyro:  X={} Y={} Z={}", gx_b, gy_b, gz_b);

    {
        let mut msg = [0u8; 128];
        let s = b"=== Calbration complete. Reading... ===";
        msg[..s.len()].copy_from_slice(s);
        USB_CHANNEL.send(msg).await;
    }

    let mut raw = [0u8; 6];

    loop {
        mpu_read(&mut i2c, REG_ACCEL_XOUT_H, &mut raw).await;
        let ax = i16_be(raw[0], raw[1]) as f32 / ACCEL_SCALE - ax_b;
        let ay = i16_be(raw[2], raw[3]) as f32 / ACCEL_SCALE - ay_b;
        let az = i16_be(raw[4], raw[5]) as f32 / ACCEL_SCALE - az_b;

        mpu_read(&mut i2c, REG_GYRO_XOUT_H, &mut raw).await;
        let gx = i16_be(raw[0], raw[1]) as f32 / GYRO_SCALE - gx_b;
        let gy = i16_be(raw[2], raw[3]) as f32 / GYRO_SCALE - gy_b;
        let gz = i16_be(raw[4], raw[5]) as f32 / GYRO_SCALE - gz_b;

        // Construir línea USB
        let mut line = [0u8; 128];
        let mut pos  = 0usize;

        macro_rules! push {
            ($s:expr) => { for &b in $s { if pos < 127 { line[pos] = b; pos += 1; } } };
        }
        macro_rules! push_f {
            ($v:expr) => {{
                let mut tmp = [0u8; 12];
                let n = fmt_f32($v, &mut tmp);
                push!(&tmp[..n]);
            }};
        }

        push!(b"A[g] X="); push_f!(ax);
        push!(b" Y=");     push_f!(ay);
        push!(b" Z=");     push_f!(az);
        push!(b" | G[d/s] X="); push_f!(gx);
        push!(b" Y=");     push_f!(gy);
        push!(b" Z=");     push_f!(gz);

        USB_CHANNEL.send(line).await;
        info!("A X={} Y={} Z={} | G X={} Y={} Z={}", ax, ay, az, gx, gy, gz);

        Timer::after(Duration::from_millis(100)).await;
    }
}