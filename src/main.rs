#![no_std]
#![no_main]

use core::fmt::Write;
use cortex_m::asm::delay as asm_delay;
use embassy_executor::Spawner;
use embassy_net::{Config, StackResources, udp::{UdpSocket, PacketMetadata}};
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::i2c::{self, Config as I2cConfig, InterruptHandler as I2cInterruptHandler};
use embassy_rp::peripherals::{DMA_CH0, I2C1, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, InterruptHandler as UsbInterruptHandler};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer, Ticker};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::Builder;
use embedded_hal_async::i2c::I2c as I2cTrait;
use rand_core::RngCore;
use static_cell::StaticCell;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use heapless::String;
use {defmt_rtt as _, panic_probe as _};

#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

const WIFI_SSID: &str = "iPhone de carlos";
const WIFI_PASS: &str = "1423_123";
const UDP_PORT:  u16  = 4210;

const T1H_NS: u32 = 2500;
const T1L_NS: u32 = 833;
const T0H_NS: u32 = 1250;

const MPU_ADDR: u8          = 0x68;
const REG_PWR_MGMT_1: u8    = 0x6B;
const REG_ACCEL_XOUT_H: u8  = 0x3B;
const REG_GYRO_XOUT_H: u8   = 0x43;
const REG_WHO_AM_I: u8      = 0x75;
const ACCEL_SCALE: f32      = 16384.0;
const GYRO_SCALE: f32       = 131.0;
const CALIB_SAMPLES: usize  = 200;

const I_MAX: f32 = 50.0;
const DSHOT_MAX: u16 = 2000;
const FAILSAFE_MS: u64 = 1000;

#[derive(Clone, Copy, Default)]
struct Setpoint {
    throttle: u16,
    pitch:    f32,
    roll:     f32,
    yaw_rate: f32,
}

#[derive(Clone, Copy, Default)]
struct Orientation {
    pitch: f32,
    roll:  f32,
    gyro_z: f32,
}

#[derive(Clone, Copy, Default)]
struct MotorValues {
    m1: u16,
    m2: u16,
    m3: u16,
    m4: u16,
}

#[derive(Clone, Copy)]
struct PidParams {
    kp: f32,
    ki: f32,
    kd: f32,
}

static USB_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 64], 8> = Channel::new();
static SETPOINT:    Mutex<CriticalSectionRawMutex, Setpoint>    = Mutex::new(Setpoint    { throttle: 0, pitch: 0.0, roll: 0.0, yaw_rate: 0.0 });
static ORIENTATION: Mutex<CriticalSectionRawMutex, Orientation> = Mutex::new(Orientation { pitch: 0.0, roll: 0.0, gyro_z: 0.0 });
static MOTORS:      Mutex<CriticalSectionRawMutex, MotorValues> = Mutex::new(MotorValues { m1: 0, m2: 0, m3: 0, m4: 0 });
static PID_PARAMS:  Mutex<CriticalSectionRawMutex, PidParams>   = Mutex::new(PidParams   { kp: 1.2, ki: 0.02, kd: 0.05 });
static LAST_PKT:    Mutex<CriticalSectionRawMutex, u64>         = Mutex::new(0u64);

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ  => UsbInterruptHandler<USB>;
    PIO0_IRQ_0   => PioInterruptHandler<PIO0>;
    I2C1_IRQ     => I2cInterruptHandler<I2C1>;
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
    config.product = Some("Drone Flight Controller");

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
            }
        };
        embassy_futures::select::select(usb_fut, io_fut).await;
    }
}

fn usb_log(s: &[u8]) {
    let mut msg = [0u8; 64];
    let n = s.len().min(63);
    msg[..n].copy_from_slice(&s[..n]);
    let _ = USB_CHANNEL.try_send(msg);
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn udp_task(stack: embassy_net::Stack<'static>) -> ! {
    static RX_META: StaticCell<[PacketMetadata; 4]> = StaticCell::new();
    static TX_META: StaticCell<[PacketMetadata; 4]> = StaticCell::new();
    static RX_BUF:  StaticCell<[u8; 256]>           = StaticCell::new();
    static TX_BUF:  StaticCell<[u8; 256]>           = StaticCell::new();

    stack.wait_config_up().await;

    let mut socket = UdpSocket::new(
        stack,
        RX_META.init([PacketMetadata::EMPTY; 4]),
        RX_BUF.init([0; 256]),
        TX_META.init([PacketMetadata::EMPTY; 4]),
        TX_BUF.init([0; 256]),
    );
    socket.bind(UDP_PORT).unwrap();

    let mut buf = [0u8; 64];
    let mut rx_count = 0;

    loop {
        match socket.recv_from(&mut buf).await {
            Ok((20, _remote)) => {
                let throttle = u16::from_le_bytes([buf[0], buf[1]]).min(DSHOT_MAX);
                let pitch    = i16::from_le_bytes([buf[2], buf[3]]) as f32 / 100.0;
                let roll     = i16::from_le_bytes([buf[4], buf[5]]) as f32 / 100.0;
                let yaw_rate = i16::from_le_bytes([buf[6], buf[7]]) as f32 / 100.0;
                
                let kp = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
                let ki = f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
                let kd = f32::from_le_bytes([buf[16], buf[17], buf[18], buf[19]]);

                *SETPOINT.lock().await = Setpoint { throttle, pitch, roll, yaw_rate };
                *PID_PARAMS.lock().await = PidParams { kp, ki, kd };
                *LAST_PKT.lock().await = Instant::now().as_millis();

                rx_count += 1;
                if rx_count % 25 == 0 {
                    let mut msg = String::<64>::new();
                    let _ = core::write!(&mut msg, "UDP OK: THR={} KP={:.2}\r\n", throttle, kp);
                    usb_log(msg.as_bytes());
                }
            }
            Ok((len, _)) => {
                let mut msg = String::<64>::new();
                let _ = core::write!(&mut msg, "UDP ERR: SIZE={}\r\n", len);
                usb_log(msg.as_bytes());
            }
            Err(_) => {}
        }
    }
}

#[embassy_executor::task]
async fn imu_task(mut i2c: i2c::I2c<'static, I2C1, i2c::Async>) -> ! {
    let mut who = [0u8; 1];
    I2cTrait::write_read(&mut i2c, MPU_ADDR, &[REG_WHO_AM_I], &mut who).await.unwrap();
    if who[0] != 0x68 {
        usb_log(b"MPU-6050 NOT FOUND\r\n");
        loop { Timer::after(Duration::from_secs(1)).await; }
    }
    
    I2cTrait::write(&mut i2c, MPU_ADDR, &[REG_PWR_MGMT_1, 0x00]).await.unwrap();
    Timer::after(Duration::from_millis(100)).await;

    usb_log(b"CALIBRATING IMU. DO NOT MOVE.\r\n");
    let mut sa = [0f32; 6];
    let mut raw = [0u8; 6];
    for _ in 0..CALIB_SAMPLES {
        I2cTrait::write_read(&mut i2c, MPU_ADDR, &[REG_ACCEL_XOUT_H], &mut raw).await.unwrap();
        sa[0] += i16_be(raw[0], raw[1]) as f32 / ACCEL_SCALE;
        sa[1] += i16_be(raw[2], raw[3]) as f32 / ACCEL_SCALE;
        sa[2] += i16_be(raw[4], raw[5]) as f32 / ACCEL_SCALE;
        I2cTrait::write_read(&mut i2c, MPU_ADDR, &[REG_GYRO_XOUT_H], &mut raw).await.unwrap();
        sa[3] += i16_be(raw[0], raw[1]) as f32 / GYRO_SCALE;
        sa[4] += i16_be(raw[2], raw[3]) as f32 / GYRO_SCALE;
        sa[5] += i16_be(raw[4], raw[5]) as f32 / GYRO_SCALE;
        Timer::after(Duration::from_millis(10)).await;
    }
    let n = CALIB_SAMPLES as f32;
    let (ax_b, ay_b, az_b) = (sa[0]/n, sa[1]/n, sa[2]/n - 1.0);
    let (gx_b, gy_b, gz_b) = (sa[3]/n, sa[4]/n, sa[5]/n);
    usb_log(b"IMU CALIBRATION COMPLETE\r\n");

    let mut pitch = 0f32;
    let mut roll  = 0f32;
    let dt = 0.01f32;
    let alpha = 0.96f32;

    let mut ticker = Ticker::every(Duration::from_millis(10));

    loop {
        ticker.next().await;

        I2cTrait::write_read(&mut i2c, MPU_ADDR, &[REG_ACCEL_XOUT_H], &mut raw).await.unwrap();
        let ax = i16_be(raw[0], raw[1]) as f32 / ACCEL_SCALE - ax_b;
        let ay = i16_be(raw[2], raw[3]) as f32 / ACCEL_SCALE - ay_b;
        let az = i16_be(raw[4], raw[5]) as f32 / ACCEL_SCALE - az_b;

        I2cTrait::write_read(&mut i2c, MPU_ADDR, &[REG_GYRO_XOUT_H], &mut raw).await.unwrap();
        let gx = i16_be(raw[0], raw[1]) as f32 / GYRO_SCALE - gx_b;
        let gy = i16_be(raw[2], raw[3]) as f32 / GYRO_SCALE - gy_b;
        let gz = i16_be(raw[4], raw[5]) as f32 / GYRO_SCALE - gz_b;

        let az_sq = az * az;
        let pitch_acc = atan2_approx(ay, libm_sqrt(ax * ax + az_sq));
        let roll_acc  = atan2_approx(-ax, libm_sqrt(ay * ay + az_sq));

        pitch = alpha * (pitch + gx * dt) + (1.0 - alpha) * pitch_acc;
        roll  = alpha * (roll  + gy * dt) + (1.0 - alpha) * roll_acc;

        *ORIENTATION.lock().await = Orientation {
            pitch: pitch * 57.2958,
            roll:  roll  * 57.2958,
            gyro_z: gz,
        };
    }
}

#[embassy_executor::task]
async fn pid_task() -> ! {
    let mut err_pitch_i = 0f32;
    let mut err_roll_i  = 0f32;
    let mut err_yaw_i   = 0f32;
    let mut prev_pitch  = 0f32;
    let mut prev_roll   = 0f32;
    let mut prev_yaw    = 0f32;
    let dt = 0.01f32;

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut last_log = 0u64;

    loop {
        ticker.next().await;

        let last = *LAST_PKT.lock().await;
        let now  = Instant::now().as_millis();
        let sp   = *SETPOINT.lock().await;

        let throttle = if now.saturating_sub(last) > FAILSAFE_MS {
            if now.saturating_sub(last_log) > 1000 {
                usb_log(b"FAILSAFE: NO UDP\r\n");
                last_log = now;
            }
            0u16
        } else if sp.throttle < 48 {
            0u16
        } else {
            sp.throttle
        };

        if throttle == 0 {
            *MOTORS.lock().await = MotorValues { m1: 0, m2: 0, m3: 0, m4: 0 };
            err_pitch_i = 0.0;
            err_roll_i  = 0.0;
            err_yaw_i   = 0.0;
            continue;
        }

        let ori = *ORIENTATION.lock().await;
        let cfg = *PID_PARAMS.lock().await;

        let ep = sp.pitch - ori.pitch;
        err_pitch_i = (err_pitch_i + ep * dt).clamp(-I_MAX, I_MAX);
        let dp = (ep - prev_pitch) / dt;
        let pid_pitch = cfg.kp * ep + cfg.ki * err_pitch_i + cfg.kd * dp;
        prev_pitch = ep;

        let er = sp.roll - ori.roll;
        err_roll_i = (err_roll_i + er * dt).clamp(-I_MAX, I_MAX);
        let dr = (er - prev_roll) / dt;
        let pid_roll = cfg.kp * er + cfg.ki * err_roll_i + cfg.kd * dr;
        prev_roll = er;

        let ey = sp.yaw_rate - ori.gyro_z;
        err_yaw_i = (err_yaw_i + ey * dt).clamp(-I_MAX, I_MAX);
        let dy = (ey - prev_yaw) / dt;
        let pid_yaw = cfg.kp * ey + cfg.ki * err_yaw_i + cfg.kd * dy;
        prev_yaw = ey;

        let t = throttle as f32;
        let m1 = (t + pid_pitch - pid_roll + pid_yaw).clamp(0.0, DSHOT_MAX as f32) as u16;
        let m2 = (t + pid_pitch + pid_roll - pid_yaw).clamp(0.0, DSHOT_MAX as f32) as u16;
        let m3 = (t - pid_pitch + pid_roll + pid_yaw).clamp(0.0, DSHOT_MAX as f32) as u16;
        let m4 = (t - pid_pitch - pid_roll - pid_yaw).clamp(0.0, DSHOT_MAX as f32) as u16;

        let clamp_min = |v: u16| if v < 100 { 100 } else { v };
        *MOTORS.lock().await = MotorValues {
            m1: clamp_min(m1),
            m2: clamp_min(m2),
            m3: clamp_min(m3),
            m4: clamp_min(m4),
        };
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut m1: Output<'static>,
    mut m2: Output<'static>,
    mut m3: Output<'static>,
    mut m4: Output<'static>,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_micros(2000));
    loop {
        ticker.next().await;
        let mv = *MOTORS.lock().await;
        let f1 = dshot_frame(mv.m1);
        let f2 = dshot_frame(mv.m2);
        let f3 = dshot_frame(mv.m3);
        let f4 = dshot_frame(mv.m4);
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, f1, f2, f3, f4);
    }
}

fn dshot_frame(throttle: u16) -> u16 {
    let p = throttle << 1;
    let crc = (p ^ (p >> 4) ^ (p >> 8)) & 0x0F;
    (p << 4) | crc
}

fn delay_ns(ns: u32) {
    let freq   = embassy_rp::clocks::clk_sys_freq();
    let cycles = (freq as u64 * ns as u64) / 1_000_000_000;
    asm_delay(cycles as u32);
}

fn dshot_send_sync(
    m1: &mut Output<'_>, m2: &mut Output<'_>,
    m3: &mut Output<'_>, m4: &mut Output<'_>,
    f1: u16, f2: u16, f3: u16, f4: u16,
) {
    cortex_m::interrupt::free(|_| {
        for i in (0..16).rev() {
            let b1 = (f1 >> i) & 1;
            let b2 = (f2 >> i) & 1;
            let b3 = (f3 >> i) & 1;
            let b4 = (f4 >> i) & 1;

            m1.set_high(); m2.set_high(); m3.set_high(); m4.set_high();

            delay_ns(T0H_NS);

            if b1 == 0 { m1.set_low(); }
            if b2 == 0 { m2.set_low(); }
            if b3 == 0 { m3.set_low(); }
            if b4 == 0 { m4.set_low(); }

            delay_ns(T1H_NS - T0H_NS);

            m1.set_low(); m2.set_low(); m3.set_low(); m4.set_low();
            delay_ns(T1L_NS);
        }
    });
    delay_ns(5000);
}

fn i16_be(h: u8, l: u8) -> i16 {
    ((h as i16) << 8) | l as i16
}

fn atan2_approx(y: f32, x: f32) -> f32 {
    use core::f32::consts::PI;
    if x == 0.0 {
        return if y > 0.0 { PI / 2.0 } else { -PI / 2.0 };
    }
    let r = y / x;
    let atan = if r.abs() <= 1.0 {
        r / (1.0 + 0.28125 * r * r)
    } else {
        let ri = 1.0 / r;
        (if r > 0.0 { PI / 2.0 } else { -PI / 2.0 }) - ri / (1.0 + 0.28125 * ri * ri)
    };
    if x < 0.0 {
        if y >= 0.0 { atan + PI } else { atan - PI }
    } else {
        atan
    }
}

fn libm_sqrt(x: f32) -> f32 {
    if x <= 0.0 { return 0.0; }
    let mut r = x;
    for _ in 0..8 { r = 0.5 * (r + x / r); }
    r
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    
    spawner.spawn(usb_task(Driver::new(p.USB, Irqs))).unwrap();
    Timer::after(Duration::from_millis(2000)).await;
    
    usb_log(b"BOOTING...\r\n");

    let mut pwr = Output::new(p.PIN_23, Level::Low);
    Timer::after(Duration::from_millis(250)).await;

    let mut rng = RoscRng;
    let fw  = include_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../cyw43-firmware/43439A0_clm.bin");

    let cs  = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(&mut pio.common, pio.sm0, DEFAULT_CLOCK_DIVIDER, pio.irq0, cs, p.PIN_24, p.PIN_29, p.DMA_CH0);

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;
    spawner.spawn(cyw43_task(runner)).unwrap();
    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    let seed = rng.next_u64();
    static RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();
    let (stack, net_runner) = embassy_net::new(
        net_device,
        Config::dhcpv4(Default::default()),
        RESOURCES.init(StackResources::new()),
        seed,
    );
    spawner.spawn(net_task(net_runner)).unwrap();

    usb_log(b"WIFI...\r\n");
    loop {
        match control.join(WIFI_SSID, cyw43::JoinOptions::new(WIFI_PASS.as_bytes())).await {
            Ok(_) => { break; }
            Err(_) => { Timer::after(Duration::from_secs(1)).await; }
        }
    }
    stack.wait_config_up().await;
    
    if let Some(cfg) = stack.config_v4() {
        let mut msg = String::<64>::new();
        let _ = core::write!(&mut msg, "IP: {}\r\n", cfg.address);
        usb_log(msg.as_bytes());
    }

    let i2c = i2c::I2c::new_async(p.I2C1, p.PIN_7, p.PIN_6, Irqs, I2cConfig::default());

    let mut m1 = Output::new(p.PIN_0, Level::Low);
    let mut m2 = Output::new(p.PIN_1, Level::Low);
    let mut m3 = Output::new(p.PIN_2, Level::Low);
    let mut m4 = Output::new(p.PIN_3, Level::Low);

    usb_log(b"ARMING...\r\n");
    let t_end = Instant::now() + Duration::from_millis(4000); 
    while Instant::now() < t_end {
        dshot_send_sync(&mut m1, &mut m2, &mut m3, &mut m4, 0, 0, 0, 0);
        cortex_m::asm::delay(62500);
    }

    spawner.spawn(imu_task(i2c)).unwrap();
    spawner.spawn(pid_task()).unwrap();
    spawner.spawn(motor_task(m1, m2, m3, m4)).unwrap();
    spawner.spawn(udp_task(stack)).unwrap();

    usb_log(b"READY\r\n");
    
    loop { Timer::after(Duration::from_secs(60)).await; }
}