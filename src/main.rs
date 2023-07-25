#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use embassy_executor::Executor;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Ipv4Address, Stack, StackResources};
use embassy_time::with_timeout;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println::println;
use hal::{
    clock::ClockControl,
    embassy,
    peripherals::Peripherals,
    prelude::*,
    timer::TimerGroup,
    uart::{
        config::{DataBits, Parity, StopBits},
        TxRxPins,
    },
    Rtc, IO,
};

use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_backtrace as _;
use esp_println::logger::init_logger;
use esp_wifi::wifi::{WifiController, WifiDevice, WifiEvent, WifiMode, WifiState};
use esp_wifi::{initialize, EspWifiInitFor};
use hal::clock::CpuClock;
use hal::peripherals::TIMG0;
use hal::peripherals::UART1;
use hal::timer::Wdt;
use hal::Rng;
use hal::Uart;

use static_cell::StaticCell;

#[toml_cfg::toml_config]
pub struct Config {
    #[default("")]
    wifi_ssid: &'static str,
    #[default("")]
    wifi_psk: &'static str,
}

macro_rules! singleton {
    ($val:expr) => {{
        type T = impl Sized;
        static STATIC_CELL: StaticCell<T> = StaticCell::new();
        let (x,) = STATIC_CELL.init(($val,));
        x
    }};
}

static EXECUTOR: StaticCell<Executor> = StaticCell::new();

#[embassy_executor::task]
async fn connection(mut controller: WifiController<'static>) {
    println!("start connection task");
    println!("Device capabilities: {:?}", controller.get_capabilities());
    loop {
        match esp_wifi::wifi::get_wifi_state() {
            WifiState::StaConnected => {
                // wait until we're no longer connected
                controller.wait_for_event(WifiEvent::StaDisconnected).await;
                Timer::after(Duration::from_millis(5000)).await
            }
            _ => {}
        }
        if !matches!(controller.is_started(), Ok(true)) {
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: CONFIG.wifi_ssid.into(),
                password: CONFIG.wifi_psk.into(),
                ..Default::default()
            });
            controller.set_configuration(&client_config).unwrap();
            println!("Starting wifi");
            controller.start().await.unwrap();
            println!("Wifi started!");
        }
        println!("About to connect...");

        match controller.connect().await {
            Ok(_) => println!("Wifi connected!"),
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
                Timer::after(Duration::from_millis(5000)).await
            }
        }
    }
}

#[embassy_executor::task]
async fn watchdog(mut wdt: Wdt<TIMG0>) {
    loop {
        wdt.feed();
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
async fn net_task(stack: &'static Stack<WifiDevice<'static>>) {
    stack.run().await
}

// AJ-SR04M sensor frame format for Mode4 (R19 = 47k):
// Trigger:  0x01
// Response: Byte1          Byte2  Byte3  Byte4
//           Start Byte=FF  MSB    LSB    Checksum=(StartByte+LSB+MSB)
async fn ping_uart(uart: &mut Uart<'static, UART1>) -> f32 {
    let mut READ_TIMEOUT: Duration = Duration::from_secs(1);
    let mut rbuf = [0; 12];

    const READ_BUF_SIZE: usize = 4;
    let mut show = true;
    let mut cnt = 8;

    loop {
        //discard all input data (if any)
        loop {
            Timer::after(Duration::from_millis(1500)).await;
            let _ = uart.set_rx_fifo_full_threshold(1 as u16);
            match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[0..])).await {
                Ok(r) => match r {
                    Ok(len) => {
                        if len == 0 {
                            println!("len == 0 (EOF), doing break");
                            break;
                        } else {
                            println!("Discarded {} bytes: {:02X?}", len, rbuf);
                        }
                    }
                    Err(_) => {}
                },
                Err(_) => {
                    println!("read timeout outer");
                    break;
                }
            }
        }

        let _ = uart.set_rx_fifo_full_threshold(4 as u16);
        println!("\nwriting 0x01...");
        for n in 0..12 {
            rbuf[n] = 0
        }
        embedded_hal_async::serial::Write::write(uart, b"\x01")
            .await
            .unwrap();

        let mut offset = 0;
        loop {
            if !show {
                cnt = cnt - 1;
                if cnt == 0 {
                    println!("HALT");
                    loop {
                        Timer::after(Duration::from_millis(5000)).await;
                    }
                }
            }
            println!("inner loop, offset = {}, buf={:02X?}", offset, rbuf);
            match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[offset..])).await {
                Ok(r) => {
                    match r {
                        Ok(len) => {
                            offset += len;
                            println!("offset = {}, len = {}", offset, len);
                            println!("> buf={:02X?}", rbuf);
                            if offset < READ_BUF_SIZE && len > 0 {
                                println!("READ {} instead of 4", len);
                                println!("sync problem - repairing after 5 secs, read timeout=10");
                                let _ = uart.set_rx_fifo_full_threshold(1 as u16);
                                READ_TIMEOUT = Duration::from_secs(10);
                                Timer::after(Duration::from_millis(5000)).await;
                                show = false;
                                continue;
                            }
                            if len == 0 {
                                //EOF
                                if offset == READ_BUF_SIZE {
                                    println!("len == 0 (EOF), doing break");
                                    break;
                                } else {
                                    println!("len == 0, continue because we don't have all data");
                                    continue;
                                }
                            }
                            println!("read len={}, buf={:02X?}", len, rbuf);
                            let crc: u8 = rbuf[0] + rbuf[1] + rbuf[2];
                            let value = rbuf[2] as u16 + ((rbuf[1] as u16) << 8);
                            println!(
                                "ORIG={:02x}, CRC={:X}, value = {}",
                                rbuf[3],
                                crc,
                                value as f32 / 10.0
                            );
                            if rbuf[0] != 0xff {
                                println!("sync problem - repairing");
                                println!("read before len={}, buf={:02X?}", len, rbuf);
                                let _ = uart.set_rx_fifo_full_threshold(1 as u16);
                                match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[0..])).await {
                                    Ok(r) => {
                                        if let Ok(len) = r {
                                            println!("#1 len = {}", len)
                                        }
                                    }
                                    Err(e) => {
                                        println!("#1 error: {:?}", e);
                                    }
                                }
                                match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[0..])).await {
                                    Ok(r) => {
                                        if let Ok(len) = r {
                                            println!("#2 len = {}", len)
                                        }
                                    }
                                    Err(e) => {
                                        println!("#2 error: {:?}", e);
                                    }
                                }
                                match with_timeout(READ_TIMEOUT, uart.read(&mut rbuf[0..])).await {
                                    Ok(r) => {
                                        if let Ok(len) = r {
                                            println!("#3 len = {}", len)
                                        }
                                    }
                                    Err(e) => {
                                        println!("#3 error: {:?}", e);
                                    }
                                }
                                println!("read after  len={}, buf={:02X?}", len, rbuf);
                                show = false;
                            } else {
                                return value as f32 / 10.0;
                            }
                            break;
                            //println!("{}", core::str::from_utf8(&rbuf[..9]).unwrap());
                        }
                        Err(e) => {
                            // buffer is full or rx fifo overflow
                            println!("buffer is full: {:?}", e);
                            continue;
                        }
                    }
                }
                Err(_) => {
                    // Timeout
                    println!("read timeout inner");
                    break;
                }
            }
        }
    }
}

#[embassy_executor::task]
async fn worker(
    stack: &'static Stack<WifiDevice<'static>>,
    mut uart: Uart<'static, UART1>,
    rtc: Rtc<'static>,
) {
    let mut rx_buffer = [0; 4096];
    let mut tx_buffer = [0; 4096];

    loop {
        if stack.is_link_up() {
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    println!("Waiting to get IP address...");
    loop {
        if let Some(config) = stack.config_v4() {
            println!("Got IP: {}", config.address);
            break;
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    loop {
        Timer::after(Duration::from_millis(1_000)).await;

        let mut socket = TcpSocket::new(&stack, &mut rx_buffer, &mut tx_buffer);

        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        let remote_endpoint = (Ipv4Address::new(192, 168, 1, 1), 1234);
        println!("connecting...");
        let r = socket.connect(remote_endpoint).await;
        if let Err(e) = r {
            println!("connect error: {:?}", e);
            continue;
        }
        println!("connected!");
        let mut buf = [0; 1024];
        loop {
            use embedded_io::asynch::Write;

            //Timer::after(Duration::from_millis(5000)).await;
            let cm = ping_uart(&mut uart).await;
            println!("Distance {:.1} cm\r", cm);

            let _: &str = format_no_std::show(
                &mut buf,
                format_args!("Distance {:.1} cm, uptime = {}\r\n", cm, rtc.get_time_ms()),
            )
            .unwrap();

            let r = socket.write_all(&buf).await;
            if let Err(e) = r {
                println!("write error: {:?}", e);
                break;
            }
            /*let n = match socket.read(&mut buf).await {
                Ok(0) => {
                    println!("read EOF");
                    break;
                }
                Ok(n) => n,
                Err(e) => {
                    println!("read error: {:?}", e);
                    break;
                }
            };
            println!("{}", core::str::from_utf8(&buf[..n]).unwrap());*/
        }
        Timer::after(Duration::from_millis(3000)).await;
    }
}

#[entry]
fn main() -> ! {
    init_logger(log::LevelFilter::Info);

    // Take Peripherals, Initialize Clocks, and Create a Handle for Each
    let peripherals = Peripherals::take();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::configure(system.clock_control, CpuClock::Clock240MHz).freeze();

    // Disable the RTC and TIMG watchdog timers
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);
    rtc.rwdt.disable();

    let timer_group1 = TimerGroup::new(
        peripherals.TIMG1,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let init = initialize(
        EspWifiInitFor::Wifi,
        timer_group1.timer0,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();
    let (wifi, _) = peripherals.RADIO.split();
    let (wifi_interface, controller) =
        esp_wifi::wifi::new_with_mode(&init, wifi, WifiMode::Sta).unwrap();

    let timer_group0 = TimerGroup::new(
        peripherals.TIMG0,
        &clocks,
        &mut system.peripheral_clock_control,
    );
    let mut wdt = timer_group0.wdt;
    wdt.start(20u64.secs());
    embassy::init(&clocks, timer_group0.timer0);

    // Instantiate and Create Handle for IO
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    let config = embassy_net::Config::dhcpv4(Default::default());

    let seed = 1234; // very random, very secure seed

    // Init network stack
    let stack = &*singleton!(Stack::new(
        wifi_interface,
        config,
        singleton!(StackResources::<3>::new()),
        seed
    ));

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let config = hal::uart::config::Config {
        baudrate: 9600,
        data_bits: DataBits::DataBits8,
        parity: Parity::ParityNone,
        stop_bits: StopBits::STOP1,
    };

    let pins = TxRxPins::new_tx_rx(
        io.pins.gpio19.into_push_pull_output(),
        io.pins.gpio21.into_floating_input(),
    );

    let mut serial1 = hal::Uart::new_with_config(
        peripherals.UART1,
        Some(config),
        Some(pins),
        &clocks,
        &mut system.peripheral_clock_control,
    );

    let _ = serial1.set_rx_fifo_full_threshold(1 as u16);
    hal::interrupt::enable(
        hal::peripherals::Interrupt::UART1,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(connection(controller)).ok();
        spawner.spawn(net_task(&stack)).ok();
        spawner.spawn(worker(&stack, serial1, rtc)).ok();
        spawner.spawn(watchdog(wdt)).ok();
    })
}
