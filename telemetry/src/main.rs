#![no_std]
#![no_main]

pub mod buffer;
pub mod faces;
pub mod timestamp;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::panic;

use panic_halt as _;

use timestamp::Timestamp;

use riscv::interrupt::{CriticalSection, Mutex};
use riscv_rt::entry;

use embedded_can::Id;

use embedded_sdmmc::VolumeIdx;

use gd32vf103xx_hal::can::{self, Can, Config, Filter, FilterEntry, FilterMode, FIFO};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC, TIMER1};
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pwm::{self, Channel, PwmTimer};
use gd32vf103xx_hal::rtc::Rtc;

use longan_nano::hal::pac;
use longan_nano::{sdcard, sdcard_pins, sprintln};

pub static TIMESTAMP: Mutex<RefCell<Option<Timestamp>>> = Mutex::new(RefCell::new(None));

pub static CAN0: Mutex<RefCell<Option<Can<CAN0, can::NoRemap>>>> = Mutex::new(RefCell::new(None));
pub static CAN1: Mutex<RefCell<Option<Can<CAN1, can::Remap1>>>> = Mutex::new(RefCell::new(None));

pub const CAN_FRAME_SIZE: usize = 20;
pub const CAN_BUFFER_SIZE: usize = 8 * CAN_FRAME_SIZE;

pub static CAN0_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN0_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

pub static CAN1_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN1_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    entrypoint()
}

fn entrypoint() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();

    let mut rcu = dp
        .RCU
        .configure()
        .ext_hf_clock(8.mhz())
        .sysclk(108.mhz())
        .freeze();

    let mut delay = McycleDelay::new(&rcu.clocks);

    let mut afio = dp.AFIO.constrain(&mut rcu);

    let gpioa = dp.GPIOA.split(&mut rcu);
    let gpiob = dp.GPIOB.split(&mut rcu);

    let timer1 = dp.TIMER1;

    let pa3 = gpioa.pa3.into_alternate_push_pull();

    let mut pwm_t4 = PwmTimer::<TIMER1, pwm::NoRemap>::new(
        timer1,
        (None, None, None, Some(&pa3)),
        &mut rcu,
        &mut afio,
    );

    let max = pwm_t4.get_max_duty();
    pwm_t4.set_period(500_000.hz());
    pwm_t4.set_duty(Channel::CH3, max / 2); // 50% duty cycle
    pwm_t4.enable(Channel::CH3);

    // stdout
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        9600.bps(),
        &mut afio,
        &mut rcu,
    );

    sprintln!("-= telemetry =-");

    sprintln!("{} {}", rcu.clocks.pclk1().0, rcu.clocks.pclk2().0);

    // rtc
    let mut backup_domain = dp.BKP.configure(&mut rcu, &mut dp.PMU);
    let rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    // timestamp
    let timestamp = Timestamp::new(rtc.current_time(), &rcu.clocks);

    // can
    let pa12 = gpioa.pa12.into_alternate_push_pull();
    let pa11 = gpioa.pa11.into_floating_input();

    let pb6 = gpiob.pb6.into_alternate_push_pull();
    let pb5 = gpiob.pb5.into_floating_input();

    let config = Config::default();

    let mut can0 =
        Can::<CAN0, can::NoRemap>::new(dp.CAN0, (pa12, pa11), config, &mut afio, &mut rcu);

    can0.set_filter(
        0,
        Filter {
            mode: FilterMode::Mask,
            entry: FilterEntry::Entry32([0, 0]),
            fifo_index: FIFO::FIFO0,
        },
    );

    can0.set_filter(
        1,
        Filter {
            mode: FilterMode::Mask,
            entry: FilterEntry::Entry32([0, 0]),
            fifo_index: FIFO::FIFO1,
        },
    );

    can0.enable_interrupt(can::Interrupt::RFFIE0);
    can0.enable_interrupt(can::Interrupt::RFFIE1);

    let config = Config::default();

    let mut can1 = Can::<CAN1, can::Remap1>::new(dp.CAN1, (pb6, pb5), config, &mut afio, &mut rcu);

    can1.set_filter(
        0,
        Filter {
            mode: FilterMode::Mask,
            entry: FilterEntry::Entry32([0, 0]),
            fifo_index: FIFO::FIFO0,
        },
    );

    can1.set_filter(
        1,
        Filter {
            mode: FilterMode::Mask,
            entry: FilterEntry::Entry32([0, 0]),
            fifo_index: FIFO::FIFO1,
        },
    );

    can1.enable_interrupt(can::Interrupt::RFFIE0);
    can1.enable_interrupt(can::Interrupt::RFFIE1);

    // sdcard
    let sdcard_pins = sdcard_pins!(gpiob);
    let mut sdcard = sdcard::configure(dp.SPI1, sdcard_pins, sdcard::SdCardFreq::Safe, &mut rcu);

    // interrupts
    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);

    unsafe {
        ECLIC::unmask(Interrupt::CAN0_RX0);
        ECLIC::unmask(Interrupt::CAN0_RX1);
        ECLIC::unmask(Interrupt::CAN1_RX0);
        ECLIC::unmask(Interrupt::CAN1_RX1);
    };

    unsafe { riscv::interrupt::enable() };

    // All hardware is now initialized, so let's do preliminary checks.

    if let Err(error) = sdcard.device().init() {
        sprintln!("Failed to initialize sdcard! {:?}", error);
        panic!();
    }

    let (can0_producer, mut can0_consumer) = CAN0_BUFFER.try_split().unwrap();
    let (can1_producer, mut can1_consumer) = CAN1_BUFFER.try_split().unwrap();
    riscv::interrupt::free(|cs| {
        TIMESTAMP.borrow(cs).replace(Some(timestamp));
        CAN0.borrow(cs).replace(Some(can0));
        CAN1.borrow(cs).replace(Some(can1));
        CAN0_PRODUCER.borrow(cs).replace(Some(can0_producer));
        CAN1_PRODUCER.borrow(cs).replace(Some(can1_producer));
    });

    let mut volume = sdcard.get_volume(VolumeIdx(0)).unwrap();

    // list files in root dir
    let root_dir = sdcard.open_root_dir(&volume).unwrap();
    let mut can0_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "CAN0",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    let mut can1_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "CAN1",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    loop {
        if let Ok(rgr) = can0_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can0_file, rgr.buf()) {
                sprintln!("CAN1 wrote {} bytes", size);
                rgr.release(size);
            } else {
                sprintln!("failed to write can0");
                panic!();
            }
        }
        if let Ok(rgr) = can1_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can1_file, rgr.buf()) {
                sprintln!("CAN1 wrote {} bytes", size);
                rgr.release(size);
            } else {
                sprintln!("failed to write can1");
                panic!();
            }
        }

        delay.delay_ms(100)
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN0_RX0() {
    riscv::interrupt::free(can0_rx);
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN0_RX1() {
    riscv::interrupt::free(can0_rx);
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN1_RX0() {
    riscv::interrupt::free(can1_rx);
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN1_RX1() {
    riscv::interrupt::free(can1_rx);
}

fn can0_rx(cs: &CriticalSection) {
    if let (Some(ref mut can0), Some(ref mut producer), Some(ref mut timestamp)) = (
        CAN0.borrow(cs).borrow_mut().deref_mut(),
        CAN0_PRODUCER.borrow(cs).borrow_mut().deref_mut(),
        TIMESTAMP.borrow(cs).borrow_mut().deref_mut(),
    ) {
        let frame = nb::block!(can0.receive()).unwrap();
        let time: [u8; 8] = timestamp.tick_us().to_ne_bytes();
        let id: [u8; 4] = match frame.id {
            Id::Standard(id) => (id.as_raw() as u32).to_ne_bytes(),
            Id::Extended(id) => id.as_raw().to_ne_bytes(),
        };
        let data: [u8; 8] = frame.data;
        let mut wrg = producer.grant_exact(CAN_FRAME_SIZE).unwrap();
        wrg.buf()[..8].copy_from_slice(&time);
        wrg.buf()[8..12].copy_from_slice(&id);
        wrg.buf()[12..20].copy_from_slice(&data);
        wrg.commit(CAN_FRAME_SIZE);
    }
}

fn can1_rx(cs: &CriticalSection) {
    if let (Some(ref mut can1), Some(ref mut producer), Some(ref mut timestamp)) = (
        CAN1.borrow(cs).borrow_mut().deref_mut(),
        CAN1_PRODUCER.borrow(cs).borrow_mut().deref_mut(),
        TIMESTAMP.borrow(cs).borrow_mut().deref_mut(),
    ) {
        let frame = nb::block!(can1.receive()).unwrap();
        let time: [u8; 8] = timestamp.tick_us().to_ne_bytes();
        let id: [u8; 4] = match frame.id {
            Id::Standard(id) => (id.as_raw() as u32).to_ne_bytes(),
            Id::Extended(id) => id.as_raw().to_ne_bytes(),
        };
        let data: [u8; 8] = frame.data;
        let mut wrg = producer.grant_exact(CAN_FRAME_SIZE).unwrap();
        wrg.buf()[..8].copy_from_slice(&time);
        wrg.buf()[8..12].copy_from_slice(&id);
        wrg.buf()[12..20].copy_from_slice(&data);
        wrg.commit(CAN_FRAME_SIZE);
    }
}

/// Handle all unhandled interrupts
#[allow(non_snake_case)]
#[no_mangle]
fn DefaultHandler() {
    let code = riscv::register::mcause::read().code() & 0xFFF;
    let cause = riscv::register::mcause::Exception::from(code);

    sprintln!("default handler: code={}, cause={:?}", code, cause);
    panic!();
}
