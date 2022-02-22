#![no_std]
#![no_main]

pub mod buffer;
pub mod faces;
pub mod timestamp;

use core::cell::RefCell;
use core::ops::DerefMut;
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use embedded_hal::digital::v2::InputPin;

use timestamp::Timestamp;

use riscv::interrupt::{CriticalSection, Mutex};
use riscv_rt::entry;

use embedded_can::Id;

use embedded_sdmmc::VolumeIdx;

use gd32vf103xx_hal::can::{self, Can, Config, Filter, FilterEntry, FilterMode, FIFO};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC, TIMER3};
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pwm::{self, Channel, PwmTimer};
use gd32vf103xx_hal::rtc::Rtc;

use longan_nano::hal::pac;
use longan_nano::{sdcard, sdcard_pins, sprintln};

pub static TIMESTAMP: Mutex<RefCell<Option<Timestamp>>> = Mutex::new(RefCell::new(None));

pub static CAN0: Mutex<RefCell<Option<Can<CAN0, can::NoRemap>>>> = Mutex::new(RefCell::new(None));
pub static CAN1: Mutex<RefCell<Option<Can<CAN1, can::Remap1>>>> = Mutex::new(RefCell::new(None));

pub const CAN_FRAME_SIZE: usize = 20;
pub const CAN_BUFFER_SIZE: usize = 64 * CAN_FRAME_SIZE;

pub static CAN0_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN0_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

pub static CAN1_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN1_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    sprintln!("{}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

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

    // stdout
    longan_nano::stdout::configure(
        dp.USART0,
        gpioa.pa9,
        gpioa.pa10,
        115_200.bps(),
        &mut afio,
        &mut rcu,
    );

    sprintln!("-= telemetry =-");

    // rtc
    let mut backup_domain = dp.BKP.configure(&mut rcu, &mut dp.PMU);
    let rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    sprintln!("rtc {}", rtc.current_time());

    // timestamp
    let timestamp = Timestamp::new(rtc.current_time(), &rcu.clocks);

    // can
    let pa12 = gpioa.pa12.into_alternate_push_pull();
    let pa11 = gpioa.pa11.into_floating_input();

    let pb6 = gpiob.pb6.into_alternate_push_pull();
    let pb5 = gpiob.pb5.into_floating_input();

    sprintln!("setting up can0");

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

    sprintln!("setting up can1");

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
    sprintln!("setting up sdcard");

    let sdcard_pins = sdcard_pins!(gpiob);
    let mut sdcard = sdcard::configure(dp.SPI1, sdcard_pins, sdcard::SdCardFreq::Safe, &mut rcu);

    if let Err(error) = sdcard.device().init() {
        sprintln!("failed to initialize sdcard! {:?}", error);
    }

    // interrupts
    sprintln!("setting up interrupts");

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

    sprintln!("setting up global resources and buffers");

    let (can0_producer, mut can0_consumer) = CAN0_BUFFER.try_split().unwrap();
    let (can1_producer, mut can1_consumer) = CAN1_BUFFER.try_split().unwrap();
    riscv::interrupt::free(|cs| {
        TIMESTAMP.borrow(cs).replace(Some(timestamp));
        CAN0.borrow(cs).replace(Some(can0));
        CAN1.borrow(cs).replace(Some(can1));
        CAN0_PRODUCER.borrow(cs).replace(Some(can0_producer));
        CAN1_PRODUCER.borrow(cs).replace(Some(can1_producer));
    });

    sprintln!("setting up sdcard volumes");

    let mut volume = sdcard.get_volume(VolumeIdx(0)).unwrap();

    sprintln!("setting up sdcard CAN0 log");

    // list files in root dir
    let root_dir = sdcard.open_root_dir(&volume).unwrap();
    let header = [0u8; 2];

    let mut can0_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "CAN0",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    sprintln!("setting up sdcard CAN1 log");

    if can0_file.length() == 0 {
        sdcard.write(&mut volume, &mut can0_file, &header).unwrap();
    }

    let mut can1_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "CAN1",
            embedded_sdmmc::Mode::ReadWriteCreateOrAppend,
        )
        .unwrap();

    if can1_file.length() == 0 {
        sdcard.write(&mut volume, &mut can1_file, &header).unwrap();
    }

    sprintln!("setting up buzzer");

    let timer3 = dp.TIMER3;

    let pb9 = gpiob.pb9.into_alternate_push_pull();

    let mut buzzer = PwmTimer::<TIMER3, pwm::NoRemap>::new(
        timer3,
        (None, None, None, Some(&pb9)),
        &mut rcu,
        &mut afio,
    );

    let max = buzzer.get_max_duty();
    buzzer.set_period(500.hz());
    buzzer.set_duty(Channel::CH3, max / 2); // 50% duty cycle

    sprintln!("setting up switch");

    let pb8 = gpiob.pb8;

    loop {
        if let Ok(rgr) = can0_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can0_file, rgr.buf()) {
                sprintln!("CAN0 wrote {} bytes", size);
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

        match pb8.is_high() {
            Ok(switch) => {
                if switch {
                    buzzer.enable(Channel::CH3);
                } else if !switch {
                    buzzer.disable(Channel::CH3);
                }
            }
            Err(err) => sprintln!("failed to read switch {:?}", err),
        }

        delay.delay_ms(100_u32);
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
