#![no_std]
#![no_main]

pub mod buffer;
pub mod timestamp;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;
use core::panic;

use embedded_graphics::primitives::{Circle, Line, PrimitiveStyle};
use panic_halt as _;

use buffer::Buffer;
use timestamp::Timestamp;

use riscv::interrupt::Mutex;
use riscv_rt::entry;

use embedded_can::{ExtendedId, Id};
use embedded_graphics::draw_target::DrawTarget;

use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::{MonoTextStyle, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::{Alignment, Baseline, Text};
use embedded_graphics::Drawable;

use gd32vf103xx_hal::can::{
    self, Can, Config, Filter, FilterEntry, FilterMode, Frame, NoRemap, Remap1, FIFO,
};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC};
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::rtc::Rtc;

use longan_nano::hal::pac;
use longan_nano::lcd::Lcd;
use longan_nano::sdcard::SdCard;
use longan_nano::{lcd, lcd_pins, sdcard, sdcard_pins, sprintln};

static DELAY: Mutex<RefCell<Option<McycleDelay>>> = Mutex::new(RefCell::new(None));
static RTC: Mutex<RefCell<Option<Rtc>>> = Mutex::new(RefCell::new(None));
static TIMESTAMP: Mutex<RefCell<Option<Timestamp>>> = Mutex::new(RefCell::new(None));

static CAN0: Mutex<RefCell<Option<Can<CAN0, NoRemap>>>> = Mutex::new(RefCell::new(None));
static CAN1: Mutex<RefCell<Option<Can<CAN1, Remap1>>>> = Mutex::new(RefCell::new(None));

static SDCARD: Mutex<RefCell<Option<SdCard>>> = Mutex::new(RefCell::new(None));

static LCD: Mutex<RefCell<Option<Lcd>>> = Mutex::new(RefCell::new(None));

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
        9600.bps(),
        &mut afio,
        &mut rcu,
    );

    sprintln!("-= telemetry =-");

    // rtc
    let mut backup_domain = dp.BKP.configure(&mut rcu, &mut dp.PMU);
    let rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    // timestamp
    let timestamp = Timestamp::new(&rcu.clocks);

    // can
    let pa12 = gpioa.pa12.into_alternate_push_pull();
    let pa11 = gpioa.pa11.into_floating_input();

    let pb6 = gpiob.pb6.into_alternate_push_pull();
    let pb5 = gpiob.pb5.into_floating_input();

    let config = Config {
        loopback_communication: true,
        ..Default::default()
    };

    let mut can0 = Can::<CAN0, NoRemap>::new(dp.CAN0, (pa12, pa11), config, &mut afio, &mut rcu);

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
            fifo_index: FIFO::FIFO0,
        },
    );

    can0.enable_interrupt(can::Interrupt::RFFIE0);
    can0.enable_interrupt(can::Interrupt::RFFIE1);

    let config = Config {
        loopback_communication: true,
        ..Default::default()
    };

    let mut can1 = Can::<CAN1, Remap1>::new(dp.CAN1, (pb6, pb5), config, &mut afio, &mut rcu);

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

    // screen
    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);

    let lcd_text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    lcd.clear(Rgb565::BLACK).unwrap();

    // All hardware is now initialized, so let's do preliminary checks that can
    // be debugged with the LCD.

    if let Err(_) = sdcard.device().init() {
        Text::with_alignment(
            "ERROR!\nSD Card not initialized",
            lcd.bounding_box().center(),
            MonoTextStyle::new(&FONT_6X10, Rgb565::RED),
            Alignment::Center,
        )
        .draw(&mut lcd)
        .unwrap();
        sprintln!("Failed to initialize sdcard!");
        panic!();
    }

    riscv::interrupt::free(|cs| {
        DELAY.borrow(cs).replace(Some(delay));
        RTC.borrow(cs).replace(Some(rtc));
        TIMESTAMP.borrow(cs).replace(Some(timestamp));
        CAN0.borrow(cs).replace(Some(can0));
        CAN1.borrow(cs).replace(Some(can1));
        SDCARD.borrow(cs).replace(Some(sdcard));
        LCD.borrow(cs).replace(Some(lcd));
    });

    riscv::interrupt::free(|cs| {
        if let Some(ref mut can0) = CAN0.borrow(cs).borrow_mut().deref_mut() {
            let frame = Frame {
                id: Id::Extended(ExtendedId::new(114514).unwrap()),
                ft: false,
                dlen: 8,
                data: [11, 22, 33, 44, 55, 66, 77, 88],
            };
            nb::block!(can0.transmit(&frame)).unwrap();
        }
    });

    let mut lcd_timer_text = [0u8; 20 * 5];
    let mut lcd_timer_text = Buffer::new(&mut lcd_timer_text[..]);

    loop {
        riscv::interrupt::free(|cs| {
            if let (Some(ref mut lcd), Some(ref mut rtc)) = (
                LCD.borrow(cs).borrow_mut().deref_mut(),
                RTC.borrow(cs).borrow_mut().deref_mut(),
            ) {
                let now = rtc.current_time();
                let hours = now / 3600;
                let minutes = (now / 60) % 60;
                let seconds = now % 60;

                lcd_timer_text.clear();
                write!(
                    &mut lcd_timer_text,
                    "{:0>2}:{:0>2}:{:0>2}",
                    hours, minutes, seconds
                )
                .unwrap();

                Text::with_baseline(
                    lcd_timer_text.as_str(),
                    Point::new(5, 5),
                    lcd_text_style,
                    Baseline::Top,
                )
                .draw(lcd)
                .unwrap();

                Circle::with_center(Point::new(160 - 10, 8), 10)
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
                    .draw(lcd)
                    .unwrap();

                Line::new(Point::new(2, 20), Point::new(160 - 2, 20))
                    .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
                    .draw(lcd)
                    .unwrap();
            }
        });
        delay.delay_ms(500)
    }
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN0_RX0() {
    riscv::interrupt::free(|cs| {
        if let Some(ref mut can0) = CAN0.borrow(cs).borrow_mut().deref_mut() {
            let a = nb::block!(can0.receive()).unwrap();
            sprintln!("CAN0_RX0! {:?}", a);
        }
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN0_RX1() {
    riscv::interrupt::free(|cs| {
        if let Some(ref mut can0) = CAN0.borrow(cs).borrow_mut().deref_mut() {
            let a = nb::block!(can0.receive()).unwrap();
            sprintln!("CAN0_RX1! {:?}", a);
        }
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN1_RX0() {
    riscv::interrupt::free(|cs| {
        if let Some(ref mut can1) = CAN1.borrow(cs).borrow_mut().deref_mut() {
            let a = nb::block!(can1.receive()).unwrap();
            sprintln!("CAN1_RX0! {:?}", a);
        }
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn CAN1_RX1() {
    riscv::interrupt::free(|cs| {
        if let Some(ref mut can1) = CAN1.borrow(cs).borrow_mut().deref_mut() {
            let a = nb::block!(can1.receive()).unwrap();
            sprintln!("CAN1_RX1! {:?}", a);
        }
    });
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
