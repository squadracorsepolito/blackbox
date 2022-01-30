#![no_std]
#![no_main]

pub mod buffer;
pub mod timestamp;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::DerefMut;

use buffer::Buffer;
use embedded_can::{ExtendedId, Id};
use gd32vf103xx_hal::rtc::Rtc;
use panic_halt as _;


use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::{Dimensions, OriginDimensions, Point, Primitive, RgbColor, Size};
use embedded_graphics::primitives::{PrimitiveStyle, Rectangle};
use embedded_graphics::text::{Alignment, Text};
use embedded_graphics::Drawable;

use gd32vf103xx_hal::prelude::*;
use embedded_hal::digital::v2::InputPin;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits, Priority, TriggerType};
use gd32vf103xx_hal::can::{Can, Frame, NoRemap, FIFO, Config};
use gd32vf103xx_hal::can::filter::{Filter, FilterMode, FilterEntry};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::exti::{Exti, ExtiLine, TriggerEdge};
use gd32vf103xx_hal::gpio::gpioa::PA8;
use gd32vf103xx_hal::gpio::{Floating, Input};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC};

use longan_nano::hal::pac;
use longan_nano::{lcd, lcd_pins, sprintln};
use riscv::interrupt::Mutex;
use riscv_rt::entry;

use crate::timestamp::Timestamp;

static BUTTON: Mutex<RefCell<Option<PA8<Input<Floating>>>>> = Mutex::new(RefCell::new(None));
static COUNT: Mutex<RefCell<Option<u32>>> = Mutex::new(RefCell::new(Some(0)));
static CHANGED: Mutex<RefCell<Option<bool>>> = Mutex::new(RefCell::new(Some(true)));

static CAN0: Mutex<RefCell<Option<Can<CAN0, NoRemap>>>> = Mutex::new(RefCell::new(None));
static CAN1: Mutex<RefCell<Option<Can<CAN1, NoRemap>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    start();
    unreachable!()
}

fn start() {
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

    // rtc
    let mut backup_domain = dp.BKP.configure(&mut rcu, &mut dp.PMU);
    let rtc = Rtc::rtc(dp.RTC, &mut backup_domain);

    // timestamp 
    let mut timestamp = Timestamp::new(&rcu.clocks);

    // can
    let pa12 = gpioa.pa12.into_alternate_push_pull();
    let pa11 = gpioa.pa11.into_floating_input();

    let pb13 = gpiob.pb13.into_alternate_push_pull();
    let pb12 = gpiob.pb12.into_floating_input();

    let mut config = Config::default();
    config.loopback_communication = true;

    let mut can0 =
        Can::<CAN0, NoRemap>::new(dp.CAN0, (pa12, pa11), config, &mut afio, &mut rcu);

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

    let mut config = Config::default();
    config.loopback_communication = true;

    let mut can1 =
        Can::<CAN1, NoRemap>::new(dp.CAN1, (pb13, pb12), config, &mut afio, &mut rcu);

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

    let frame = Frame {
        id: Id::Extended(ExtendedId::new(114514).unwrap()),
        ft: false,
        dlen: 8,
        data: [11, 22, 33, 44, 55, 66, 77, 88],
    };

    nb::block!(can0.transmit(&frame)).unwrap();

    sprintln!("Sent Can0!");

    let a = nb::block!(can0.receive()).unwrap();

    sprintln!("Can! {:?}", a);

    sprintln!("Sending Can1! ...");

    nb::block!(can1.transmit(&frame)).unwrap();

    sprintln!("Sent Can1!");

    let a = nb::block!(can1.receive()).unwrap();

    sprintln!("Can1! {:?}", a);

    // button
    let button = gpioa.pa8.into_floating_input();

    // interrupts
    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);

    ECLIC::setup(
        Interrupt::EXTI_LINE9_5,
        TriggerType::Level,
        Level::L1,
        Priority::P1,
    );

    afio.extiss(button.port(), button.pin_number());

    unsafe {
        ECLIC::unmask(Interrupt::EXTI_LINE9_5);
    };

    let mut exti = Exti::new(dp.EXTI);

    let extiline = ExtiLine::from_gpio_line(button.pin_number()).unwrap();
    exti.listen(extiline, TriggerEdge::Both);
    Exti::clear(extiline);

    unsafe { riscv::interrupt::enable() };

    // screen
    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut screen = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);
    let (width, height) = (screen.size().width as u32, screen.size().height as u32);

    let character_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    Rectangle::new(Point::new(0, 0), Size::new(width, height))
        .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
        .draw(&mut screen)
        .unwrap();

    let mut screen_text = [0u8; 20 * 5];
    let mut screen_text = Buffer::new(&mut screen_text[..]);

    riscv::interrupt::free(|cs| {
        BUTTON.borrow(cs).replace(Some(button));
        CAN0.borrow(cs).replace(Some(can0));
        CAN1.borrow(cs).replace(Some(can1));
    });

    loop {
        riscv::interrupt::free(|cs| {
            if let (Some(ref mut count), Some(ref mut changed)) = (
                COUNT.borrow(cs).borrow_mut().deref_mut(),
                CHANGED.borrow(cs).borrow_mut().deref_mut(),
            ) {
                if *changed {
                    sprintln!("{} {}", rtc.current_time(), timestamp.now());
                    screen_text.clear();
                    write!(&mut screen_text, "Clicked {:0>3} times", *count).unwrap();
                    Text::with_alignment(
                        screen_text.as_str(),
                        screen.bounding_box().center(),
                        character_style,
                        Alignment::Center,
                    )
                    .draw(&mut screen)
                    .unwrap();
                    *changed = false;
                }
            }
        });
        delay.delay_ms(200);
    }
}

/// Handle boot0 button press
#[allow(non_snake_case)]
#[no_mangle]
fn EXTI_LINE9_5() {
    let extiline = ExtiLine::from_gpio_line(8).unwrap();
    if Exti::is_pending(extiline) {
        riscv::interrupt::free(|cs| {
            if let (Some(ref mut button), Some(ref mut count), Some(ref mut changed)) = (
                BUTTON.borrow(cs).borrow_mut().deref_mut(),
                COUNT.borrow(cs).borrow_mut().deref_mut(),
                CHANGED.borrow(cs).borrow_mut().deref_mut(),
            ) {
                if button.is_high().unwrap() {
                    *count += 1;
                    *changed = true;
                    sprintln!("Interrupt {}", count);
                }
            }
        });
        Exti::unpend(extiline);
        Exti::clear(extiline);
    }
}

/// Handle all unhandled interrupts
#[allow(non_snake_case)]
#[no_mangle]
fn DefaultHandler() {
    let code = riscv::register::mcause::read().code() & 0xFFF;
    let cause = riscv::register::mcause::Exception::from(code);

    sprintln!("default handler: code={}, cause={:?}", code, cause);
    loop {}
}
