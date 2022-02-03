#![no_std]
#![no_main]

pub mod buffer;
pub mod timestamp;
pub mod faces;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::{DerefMut, Sub};
use core::panic;

use embedded_graphics::image::{ImageRaw, Image};
use embedded_graphics::pixelcolor::raw::LittleEndian;
use panic_halt as _;

use buffer::Buffer;
use timestamp::Timestamp;

use riscv::interrupt::{CriticalSection, Mutex};
use riscv_rt::entry;

use embedded_graphics::prelude::*;

use embedded_can::{ExtendedId, Id};
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::ascii::FONT_6X10;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::primitives::{
    Arc, Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use embedded_graphics::Drawable;

use embedded_sdmmc::VolumeIdx;

use gd32vf103xx_hal::can::{
    self, Can, Config, Filter, FilterEntry, FilterMode, Frame, NoRemap, Remap1, FIFO,
};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC};
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::rtc::Rtc;

use longan_nano::hal::pac;
use longan_nano::{lcd, lcd_pins, sdcard, sdcard_pins, sprintln};

pub static TIMESTAMP: Mutex<RefCell<Option<Timestamp>>> = Mutex::new(RefCell::new(None));

pub static CAN0: Mutex<RefCell<Option<Can<CAN0, NoRemap>>>> = Mutex::new(RefCell::new(None));
pub static CAN1: Mutex<RefCell<Option<Can<CAN1, Remap1>>>> = Mutex::new(RefCell::new(None));

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
    let timestamp = Timestamp::new(rtc.current_time(), &rcu.clocks);

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

    let lcd_text_style_white = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    let lcd_text_style_red = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(Rgb565::RED)
        .background_color(Rgb565::BLACK)
        .build();

    lcd.clear(Rgb565::BLACK).unwrap();

    // All hardware is now initialized, so let's do preliminary checks that can
    // be debugged with the LCD.

    if let Err(error) = sdcard.device().init() {
        Text::with_alignment(
            "ERROR!\nSD Card not initialized",
            lcd.bounding_box().center(),
            lcd_text_style_red,
            Alignment::Center,
        )
        .draw(&mut lcd)
        .unwrap();
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

    let mut volume = sdcard.get_volume(VolumeIdx(0)).unwrap();

    // list files in root dir
    let root_dir = sdcard.open_root_dir(&volume).unwrap();
    let mut can0_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "can0",
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )
        .unwrap();

    let mut can1_file = sdcard
        .open_file_in_dir(
            &mut volume,
            &root_dir,
            "can1",
            embedded_sdmmc::Mode::ReadWriteCreateOrTruncate,
        )
        .unwrap();

    let sdcard_max_size: usize = sdcard.device().card_size_bytes().unwrap() as usize / 1_000_000;
    let mut sdcard_used_size: usize = can0_file.length() as usize + can1_file.length() as usize;

    let mut lcd_text = [0u8; 20 * 5];
    let mut lcd_text = Buffer::new(&mut lcd_text[..]);

    let raw_image: ImageRaw<Rgb565, LittleEndian> = ImageRaw::new(&faces::COOL, faces::WIDTH);
    Image::new(&raw_image, Point::new(10, 45 - faces::HEIGHT as i32 / 2))
        .draw(&mut lcd)
        .unwrap();

    loop {
        if let Ok(rgr) = can0_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can0_file, rgr.buf()) {
                sdcard_used_size += size;
                rgr.release(size);
            } else {
                sprintln!("failed to write can0");
                panic!();
            }
        }
        if let Ok(rgr) = can1_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can1_file, rgr.buf()) {
                sdcard_used_size += size;
                rgr.release(size);
            } else {
                sprintln!("failed to write can1");
                panic!();
            }
        }

        // rendering
        let now = rtc.current_time();
        let (hours, minutes, seconds) = (now / 3600, (now / 60) % 60, now % 60);

        lcd_text.clear();
        write!(
            &mut lcd_text,
            "{:0>2}:{:0>2}:{:0>2}",
            hours, minutes, seconds
        )
        .unwrap();

        Text::with_baseline(
            lcd_text.as_str(),
            Point::new(5, 5),
            lcd_text_style_white,
            Baseline::Top,
        )
        .draw(&mut lcd)
        .unwrap();

        if true {
            // is recording
            Text::with_text_style(
                "REC",
                Point::new(160 - 16, 5),
                lcd_text_style_red,
                TextStyleBuilder::new()
                    .alignment(Alignment::Right)
                    .baseline(Baseline::Top)
                    .build(),
            )
            .draw(&mut lcd)
            .unwrap();

            Circle::with_center(Point::new(160 - 9, 9), 8)
                .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
                .draw(&mut lcd)
                .unwrap();
        } else {
            Rectangle::with_corners(Point::new(160 - 16, 0), Point::new(160, 18))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut lcd)
                .unwrap();
        }

        Line::new(Point::new(2, 20), Point::new(160 - 2, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut lcd)
            .unwrap();

        let sdcard_used_percentage = sdcard_used_size as f32 / sdcard_max_size as f32;
        let sweep = sdcard_used_percentage * 360.0;

        Arc::new(
            Point::new(130, 50).sub(Point::new(20, 20)),
            40,
            (90.0 + sweep).deg(),
            (360.0 - sweep).deg(),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::new(10, 10, 10))
                .stroke_width(5)
                .stroke_alignment(StrokeAlignment::Inside)
                .build(),
        )
        .draw(&mut lcd)
        .unwrap();

        Arc::new(
            Point::new(130, 50).sub(Point::new(20, 20)),
            40,
            90.0.deg(),
            sweep.deg(),
        )
        .into_styled(
            PrimitiveStyleBuilder::new()
                .stroke_color(Rgb565::BLUE)
                .stroke_width(5)
                .stroke_alignment(StrokeAlignment::Inside)
                .build(),
        )
        .draw(&mut lcd)
        .unwrap();

        lcd_text.clear();
        write!(&mut lcd_text, "{:0>2.0}%", sdcard_used_percentage * 100.).unwrap();

        Text::with_text_style(
            lcd_text.as_str(),
            Point::new(130, 50),
            lcd_text_style_white,
            TextStyleBuilder::new()
                .alignment(Alignment::Center)
                .baseline(Baseline::Middle)
                .build(),
        )
        .draw(&mut lcd)
        .unwrap();

        delay.delay_ms(500)
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
