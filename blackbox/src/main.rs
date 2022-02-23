#![no_std]
#![no_main]

pub mod buffer;
pub mod faces;
pub mod timestamp;
pub mod tones;

use core::cell::RefCell;
use core::fmt::Write;
use core::ops::{DerefMut, Sub};
use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

use riscv::interrupt::{CriticalSection, Mutex};
use riscv_rt::entry;

use embedded_can::Id;
use embedded_graphics::image::{Image, ImageRaw};
use embedded_graphics::mono_font::{self, MonoTextStyleBuilder};
use embedded_graphics::pixelcolor::raw::LittleEndian;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::{
    Arc, Circle, Line, PrimitiveStyle, PrimitiveStyleBuilder, Rectangle, StrokeAlignment,
};
use embedded_graphics::text::{Alignment, Baseline, Text, TextStyleBuilder};
use embedded_hal::digital::v2::InputPin;
use embedded_sdmmc::VolumeIdx;

use gd32vf103xx_hal::can::{self, Can, Config, Filter, FilterEntry, FilterMode, FIFO};
use gd32vf103xx_hal::delay::McycleDelay;
use gd32vf103xx_hal::eclic::{EclicExt, Level, LevelPriorityBits};
use gd32vf103xx_hal::pac::{Interrupt, CAN0, CAN1, ECLIC, TIMER1, TIMER3};
use gd32vf103xx_hal::prelude::*;
use gd32vf103xx_hal::pwm::{self, Channel, PwmTimer};
use gd32vf103xx_hal::rtc::Rtc;
use gd32vf103xx_hal::timer::{self, Timer};

use longan_nano::hal::pac;
use longan_nano::{lcd, lcd_pins, sdcard, sdcard_pins, sprintln};

use timestamp::Timestamp;

use crate::buffer::Buffer;

pub static TIMESTAMP: Mutex<RefCell<Option<Timestamp>>> = Mutex::new(RefCell::new(None));

pub static CAN0: Mutex<RefCell<Option<Can<CAN0, can::NoRemap>>>> = Mutex::new(RefCell::new(None));
pub static CAN1: Mutex<RefCell<Option<Can<CAN1, can::Remap1>>>> = Mutex::new(RefCell::new(None));

pub const BUZZER_CHANNEL: Channel = Channel::CH3;
pub static BUZZER_INDEX: Mutex<RefCell<usize>> = Mutex::new(RefCell::new(0));

pub static BUZZER_TIMER: Mutex<RefCell<Option<PwmTimer<TIMER3, pwm::NoRemap>>>> =
    Mutex::new(RefCell::new(None));
pub static BUZZER_TONE_TIMER: Mutex<RefCell<Option<Timer<TIMER1>>>> =
    Mutex::new(RefCell::new(None));

pub const CAN_FRAME_SIZE: usize = 20;
pub const CAN_BUFFER_SIZE: usize = 64 * CAN_FRAME_SIZE;

pub static CAN0_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN0_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

pub static CAN1_BUFFER: bbqueue::BBBuffer<CAN_BUFFER_SIZE> = bbqueue::BBBuffer::new();
pub static CAN1_PRODUCER: Mutex<RefCell<Option<bbqueue::Producer<'static, CAN_BUFFER_SIZE>>>> =
    Mutex::new(RefCell::new(None));

pub const MARKER_BUFFER: &'static [u8] = &[0; CAN_FRAME_SIZE];

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

    sprintln!("-= blackbox =-");

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

    // tone generator
    sprintln!("setting up buzzer");

    let timer3 = dp.TIMER3;

    let pb9 = gpiob.pb9.into_alternate_push_pull();

    let mut buzzer_timer = PwmTimer::<TIMER3, pwm::NoRemap>::new(
        timer3,
        (None, None, None, Some(&pb9)),
        &mut rcu,
        &mut afio,
    );

    let max = buzzer_timer.get_max_duty();
    buzzer_timer.set_period(500.hz());
    buzzer_timer.set_duty(BUZZER_CHANNEL, max / 2); // 50% duty cycle

    let mut buzzer_tone_timer = Timer::<TIMER1>::timer1(dp.TIMER1, 1.hz(), &mut rcu);
    buzzer_tone_timer.listen(timer::Event::Update);
    buzzer_tone_timer.unlisten(timer::Event::Update);

    sprintln!("setting up switch");

    // marker
    let marker_pin = gpioa.pa8.into_floating_input();

    // switch
    let switch_pin = gpiob.pb8.into_floating_input();

    // lcd
    let lcd_pins = lcd_pins!(gpioa, gpiob);
    let mut lcd = lcd::configure(dp.SPI0, lcd_pins, &mut afio, &mut rcu);

    let lcd_text_style_white = MonoTextStyleBuilder::new()
        .font(&mono_font::ascii::FONT_6X10)
        .text_color(Rgb565::WHITE)
        .background_color(Rgb565::BLACK)
        .build();

    let lcd_text_style_red = MonoTextStyleBuilder::new()
        .font(&mono_font::ascii::FONT_6X10)
        .text_color(Rgb565::RED)
        .background_color(Rgb565::BLACK)
        .build();

    let lcd_text_style_big = MonoTextStyleBuilder::new()
        .font(&mono_font::ascii::FONT_9X18_BOLD)
        .text_color(Rgb565::YELLOW)
        .background_color(Rgb565::BLACK)
        .build();

    lcd.clear(Rgb565::BLACK).unwrap();

    // interrupts
    sprintln!("setting up interrupts");

    ECLIC::reset();
    ECLIC::set_threshold_level(Level::L0);
    ECLIC::set_level_priority_bits(LevelPriorityBits::L3P1);

    unsafe { riscv::interrupt::enable() };

    unsafe {
        ECLIC::unmask(Interrupt::TIMER1);
    };

    // ---

    sprintln!("setting up global resources and buffers");

    let (can0_producer, mut can0_consumer) = CAN0_BUFFER.try_split().unwrap();
    let (can1_producer, mut can1_consumer) = CAN1_BUFFER.try_split().unwrap();
    riscv::interrupt::free(|cs| {
        TIMESTAMP.borrow(cs).replace(Some(timestamp));
        CAN0.borrow(cs).replace(Some(can0));
        CAN1.borrow(cs).replace(Some(can1));
        BUZZER_TIMER.borrow(cs).replace(Some(buzzer_timer));
        BUZZER_TONE_TIMER
            .borrow(cs)
            .replace(Some(buzzer_tone_timer));
        CAN0_PRODUCER.borrow(cs).replace(Some(can0_producer));
        CAN1_PRODUCER.borrow(cs).replace(Some(can1_producer));
    });

    sprintln!("setting up sdcard");

    if let Err(error) = sdcard.device().init() {
        Text::with_alignment(
            "ERROR!\nSD Card not initialized",
            lcd.bounding_box().center(),
            lcd_text_style_red,
            Alignment::Center,
        )
        .draw(&mut lcd)
        .unwrap();
        panic!("failed to initialize sdcard! {:?}", error);
    }

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

    sprintln!("entering main loop!");

    // screen info
    let mut enable_log = switch_pin.is_high().unwrap();
    if enable_log {
        unsafe {
            ECLIC::unmask(Interrupt::CAN0_RX0);
            ECLIC::unmask(Interrupt::CAN0_RX1);
            ECLIC::unmask(Interrupt::CAN1_RX0);
            ECLIC::unmask(Interrupt::CAN1_RX1);
        };
    }

    let mut marker = marker_pin.is_high().unwrap();
    let mut marker_changed = false;

    let mut lcd_text = [0u8; 20 * 5];
    let mut lcd_buffer = Buffer::new(&mut lcd_text[..]);

    let sdcard_max_size = sdcard.device().card_size_bytes().unwrap() as usize;
    let mut sdcard_used_size = (can0_file.length() + can1_file.length()) as usize;

    sprintln!("sdcard capacity: {}", sdcard_max_size);
    sprintln!("sdcard used size: {}", sdcard_used_size);

    let lcd_size = lcd.size();
    let (lcd_width, lcd_height) = (lcd_size.width as i32, lcd_size.height as i32);

    let raw_image: ImageRaw<Rgb565, LittleEndian> = ImageRaw::new(&faces::COOL, faces::WIDTH);

    loop {
        if let Ok(rgr) = can0_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can0_file, rgr.buf()) {
                sprintln!("CAN0 wrote {} bytes", size);
                rgr.release(size);
                sdcard_used_size += size;
            } else {
                panic!("failed to write can0");
            }
        }
        if let Ok(rgr) = can1_consumer.read() {
            if let Ok(size) = sdcard.write(&mut volume, &mut can1_file, rgr.buf()) {
                sprintln!("CAN1 wrote {} bytes", size);
                rgr.release(size);
                sdcard_used_size += size;
            } else {
                panic!("failed to write can1");
            }
        }

        if marker {
            if let Ok(size) = sdcard.write(&mut volume, &mut can0_file, MARKER_BUFFER) {
                sprintln!("Marker!");
                sdcard_used_size += size;
            } else {
                panic!("failed to write can0");
            }
            if let Ok(size) = sdcard.write(&mut volume, &mut can1_file, MARKER_BUFFER) {
                sprintln!("Marker!");
                sdcard_used_size += size;
            } else {
                panic!("failed to write can1");
            }
            marker = false;
            marker_changed = true;
        }

        if enable_log && !marker && marker_pin.is_high().unwrap() {
            marker = true;
            marker_changed = true;
        }

        if !enable_log && switch_pin.is_high().unwrap() {
            ECLIC::mask(Interrupt::CAN0_RX0);
            ECLIC::mask(Interrupt::CAN0_RX1);
            ECLIC::mask(Interrupt::CAN1_RX0);
            ECLIC::mask(Interrupt::CAN1_RX1);
            enable_log = true;
            buzzer_startup();
            sprintln!("logging enabled");
        } else if enable_log && switch_pin.is_low().unwrap() {
            unsafe {
                ECLIC::unmask(Interrupt::CAN0_RX0);
                ECLIC::unmask(Interrupt::CAN0_RX1);
                ECLIC::unmask(Interrupt::CAN1_RX0);
                ECLIC::unmask(Interrupt::CAN1_RX1);
            };
            enable_log = false;
            sprintln!("logging disabled");
        }

        let now = rtc.current_time();
        let (hours, minutes, seconds) = (now / 3600, (now / 60) % 60, now % 60);

        lcd_buffer.clear();
        write!(
            &mut lcd_buffer,
            "{:0>2}:{:0>2}:{:0>2}",
            hours, minutes, seconds
        )
        .unwrap();

        Text::with_baseline(
            lcd_buffer.as_str(),
            Point::new(5, 5),
            lcd_text_style_white,
            Baseline::Top,
        )
        .draw(&mut lcd)
        .unwrap();

        if enable_log {
            Text::with_text_style(
                "REC",
                Point::new(lcd_width - 16, 5),
                lcd_text_style_red,
                TextStyleBuilder::new()
                    .alignment(Alignment::Right)
                    .baseline(Baseline::Top)
                    .build(),
            )
            .draw(&mut lcd)
            .unwrap();

            Circle::with_center(Point::new(lcd_width - 9, 9), 8)
                .into_styled(PrimitiveStyle::with_fill(Rgb565::RED))
                .draw(&mut lcd)
                .unwrap();
        } else {
            Rectangle::with_corners(Point::new(100, 0), Point::new(lcd_width, 18))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut lcd)
                .unwrap();
        }

        Line::new(Point::new(2, 20), Point::new(lcd_width - 2, 20))
            .into_styled(PrimitiveStyle::with_stroke(Rgb565::WHITE, 1))
            .draw(&mut lcd)
            .unwrap();

        if marker_changed {
            Rectangle::with_corners(Point::new(0, 21), Point::new(lcd_width, lcd_height))
                .into_styled(PrimitiveStyle::with_fill(Rgb565::BLACK))
                .draw(&mut lcd)
                .unwrap();
            marker_changed = false;
        }

        if marker {
            Text::with_alignment(
                "MARKER",
                Point::new(lcd_width / 2, (lcd_height / 3) * 2),
                lcd_text_style_big,
                Alignment::Center,
            )
            .draw(&mut lcd)
            .unwrap();
        } else {
            Image::new(&raw_image, Point::new(10, 45 - faces::HEIGHT as i32 / 2))
                .draw(&mut lcd)
                .unwrap();

            let mut sdcard_used_percentage = sdcard_used_size as f32 / sdcard_max_size as f32;
            if sdcard_used_percentage < 0.01 {
                sdcard_used_percentage = 0.36
            }

            let sdcard_usage_arc_sweep = sdcard_used_percentage * 360.0;

            lcd_buffer.clear();
            write!(&mut lcd_buffer, "{:0>2.0}%", sdcard_used_percentage * 100.).unwrap();

            Text::with_text_style(
                lcd_buffer.as_str(),
                Point::new(130, 50),
                lcd_text_style_white,
                TextStyleBuilder::new()
                    .alignment(Alignment::Center)
                    .baseline(Baseline::Middle)
                    .build(),
            )
            .draw(&mut lcd)
            .unwrap();

            Arc::new(
                Point::new(130, 50).sub(Point::new(20, 20)),
                40,
                (90.0 + sdcard_usage_arc_sweep).deg(),
                (360.0 - sdcard_usage_arc_sweep).deg(),
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
                sdcard_usage_arc_sweep.deg(),
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
        }

        delay.delay_ms(100_u32);
    }
}

fn buzzer_startup() {
    riscv::interrupt::free(|cs| {
        if let (Some(buzzer_tone_timer), buzzer_index) = (
            BUZZER_TONE_TIMER.borrow(cs).borrow_mut().as_mut(),
            BUZZER_INDEX.borrow(cs).borrow_mut().deref_mut(),
        ) {
            *buzzer_index = 0;
            buzzer_tone_timer.listen(timer::Event::Update);
        }
    });
}

#[allow(non_snake_case)]
#[no_mangle]
fn TIMER1() {
    riscv::interrupt::free(|cs| {
        if let (Some(buzzer_timer), Some(buzzer_tone_timer), buzzer_index) = (
            BUZZER_TIMER.borrow(cs).borrow_mut().as_mut(),
            BUZZER_TONE_TIMER.borrow(cs).borrow_mut().as_mut(),
            BUZZER_INDEX.borrow(cs).borrow_mut().deref_mut(),
        ) {
            buzzer_tone_timer.clear_update_interrupt_flag();

            *buzzer_index += 1;
            if *buzzer_index < tones::STARTUP.len() {
                let frequency = tones::STARTUP[*buzzer_index].0 << 2;
                if frequency == 0 {
                    buzzer_timer.disable(BUZZER_CHANNEL);
                } else {
                    buzzer_timer.enable(BUZZER_CHANNEL);
                    buzzer_timer.set_period(frequency.hz());
                }

                let frequency = (1000. / (tones::STARTUP[*buzzer_index].1) as f32) as u32;
                if frequency > 0 {
                    buzzer_tone_timer.start(frequency.hz());
                } else {
                    buzzer_tone_timer.start(1.hz());
                }
            } else {
                *buzzer_index = 0;
                buzzer_tone_timer.unlisten(timer::Event::Update);
            }
        }
    });
}

// can interrupts

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

#[inline(never)]
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    sprintln!("{}", info);
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

/// Handle all unhandled interrupts
#[allow(non_snake_case)]
#[no_mangle]
fn DefaultHandler() {
    let code = riscv::register::mcause::read().code() & 0xFFF;
    let cause = riscv::register::mcause::Exception::from(code);
    panic!("default handler: code={}, cause={:?}", code, cause);
}
