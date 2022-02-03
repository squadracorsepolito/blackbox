use gd32vf103xx_hal::{rcu::Clocks};

pub struct Timestamp {
    starting_time: u64,
    core_frequency: u64,
}

impl Timestamp {
    pub fn new(starting_time: u32, clocks: &Clocks) -> Self {
        Self {
            starting_time: starting_time as u64,
            core_frequency: clocks.sysclk().0 as u64,
        }
    }

    pub fn tick_us(&self) -> u64 {
        self.starting_time  * 1_000_000 + (riscv::register::mcycle::read64() * 1_000_000) / self.core_frequency
    }

    pub fn tick_ms(&self) -> u64 {
        self.starting_time  * 1_000 + (riscv::register::mcycle::read64() * 1_000) / self.core_frequency
    }
}
