# `blackbox`

## Getting started

Follow the documentation in the [longan-nano](./longan-nano/readme.md) crate.

### Quickstart

On a terminal window, start OpenOCD:

```
riscv-openocd -f sipeed-jtag.cfg -f openocd.cfg
```

On another terminal window, run cankong in a gdb session:

```
cargo run -p blackbox --release
```

The software should start executing on the longan-nano.

If you UART output, install `minicom` and, on another terminal window:

```
minicom -D /dev/ttyUSB1 -b 9600
```
