A simple Rust driver for the VL53l0X TOF ranging sensor.

This crate is intentionally basic and does not implement all the features of the VL53l0X. It is designed to be a simple way to get distance readings from the sensor. PRs are welcome if you would like to add more features.

The overall architecture of this crate is based on the original C code and the [Polulu Arduino library](https://github.com/pololu/vl53l0x-arduino), but the interface has been greatly simplified and Rust-ified.

## Usage

```rust
let mut tof = Vl53l0x::new(i2c, x_shut, 0x31, &clock_source).unwrap();

loop {
    if let Some(reading) = tof.try_read().unwrap() {
        println!("Distance: {}mm", reading);
    }
}
```
