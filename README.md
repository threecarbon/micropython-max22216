# micropython-max22216

MicroPython driver for the MAX22216/MAX22217 quad smart serial-controlled solenoid and motor driver.

Single file. No dependencies. MicroPython, not CircuitPython.

[Datasheet (PDF)](https://www.analog.com/media/en/technical-documentation/data-sheets/max22216-max22217.pdf)

## Features

### Drive Control
- 4-channel independent half-bridge control (36V, 3.2A MAX22216 / 1A MAX22217)
- Two-level sequencer: excitation (hit) and hold duty cycles with configurable timing
- VDR (voltage drive regulation) with supply-compensated output
- CDR (current drive regulation) with PI controller configuration
- Configurable hardware topology (CHS): independent, parallel, and full-bridge combinations
- Per-channel PWM frequency (2.5–100 kHz master with per-channel dividers)
- Configurable slew rate, blanking time, GAIN, and SNSF

### Diagnostics
- Detection of Plunger Movement (DPM): peak/valley current, travel time, reaction time
- Inductance measurement via AC sine wave injection
- Resistance measurement
- Real-time current monitoring (I_MONITOR)
- PWM duty cycle monitoring
- Supply voltage monitoring (VM)
- Full diagnostic readout with unit-scaled values (mA, mH, ohms, ms, %)

### Protection and Faults
- Overcurrent protection (OCP)
- Overtemperature protection (OVT)
- Undervoltage lockout (UVM)
- Open-load detection during channel off (OLF)
- Hit current not reached detection (HHF)
- Communication error detection (COMER)
- Per-channel fault logging with write-1-to-clear registers
- Configurable fault masking

### Driver Implementation
- SPI register read/write with optional CRC-5 validation (table-driven, one lookup per byte)
- Diagnostics as integers in a per-channel dict filled in place, so a periodic read allocates nothing; a live-only read of the three registers that move while a channel is driven
- Configuration the diagnostics depend on (SNSF/GAIN, PWM frequency, AC scan) cached until the register is written
- Property accessors for all register bitfields (generated at import time)
- Register dump and `test()` diagnostic summary
- MAX22216 and MAX22217 variant support (automatic current scaling)
- Every setter checks its argument against the register field and raises `ValueError`; a value is never masked into a different one

## Not Yet Implemented

- **Full-bridge drive methods** — CHS configurations 0x5–0x8 can be selected but no forward/reverse/brake helpers
- **DC motor control** — CTRL_MODE 2 (current limiter/VDR) and mode 3 (VDR/CDR) are selectable but no motor-specific helpers
- **Ramp control** — RUPE/RMDE/RDWE and RAMP value settable via `config_ctrl0()` but no high-level ramp profile API
- **Dither** — DITH_EN accessible via bitfield but no convenience methods
- **DC_H2L** — Hold-to-low transition duty cycle accessible via bitfield but not wired into `config_channel()`
- **Status pin configuration** — STAT_FUN, STAT_SEL, STRETCH_EN accessible via bitfields but no convenience methods
- **VM threshold alarms** — VM_THLD_UP/VM_THLD_DOWN accessible via bitfields but no convenience methods
- **OTP programming** — One-time programmable register writing not supported

## Usage

### Setup

```python
from machine import SPI, Pin
from max22216 import MAX22216

spi = SPI(1, baudrate=5_000_000, polarity=0, phase=0)   # the device is rated to 10 MHz
cs = Pin(17, Pin.OUT, value=1)
enable = Pin(16, Pin.OUT, value=0)

drv = MAX22216(spi, cs, enable)
drv.enable()
```

### Activation

The device must be activated before any channels can fire:

```python
# Activate with VDR mode, 4x independent half-bridges (default)
drv.activate(vdr=True, chs=0, clear_flags=True)

# Or with a fault mask to suppress known-benign faults
drv.activate(fault_mask=MAX22216.FMASK_OLF | MAX22216.FMASK_DPM)
```

### Channel Configuration

```python
drv.config_ctrl1(0, snsf=0, gain=0, slew_rate=0, f_pwm=0)

drv.config_channel(
    channel=0,
    dc_h=8000,          # hold duty cycle
    dc_l2h=40000,       # hit (pull-in) duty cycle
    time_l2h=5000,      # hit duration (PWM cycles)
    dpm_enable=True,     # plunger movement detection
    l_meas_h=True,       # inductance measurement during hold
    res_thld=2000,       # resistance fault threshold
    iac_thld=500,        # inductance fault threshold
)

# The sine-wave generator for inductance measurement is one register pair per
# device, not per channel
drv.f_ac = 500          # AC frequency (12-bit raw)
drv.u_ac_scan = 1000    # AC amplitude (15-bit raw)
```

Every value is checked before the first write: one outside its field raises `ValueError` and the channel keeps its configuration.

`dpm_enable=False` clears DPM_EN (it persists across configurations otherwise), `l_meas_h` and `l_meas_l2h` are written both ways, and `ramp`, `rupe` and `rdwe` set the ramp controls in CTRL0. `dpm_start` is a current level (about 65 mA per unit at default scaling; see `ma_to_dpm_start()`), not a delay.

### Firing a Channel

```python
drv.set_channel(0, True)    # enable channel 0
drv.set_channel(0, False)   # disable channel 0
drv.all_channels_off()      # clear every CNTL bit, e.g. before (re)activation
```

### Diagnostics

```python
# Full scaled diagnostics: integers, so a periodic read allocates nothing
diag = drv.diagnostics(0)
print(f"Supply: {diag['voltage_mv'] / 1000} V")
print(f"Current: {diag['i_monitor_ma']} mA")
print(f"Resistance: {diag['resistance_mohm'] / 1000} Ω")
print(f"Inductance: {diag['inductance_uh'] / 1000} mH")
print(f"PWM duty: {diag['pwm_duty_permille'] / 10} %")
# also: peak_ma, valley_ma, travel_us, reaction_us, i_ac_ma, and the raw
# counts travel_raw, reaction_raw (PWM periods) and pwm_duty_raw.
# A field is None, never 0, when there is no measurement behind it: the
# register held the mid-conversion value (0xFF00 and above), the PWM
# frequency or current scale it needs is zero, or (resistance) the chip has
# not measured since the last inductance measurement. i_monitor_ma is signed.

# The dict is one object per channel, refilled in place: copy it to keep a snapshot
snapshot = dict(diag)

# While a channel is held, only the monitor current, AC current and PWM duty
# move; read just those (three register reads) and keep the DPM, travel,
# reaction and resistance values from the last full read
diag = drv.diagnostics(0, live_only=True)

# Pass a supply voltage you already have to save the VM read
diag = drv.diagnostics(0, vm=drv.vm)

# DPM diagnostics (peak_mA, valley_mA, travel_ms, reaction_ms)
peak, valley, travel, react = drv.dpm(0)

# Raw diagnostic registers
raw = drv.diagnostics_raw(0)

# Motor voltage
print(f"VM: {drv.vm:.2f}V")

# Conversions between register codes and units, using the channel's PWM
# frequency and current scale
ms = drv.pwm_counts_to_ms(diag["travel_raw"], 0)
drv.time_l2h_0 = drv.ms_to_pwm_counts(50, 0)
drv.iac_thld_0 = drv.ma_to_iac_thld(120, 0)
print(f"master PWM: {drv.master_pwm_khz()} kHz")

# The global and per-channel configuration registers (0x00 to 0x40) as
# consecutive little-endian 16-bit words, e.g. to compare against a known file
words = drv.register_file()
```

### Faults

```python
# Read combined fault register (32-bit: fault1 << 16 | fault0)
print(f"Faults: 0x{drv.fault:08X}")

# Clear all faults. The status byte that comes back with a frame describes
# the state before the frame took effect, so a COMER reported by the clear
# itself is ignored; a persistent fault shows up on the next read.
drv.fault_clear()

# Clear one channel's latched faults only (OCP, HHF, OLF, DPM in FAULT0 and
# IND, RES in FAULT1); the other channels' bits and the device-wide UVM,
# COMER and OVT stay latched
drv.fault_clear_channel(0)

# Live status register (not latched)
print(f"status: 0x{drv.status:04X}")

# Read individual fault bits via bitfield accessors
if drv.ocp0_fault:
    print("Channel 0 over-current!")

# Apply fault mask (suppress faults from triggering FAULT pin)
drv.apply_fault_mask(MAX22216.FMASK_OLF | MAX22216.FMASK_DPM)
```

### Bitfield Accessors

All register bitfields are available as properties on the instance:

```python
# Global config
drv.active = 1
drv.vdr_nduty = 1
drv.chs = 0
drv.f_pwm_m = 3        # master PWM frequency code (0-12, Table 4)

# Per-channel (suffixed with _0 through _3)
drv.dc_h_0 = 8000
drv.dc_l2h_0 = 40000
drv.ctrl_mode_0 = MAX22216.CTRL_MODE_VDR_VDR
drv.dpm_en_0 = 1
drv.l_meas_en_0 = 1

# Read-only diagnostics
print(drv.i_monitor_0)
print(drv.i_ac_0)
print(drv.res_0)
```

### Register Dump

```python
drv.dump_registers()
# --- Global Registers ---
#   0x00 GLOBAL_CTRL   = 0x0000
#   0x01 GLOBAL_CFG    = 0x8010
#   ...
# --- Channel 0 Config ---
#   0x09 DC_L2H        = 0x9C40
#   ...
```

### Diagnostic Summary

```python
drv.test()
# MAX22216
#   VM: 23.45V
#   status: 0x0000  active: True
#   global_ctrl: 0x0001  global_cfg: 0x8010
#   faults: none
#   ch0: VDR/VDR  on=True  dpm=True  scale=1.000  pwm=100kHz
#   ch1: VDR/VDR  on=False  dpm=False  scale=1.000  pwm=100kHz
#   ...
```

### CRC Mode

```python
drv = MAX22216(spi, cs, enable, crc_enable=True)
# All SPI transactions now include CRC-5 validation
# Raises MAX22216CRCError (a MAX22216SPIError) on mismatch, so one handler
# covers communication errors and corrupted replies alike
# The read command frames are built once per register; MAX22216.reads counts
# register reads across all instances for profiling (reset it yourself)
```

### MAX22217 Variant

```python
drv = MAX22216(spi, cs, enable, device_variant="MAX22217")
# Uses MAX22217 current scaling constants (lower current range)
```

## Pin Connections

| Function | Description |
|----------|-------------|
| SPI MOSI/MISO/SCK | Standard SPI bus |
| CS | Chip select (active low) |
| ENABLE | Device enable (active high, 0.8ms startup) |
| FAULT | Open-drain fault output (optional) |
| CNTL0–CNTL3 | Hardware channel control pins (optional) |
| STAT0–STAT1 | Status output pins (optional) |

## License

MIT License. See [LICENSE](LICENSE) for details.
