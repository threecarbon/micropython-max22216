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
- SPI register read/write with optional CRC-5 validation
- Property accessors for all register bitfields (generated at import time)
- Register dump and `test()` diagnostic summary
- MAX22216 and MAX22217 variant support (automatic current scaling)

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

spi = SPI(1, baudrate=250_000, polarity=0, phase=0)
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
    f_ac=500,            # AC frequency for inductance measurement
    u_ac=1000,           # AC amplitude
    res_thld=2000,       # resistance fault threshold
    iac_thld=500,        # inductance fault threshold
)
```

### Firing a Channel

```python
drv.set_channel(0, True)    # enable channel 0
drv.set_channel(0, False)   # disable channel 0
```

### Diagnostics

```python
# Full scaled diagnostics
diag = drv.diagnostics(0)
print(f"Current: {diag['i_monitor_ma']}mA")
print(f"Resistance: {diag['resistance_ohm']}Ω")
print(f"Inductance: {diag['inductance_mh']}mH")
print(f"PWM duty: {diag['pwm_duty']}%")

# DPM diagnostics (peak_mA, valley_mA, travel_ms, reaction_ms)
peak, valley, travel, react = drv.dpm(0)

# Raw diagnostic registers
raw = drv.diagnostics_raw(0)

# Motor voltage
print(f"VM: {drv.vm:.2f}V")
```

### Faults

```python
# Read combined fault register (32-bit: fault1 << 16 | fault0)
print(f"Faults: 0x{drv.fault:08X}")

# Clear all faults
drv.fault_clear()

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
# Raises MAX22216CRCError on mismatch
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
