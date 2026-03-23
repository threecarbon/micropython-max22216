"""
MicroPython MAX22216/MAX22217
Quad Smart Serial-Controlled Solenoid and Motor Driver with Advanced Diagnostic

https://www.analog.com/media/en/technical-documentation/data-sheets/max22216-max22217.pdf

MIT License

Copyright (c) 2025-2026 Seth Hardy

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

import time

from machine import Pin, SPI, SoftSPI
from micropython import const


__version__ = "0.1.0"


# global registers
_REG_GLOBAL_CTRL   = const(0x00)
_REG_GLOBAL_CFG    = const(0x01)
_REG_STATUS        = const(0x02)
_REG_STATUS_CFG    = const(0x03)
_REG_DC_H2L        = const(0x04)
_REG_VM_MONITOR    = const(0x05)
_REG_VM_THRESHOLD  = const(0x06)
_REG_F_AC          = const(0x07)
_REG_U_AC_SCAN     = const(0x08)

# configuration registers channel 0
_REG_CFG0_DC_L2H   = const(0x09)
_REG_CFG0_DC_H     = const(0x0A)
_REG_CFG0_DC_L     = const(0x0B)
_REG_CFG0_L2H_TIME = const(0x0C)
_REG_CFG0_CTRL0    = const(0x0D)
_REG_CFG0_CTRL1    = const(0x0E)
_REG_CFG0_DPM0     = const(0x0F)
_REG_CFG0_DPM1     = const(0x10)
_REG_CFG0_IDC_THLD = const(0x11)
_REG_CFG0_R_THLD   = const(0x12)
_REG_CFG0_IND_0    = const(0x13)
_REG_CFG0_IND_1    = const(0x14)
_REG_CFG0_P        = const(0x15)
_REG_CFG0_I        = const(0x16)

# configuration registers channel 1
_REG_CFG1_DC_L2H   = const(0x17)
_REG_CFG1_DC_H     = const(0x18)
_REG_CFG1_DC_L     = const(0x19)
_REG_CFG1_L2H_TIME = const(0x1A)
_REG_CFG1_CTRL0    = const(0x1B)
_REG_CFG1_CTRL1    = const(0x1C)
_REG_CFG1_DPM0     = const(0x1D)
_REG_CFG1_DPM1     = const(0x1E)
_REG_CFG1_IDC_THLD = const(0x1F)
_REG_CFG1_R_THLD   = const(0x20)
_REG_CFG1_IND_0    = const(0x21)
_REG_CFG1_IND_1    = const(0x22)
_REG_CFG1_P        = const(0x23)
_REG_CFG1_I        = const(0x24)

# configuration registers channel 2
_REG_CFG2_DC_L2H   = const(0x25)
_REG_CFG2_DC_H     = const(0x26)
_REG_CFG2_DC_L     = const(0x27)
_REG_CFG2_L2H_TIME = const(0x28)
_REG_CFG2_CTRL0    = const(0x29)
_REG_CFG2_CTRL1    = const(0x2A)
_REG_CFG2_DPM0     = const(0x2B)
_REG_CFG2_DPM1     = const(0x2C)
_REG_CFG2_IDC_THLD = const(0x2D)
_REG_CFG2_R_THLD   = const(0x2E)
_REG_CFG2_IND_0    = const(0x2F)
_REG_CFG2_IND_1    = const(0x30)
_REG_CFG2_P        = const(0x31)
_REG_CFG2_I        = const(0x32)

# configuration registers channel 3
_REG_CFG3_DC_L2H   = const(0x33)
_REG_CFG3_DC_H     = const(0x34)
_REG_CFG3_DC_L     = const(0x35)
_REG_CFG3_L2H_TIME = const(0x36)
_REG_CFG3_CTRL0    = const(0x37)
_REG_CFG3_CTRL1    = const(0x38)
_REG_CFG3_DPM0     = const(0x39)
_REG_CFG3_DPM1     = const(0x3A)
_REG_CFG3_IDC_THLD = const(0x3B)
_REG_CFG3_R_THLD   = const(0x3C)
_REG_CFG3_IND_0    = const(0x3D)
_REG_CFG3_IND_1    = const(0x3E)
_REG_CFG3_P        = const(0x3F)
_REG_CFG3_I        = const(0x40)

# diagnostics channel 0
_REG_DIA0_I_DPM_PEAK    = const(0x41)
_REG_DIA0_I_DPM_VALLEY  = const(0x42)
_REG_DIA0_TRAVEL_TIME   = const(0x43)
_REG_DIA0_REACTION_TIME = const(0x44)
_REG_DIA0_I_MONITOR     = const(0x45)
# UNUSED                = const(0x46)
_REG_DIA0_I_AC          = const(0x47)
_REG_DIA0_RES           = const(0x48)
_REG_DIA0_PWM_DUTY      = const(0x49)

# diagnostics channel 1
_REG_DIA1_I_DPM_PEAK    = const(0x4A)
_REG_DIA1_I_DPM_VALLEY  = const(0x4B)
_REG_DIA1_TRAVEL_TIME   = const(0x4C)
_REG_DIA1_REACTION_TIME = const(0x4D)
_REG_DIA1_I_MONITOR     = const(0x4E)
# UNUSED                = const(0x4F)
_REG_DIA1_I_AC          = const(0x50)
_REG_DIA1_RES           = const(0x51)
_REG_DIA1_PWM_DUTY      = const(0x52)

# diagnostics channel 2
_REG_DIA2_I_DPM_PEAK    = const(0x53)
_REG_DIA2_I_DPM_VALLEY  = const(0x54)
_REG_DIA2_TRAVEL_TIME   = const(0x55)
_REG_DIA2_REACTION_TIME = const(0x56)
_REG_DIA2_I_MONITOR     = const(0x57)
# UNUSED                = const(0x58)
_REG_DIA2_I_AC          = const(0x59)
_REG_DIA2_RES           = const(0x5A)
_REG_DIA2_PWM_DUTY      = const(0x5B)

# diagnostics channel 3
_REG_DIA3_I_DPM_PEAK    = const(0x5C)
_REG_DIA3_I_DPM_VALLEY  = const(0x5D)
_REG_DIA3_TRAVEL_TIME   = const(0x5E)
_REG_DIA3_REACTION_TIME = const(0x5F)
_REG_DIA3_I_MONITOR     = const(0x60)
# UNUSED                = const(0x61)
_REG_DIA3_I_AC          = const(0x62)
_REG_DIA3_RES           = const(0x63)
_REG_DIA3_PWM_DUTY      = const(0x64)

# fault log
_REG_FAULT0            = const(0x65)
_REG_FAULT1            = const(0x66)

# per-channel register strides
_CH_STRIDE             = const(14)  # config registers
_DIAG_STRIDE           = const(9)   # diagnostic registers


# bitfields: (register, mask, shift, attributes)
_BITFIELDS = {
    "CNTL0": (_REG_GLOBAL_CTRL, 0x0001, 0, "rw"),
    "CNTL1": (_REG_GLOBAL_CTRL, 0x0002, 1, "rw"),
    "CNTL2": (_REG_GLOBAL_CTRL, 0x0004, 2, "rw"),
    "CNTL3": (_REG_GLOBAL_CTRL, 0x0008, 3, "rw"),
    "F_PWM_M": (_REG_GLOBAL_CTRL, 0x00F0, 4, "rw"),
    "CHS": (_REG_GLOBAL_CFG, 0x000F, 0, "rw"),
    "VDR_NDUTY": (_REG_GLOBAL_CFG, 0x0010, 4, "rw"),
    "STAT_POL": (_REG_GLOBAL_CFG, 0x0040, 6, "rw"),
    "CNTL_POL": (_REG_GLOBAL_CFG, 0x0080, 7, "rw"),
    "M_UVM": (_REG_GLOBAL_CFG, 0x0100, 8, "rw"),
    "M_COMF": (_REG_GLOBAL_CFG, 0x0200, 9, "rw"),
    "M_DPM": (_REG_GLOBAL_CFG, 0x0400, 10, "rw"),
    "M_HHF": (_REG_GLOBAL_CFG, 0x0800, 11, "rw"),
    "M_OLF": (_REG_GLOBAL_CFG, 0x1000, 12, "rw"),
    "M_OCP": (_REG_GLOBAL_CFG, 0x2000, 13, "rw"),
    "M_OVT": (_REG_GLOBAL_CFG, 0x4000, 14, "rw"),
    "ACTIVE": (_REG_GLOBAL_CFG, 0x8000, 15, "rw"),
    "RFU": (_REG_STATUS, 0x0001, 0, "r"),
    "UVM": (_REG_STATUS, 0x0002, 1, "r"),
    "COMER": (_REG_STATUS, 0x0004, 2, "r"),
    "DPM": (_REG_STATUS, 0x0008, 3, "r"),
    "HHF": (_REG_STATUS, 0x0010, 4, "r"),
    "OLF": (_REG_STATUS, 0x0020, 5, "r"),
    "OCP": (_REG_STATUS, 0x0040, 6, "r"),
    "OVT": (_REG_STATUS, 0x0080, 7, "r"),
    "IND": (_REG_STATUS, 0x0100, 8, "r"),
    "RES": (_REG_STATUS, 0x0200, 9, "r"),
    "MIN_T_ON": (_REG_STATUS, 0x0400, 10, "r"),
    "STAT0": (_REG_STATUS, 0x0800, 11, "r"),
    "STAT1": (_REG_STATUS, 0x1000, 12, "r"),
    "STAT2": (_REG_STATUS, 0x2000, 13, "r"),
    "STAT3": (_REG_STATUS, 0x4000, 14, "r"),
    "STAT_FUN": (_REG_STATUS_CFG, 0x0007, 0, "rw"),
    "STAT_SEL0": (_REG_STATUS_CFG, 0x0008, 3, "rw"),
    "STAT_SEL1": (_REG_STATUS_CFG, 0x0010, 4, "rw"),
    "STRETCH_EN": (_REG_STATUS_CFG, 0x0060, 5, "rw"),
    "EN_LDO": (_REG_STATUS_CFG, 0x0080, 7, "r"),
    "V5_NV3": (_REG_STATUS_CFG, 0x0100, 8, "r"),
    "M_UVM_CMP": (_REG_STATUS_CFG, 0x0200, 9, "rw"),
    "DC_H2L": (_REG_DC_H2L, 0xFFFF, 0, "rw"),
    "ADC_VM_RAW": (_REG_VM_MONITOR, 0x1FFF, 0, "r"),
    "VM_THLD_DOWN": (_REG_VM_THRESHOLD, 0x000F, 0, "rw"),
    "VM_THLD_UP": (_REG_VM_THRESHOLD, 0x00F0, 4, "rw"),
    "F_AC_SCAN": (_REG_F_AC, 0x0FFF, 0, "rw"),
    "U_AC_SCAN": (_REG_U_AC_SCAN, 0x7FFF, 0, "rw"),
    "DC_L2H_0": (_REG_CFG0_DC_L2H, 0xFFFF, 0, "rw"),
    "DC_H_0": (_REG_CFG0_DC_H, 0xFFFF, 0, "rw"),
    "DC_L_0": (_REG_CFG0_DC_L, 0xFFFF, 0, "rw"),
    "TIME_L2H_0": (_REG_CFG0_L2H_TIME, 0xFFFF, 0, "rw"),
    "RAMP_0": (_REG_CFG0_CTRL0, 0x00FF, 0, "rw"),
    "RUPE_0": (_REG_CFG0_CTRL0, 0x0100, 8, "rw"),
    "RMDE_0": (_REG_CFG0_CTRL0, 0x0200, 9, "rw"),
    "RDWE_0": (_REG_CFG0_CTRL0, 0x0400, 10, "rw"),
    "H2L_EN_0": (_REG_CFG0_CTRL0, 0x0800, 11, "rw"),
    "OL_EN_0": (_REG_CFG0_CTRL0, 0x1000, 12, "rw"),
    "HHF_EN_0": (_REG_CFG0_CTRL0, 0x2000, 13, "rw"),
    "CTRL_MODE_0": (_REG_CFG0_CTRL0, 0xC000, 14, "rw"),
    "SNSF_0": (_REG_CFG0_CTRL1, 0x0003, 0, "rw"),
    "GAIN_0": (_REG_CFG0_CTRL1, 0x000C, 2, "rw"),
    "SLEW_RATE_0": (_REG_CFG0_CTRL1, 0x0030, 4, "rw"),
    "T_BLANKING_0": (_REG_CFG0_CTRL1, 0x00C0, 6, "rw"),
    "F_PWM_0": (_REG_CFG0_CTRL1, 0x0300, 8, "rw"),
    "HSNLS_0": (_REG_CFG0_CTRL1, 0x0400, 10, "rw"),
    "DPM_THLD_0": (_REG_CFG0_DPM0, 0x0FFF, 0, "rw"),
    "DPM_START_0": (_REG_CFG0_DPM1, 0x00FF, 0, "rw"),
    "DPM_MIN_NBR_0": (_REG_CFG0_DPM1, 0x0F00, 8, "rw"),
    "END_HIT_AUTO_0": (_REG_CFG0_DPM1, 0x1000, 12, "rw"),
    "END_HIT_TO_HIZ_AUTO_0": (_REG_CFG0_DPM1, 0x2000, 13, "rw"),
    "DPM_EN_0": (_REG_CFG0_DPM1, 0x4000, 14, "rw"),
    "IDC_THLD_0": (_REG_CFG0_IDC_THLD, 0xFFFF, 0, "rw"),
    "RES_THLD_0": (_REG_CFG0_R_THLD, 0xFFFF, 0, "rw"),
    "L_NBR_CALC_0": (_REG_CFG0_IND_0, 0x000F, 0, "rw"),
    "L_MEAS_WCYCLES_0": (_REG_CFG0_IND_0, 0x00F0, 4, "rw"),
    "L_MEAS_H_0": (_REG_CFG0_IND_0, 0x0100, 8, "rw"),
    "L_MEAS_L2H_0": (_REG_CFG0_IND_0, 0x0200, 9, "rw"),
    "L_MEAS_EN_0": (_REG_CFG0_IND_0, 0x0400, 10, "rw"),
    "DITH_EN_0": (_REG_CFG0_IND_0, 0x0800, 11, "rw"),
    "IAC_THLD_0": (_REG_CFG0_IND_1, 0x0FFF, 0, "rw"),
    "CFG_P_0": (_REG_CFG0_P, 0xFFFF, 0, "rw"),
    "CFG_I_0": (_REG_CFG0_I, 0xFFFF, 0, "rw"),
    "DC_L2H_1": (_REG_CFG1_DC_L2H, 0xFFFF, 0, "rw"),
    "DC_H_1": (_REG_CFG1_DC_H, 0xFFFF, 0, "rw"),
    "DC_L_1": (_REG_CFG1_DC_L, 0xFFFF, 0, "rw"),
    "TIME_L2H_1": (_REG_CFG1_L2H_TIME, 0xFFFF, 0, "rw"),
    "RAMP_1": (_REG_CFG1_CTRL0, 0x00FF, 0, "rw"),
    "RUPE_1": (_REG_CFG1_CTRL0, 0x0100, 8, "rw"),
    "RMDE_1": (_REG_CFG1_CTRL0, 0x0200, 9, "rw"),
    "RDWE_1": (_REG_CFG1_CTRL0, 0x0400, 10, "rw"),
    "H2L_EN_1": (_REG_CFG1_CTRL0, 0x0800, 11, "rw"),
    "OL_EN_1": (_REG_CFG1_CTRL0, 0x1000, 12, "rw"),
    "HHF_EN_1": (_REG_CFG1_CTRL0, 0x2000, 13, "rw"),
    "CTRL_MODE_1": (_REG_CFG1_CTRL0, 0xC000, 14, "rw"),
    "SNSF_1": (_REG_CFG1_CTRL1, 0x0003, 0, "rw"),
    "GAIN_1": (_REG_CFG1_CTRL1, 0x000C, 2, "rw"),
    "SLEW_RATE_1": (_REG_CFG1_CTRL1, 0x0030, 4, "rw"),
    "T_BLANKING_1": (_REG_CFG1_CTRL1, 0x00C0, 6, "rw"),
    "F_PWM_1": (_REG_CFG1_CTRL1, 0x0300, 8, "rw"),
    "HSNLS_1": (_REG_CFG1_CTRL1, 0x0400, 10, "rw"),
    "DPM_THLD_1": (_REG_CFG1_DPM0, 0x0FFF, 0, "rw"),
    "DPM_START_1": (_REG_CFG1_DPM1, 0x00FF, 0, "rw"),
    "DPM_MIN_NBR_1": (_REG_CFG1_DPM1, 0x0F00, 8, "rw"),
    "END_HIT_AUTO_1": (_REG_CFG1_DPM1, 0x1000, 12, "rw"),
    "END_HIT_TO_HIZ_AUTO_1": (_REG_CFG1_DPM1, 0x2000, 13, "rw"),
    "DPM_EN_1": (_REG_CFG1_DPM1, 0x4000, 14, "rw"),
    "IDC_THLD_1": (_REG_CFG1_IDC_THLD, 0xFFFF, 0, "rw"),
    "RES_THLD_1": (_REG_CFG1_R_THLD, 0xFFFF, 0, "rw"),
    "L_NBR_CALC_1": (_REG_CFG1_IND_0, 0x000F, 0, "rw"),
    "L_MEAS_WCYCLES_1": (_REG_CFG1_IND_0, 0x00F0, 4, "rw"),
    "L_MEAS_H_1": (_REG_CFG1_IND_0, 0x0100, 8, "rw"),
    "L_MEAS_L2H_1": (_REG_CFG1_IND_0, 0x0200, 9, "rw"),
    "L_MEAS_EN_1": (_REG_CFG1_IND_0, 0x0400, 10, "rw"),
    "DITH_EN_1": (_REG_CFG1_IND_0, 0x0800, 11, "rw"),
    "IAC_THLD_1": (_REG_CFG1_IND_1, 0x0FFF, 0, "rw"),
    "CFG_P_1": (_REG_CFG1_P, 0xFFFF, 0, "rw"),
    "CFG_I_1": (_REG_CFG1_I, 0xFFFF, 0, "rw"),
    "DC_L2H_2": (_REG_CFG2_DC_L2H, 0xFFFF, 0, "rw"),
    "DC_H_2": (_REG_CFG2_DC_H, 0xFFFF, 0, "rw"),
    "DC_L_2": (_REG_CFG2_DC_L, 0xFFFF, 0, "rw"),
    "TIME_L2H_2": (_REG_CFG2_L2H_TIME, 0xFFFF, 0, "rw"),
    "RAMP_2": (_REG_CFG2_CTRL0, 0x00FF, 0, "rw"),
    "RUPE_2": (_REG_CFG2_CTRL0, 0x0100, 8, "rw"),
    "RMDE_2": (_REG_CFG2_CTRL0, 0x0200, 9, "rw"),
    "RDWE_2": (_REG_CFG2_CTRL0, 0x0400, 10, "rw"),
    "H2L_EN_2": (_REG_CFG2_CTRL0, 0x0800, 11, "rw"),
    "OL_EN_2": (_REG_CFG2_CTRL0, 0x1000, 12, "rw"),
    "HHF_EN_2": (_REG_CFG2_CTRL0, 0x2000, 13, "rw"),
    "CTRL_MODE_2": (_REG_CFG2_CTRL0, 0xC000, 14, "rw"),
    "SNSF_2": (_REG_CFG2_CTRL1, 0x0003, 0, "rw"),
    "GAIN_2": (_REG_CFG2_CTRL1, 0x000C, 2, "rw"),
    "SLEW_RATE_2": (_REG_CFG2_CTRL1, 0x0030, 4, "rw"),
    "T_BLANKING_2": (_REG_CFG2_CTRL1, 0x00C0, 6, "rw"),
    "F_PWM_2": (_REG_CFG2_CTRL1, 0x0300, 8, "rw"),
    "HSNLS_2": (_REG_CFG2_CTRL1, 0x0400, 10, "rw"),
    "DPM_THLD_2": (_REG_CFG2_DPM0, 0x0FFF, 0, "rw"),
    "DPM_START_2": (_REG_CFG2_DPM1, 0x00FF, 0, "rw"),
    "DPM_MIN_NBR_2": (_REG_CFG2_DPM1, 0x0F00, 8, "rw"),
    "END_HIT_AUTO_2": (_REG_CFG2_DPM1, 0x1000, 12, "rw"),
    "END_HIT_TO_HIZ_AUTO_2": (_REG_CFG2_DPM1, 0x2000, 13, "rw"),
    "DPM_EN_2": (_REG_CFG2_DPM1, 0x4000, 14, "rw"),
    "IDC_THLD_2": (_REG_CFG2_IDC_THLD, 0xFFFF, 0, "rw"),
    "RES_THLD_2": (_REG_CFG2_R_THLD, 0xFFFF, 0, "rw"),
    "L_NBR_CALC_2": (_REG_CFG2_IND_0, 0x000F, 0, "rw"),
    "L_MEAS_WCYCLES_2": (_REG_CFG2_IND_0, 0x00F0, 4, "rw"),
    "L_MEAS_H_2": (_REG_CFG2_IND_0, 0x0100, 8, "rw"),
    "L_MEAS_L2H_2": (_REG_CFG2_IND_0, 0x0200, 9, "rw"),
    "L_MEAS_EN_2": (_REG_CFG2_IND_0, 0x0400, 10, "rw"),
    "DITH_EN_2": (_REG_CFG2_IND_0, 0x0800, 11, "rw"),
    "IAC_THLD_2": (_REG_CFG2_IND_1, 0x0FFF, 0, "rw"),
    "CFG_P_2": (_REG_CFG2_P, 0xFFFF, 0, "rw"),
    "CFG_I_2": (_REG_CFG2_I, 0xFFFF, 0, "rw"),
    "DC_L2H_3": (_REG_CFG3_DC_L2H, 0xFFFF, 0, "rw"),
    "DC_H_3": (_REG_CFG3_DC_H, 0xFFFF, 0, "rw"),
    "DC_L_3": (_REG_CFG3_DC_L, 0xFFFF, 0, "rw"),
    "TIME_L2H_3": (_REG_CFG3_L2H_TIME, 0xFFFF, 0, "rw"),
    "RAMP_3": (_REG_CFG3_CTRL0, 0x00FF, 0, "rw"),
    "RUPE_3": (_REG_CFG3_CTRL0, 0x0100, 8, "rw"),
    "RMDE_3": (_REG_CFG3_CTRL0, 0x0200, 9, "rw"),
    "RDWE_3": (_REG_CFG3_CTRL0, 0x0400, 10, "rw"),
    "H2L_EN_3": (_REG_CFG3_CTRL0, 0x0800, 11, "rw"),
    "OL_EN_3": (_REG_CFG3_CTRL0, 0x1000, 12, "rw"),
    "HHF_EN_3": (_REG_CFG3_CTRL0, 0x2000, 13, "rw"),
    "CTRL_MODE_3": (_REG_CFG3_CTRL0, 0xC000, 14, "rw"),
    "SNSF_3": (_REG_CFG3_CTRL1, 0x0003, 0, "rw"),
    "GAIN_3": (_REG_CFG3_CTRL1, 0x000C, 2, "rw"),
    "SLEW_RATE_3": (_REG_CFG3_CTRL1, 0x0030, 4, "rw"),
    "T_BLANKING_3": (_REG_CFG3_CTRL1, 0x00C0, 6, "rw"),
    "F_PWM_3": (_REG_CFG3_CTRL1, 0x0300, 8, "rw"),
    "HSNLS_3": (_REG_CFG3_CTRL1, 0x0400, 10, "rw"),
    "DPM_THLD_3": (_REG_CFG3_DPM0, 0x0FFF, 0, "rw"),
    "DPM_START_3": (_REG_CFG3_DPM1, 0x00FF, 0, "rw"),
    "DPM_MIN_NBR_3": (_REG_CFG3_DPM1, 0x0F00, 8, "rw"),
    "END_HIT_AUTO_3": (_REG_CFG3_DPM1, 0x1000, 12, "rw"),
    "END_HIT_TO_HIZ_AUTO_3": (_REG_CFG3_DPM1, 0x2000, 13, "rw"),
    "DPM_EN_3": (_REG_CFG3_DPM1, 0x4000, 14, "rw"),
    "IDC_THLD_3": (_REG_CFG3_IDC_THLD, 0xFFFF, 0, "rw"),
    "RES_THLD_3": (_REG_CFG3_R_THLD, 0xFFFF, 0, "rw"),
    "L_NBR_CALC_3": (_REG_CFG3_IND_0, 0x000F, 0, "rw"),
    "L_MEAS_WCYCLES_3": (_REG_CFG3_IND_0, 0x00F0, 4, "rw"),
    "L_MEAS_H_3": (_REG_CFG3_IND_0, 0x0100, 8, "rw"),
    "L_MEAS_L2H_3": (_REG_CFG3_IND_0, 0x0200, 9, "rw"),
    "L_MEAS_EN_3": (_REG_CFG3_IND_0, 0x0400, 10, "rw"),
    "DITH_EN_3": (_REG_CFG3_IND_0, 0x0800, 11, "rw"),
    "IAC_THLD_3": (_REG_CFG3_IND_1, 0x0FFF, 0, "rw"),
    "CFG_P_3": (_REG_CFG3_P, 0xFFFF, 0, "rw"),
    "CFG_I_3": (_REG_CFG3_I, 0xFFFF, 0, "rw"),
    "I_DPM_PEAK_0": (_REG_DIA0_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_0": (_REG_DIA0_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_0": (_REG_DIA0_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_0": (_REG_DIA0_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_0": (_REG_DIA0_I_MONITOR, 0xFFFF, 0, "r"),
    # "I_DC_0": (0x46, 0xFFFF, 0, "r"),
    "I_AC_0": (_REG_DIA0_I_AC, 0xFFFF, 0, "r"),
    "RES_0": (_REG_DIA0_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_0": (_REG_DIA0_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_1": (_REG_DIA1_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_1": (_REG_DIA1_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_1": (_REG_DIA1_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_1": (_REG_DIA1_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_1": (_REG_DIA1_I_MONITOR, 0xFFFF, 0, "r"),
    # "I_DC_1": (0x4F, 0xFFFF, 0, "r"),
    "I_AC_1": (_REG_DIA1_I_AC, 0xFFFF, 0, "r"),
    "RES_1": (_REG_DIA1_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_1": (_REG_DIA1_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_2": (_REG_DIA2_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_2": (_REG_DIA2_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_2": (_REG_DIA2_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_2": (_REG_DIA2_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_2": (_REG_DIA2_I_MONITOR, 0xFFFF, 0, "r"),
    # "I_DC_2": (0x58, 0xFFFF, 0, "r"),
    "I_AC_2": (_REG_DIA2_I_AC, 0xFFFF, 0, "r"),
    "RES_2": (_REG_DIA2_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_2": (_REG_DIA2_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_3": (_REG_DIA3_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_3": (_REG_DIA3_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_3": (_REG_DIA3_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_3": (_REG_DIA3_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_3": (_REG_DIA3_I_MONITOR, 0xFFFF, 0, "r"),
    # "I_DC_3": (0x61, 0xFFFF, 0, "r"),
    "I_AC_3": (_REG_DIA3_I_AC, 0xFFFF, 0, "r"),
    "RES_3": (_REG_DIA3_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_3": (_REG_DIA3_PWM_DUTY, 0xFFFF, 0, "r"),
    "OCP0_FAULT": (_REG_FAULT0, 0x0001, 0, "rw1c"),
    "OCP1_FAULT": (_REG_FAULT0, 0x0002, 1, "rw1c"),
    "OCP2_FAULT": (_REG_FAULT0, 0x0004, 2, "rw1c"),
    "OCP3_FAULT": (_REG_FAULT0, 0x0008, 3, "rw1c"),
    "HHF0_FAULT": (_REG_FAULT0, 0x0010, 4, "rw1c"),
    "HHF1_FAULT": (_REG_FAULT0, 0x0020, 5, "rw1c"),
    "HHF2_FAULT": (_REG_FAULT0, 0x0040, 6, "rw1c"),
    "HHF3_FAULT": (_REG_FAULT0, 0x0080, 7, "rw1c"),
    "OLF0_FAULT": (_REG_FAULT0, 0x0100, 8, "rw1c"),
    "OLF1_FAULT": (_REG_FAULT0, 0x0200, 9, "rw1c"),
    "OLF2_FAULT": (_REG_FAULT0, 0x0400, 10, "rw1c"),
    "OLF3_FAULT": (_REG_FAULT0, 0x0800, 11, "rw1c"),
    "DPM0_FAULT": (_REG_FAULT0, 0x1000, 12, "rw1c"),
    "DPM1_FAULT": (_REG_FAULT0, 0x2000, 13, "rw1c"),
    "DPM2_FAULT": (_REG_FAULT0, 0x4000, 14, "rw1c"),
    "DPM3_FAULT": (_REG_FAULT0, 0x8000, 15, "rw1c"),
    "IND0_FAULT": (_REG_FAULT1, 0x0001, 0, "rw1c"),
    "IND1_FAULT": (_REG_FAULT1, 0x0002, 1, "rw1c"),
    "IND2_FAULT": (_REG_FAULT1, 0x0004, 2, "rw1c"),
    "IND3_FAULT": (_REG_FAULT1, 0x0008, 3, "rw1c"),
    "UVM_FAULT": (_REG_FAULT1, 0x0010, 4, "rw1c"),
    "COMER_FAULT": (_REG_FAULT1, 0x0020, 5, "rw1c"),
    "OVT_FAULT": (_REG_FAULT1, 0x0040, 6, "rw1c"),
    "RES0_FAULT": (_REG_FAULT1, 0x0080, 7, "rw1c"),
    "RES1_FAULT": (_REG_FAULT1, 0x0100, 8, "rw1c"),
    "RES2_FAULT": (_REG_FAULT1, 0x0200, 9, "rw1c"),
    "RES3_FAULT": (_REG_FAULT1, 0x0400, 10, "rw1c"),
}

# SPI Timing Constants (in nanoseconds, converted to microseconds for delays)
_SPI_CLK_MIN_PERIOD_US = const(1)  # 100ns min -> 1us for safety
_SPI_SETUP_TIME_US     = const(1)  # 10ns setup -> 1us for safety
_SPI_HOLD_TIME_US      = const(1)  # 10ns hold -> 1us for safety

# Diagnostic scaling constants
_TWO_PI                = 6.283185307
_F_AC_DIVISOR          = const(65535)  # F_AC (Hz) = F_PWM_M × (F_AC_SCAN / 65535)
_DPM_CURRENT_SCALE     = const(8)     # DPM current: raw × 8 × GAIN × SNSF × K_CDR
_PWM_DUTY_MAX          = const(0x7FFF)


class MAX22216SPIError(Exception):
    """Exception raised for MAX22216 SPI communication errors."""


class MAX22216CRCError(Exception):
    """Exception raised for CRC validation errors."""


class MAX22216BitfieldWriteError(Exception):
    """Exception raised for writing to a no-write bitfield."""


class MAX22216:
    _spi: SPI | SoftSPI
    _cs: Pin
    _enable: Pin
    _crc_enable: bool
    _device_variant: str

    # Optional hardware pins (stored for convenience, not used by the driver)
    # Named with _pin suffix to avoid collision with SPI bitfield accessors
    fault_pin: Pin | None
    cntl0_pin: Pin | None
    cntl1_pin: Pin | None
    cntl2_pin: Pin | None
    cntl3_pin: Pin | None
    stat0_pin: Pin | None
    stat1_pin: Pin | None

    K_CDR: float
    IFSMAX_LS_FB: float
    IFSMAX_HS: float
    K_VDR: float
    K_VM: float
    K_R: float

    # Control mode constants
    CTRL_MODE_VDR_VDR      = const(0)  # Voltage-Voltage
    CTRL_MODE_CDR_CDR      = const(1)  # Current-Current
    CTRL_MODE_CURR_LIM_VDR = const(2)  # Current limiter-Voltage
    CTRL_MODE_VDR_CDR      = const(3)  # Voltage-Current

    # Hardware configuration constants
    HW_CFG_4_INDEPENDENT_HB    = const(0x0)
    HW_CFG_3_PARALLEL_1_INDEP  = const(0x1)
    HW_CFG_2_PARALLEL_2_INDEP  = const(0x2)
    HW_CFG_2_PARALLEL_2_PARALLEL = const(0x3)
    HW_CFG_4_PARALLEL          = const(0x4)
    HW_CFG_2_FULL_BRIDGES      = const(0x5)
    HW_CFG_1_FB_2_INDEP        = const(0x6)
    HW_CFG_1_FB_2_PARALLEL     = const(0x7)
    HW_CFG_1_PARALLEL_FB       = const(0x8)

    # PWM frequency settings
    PWM_FREQ_100K = const(0x0)
    PWM_FREQ_80K  = const(0x1)
    PWM_FREQ_60K  = const(0x2)
    PWM_FREQ_50K  = const(0x3)
    PWM_FREQ_40K  = const(0x4)
    PWM_FREQ_30K  = const(0x5)
    PWM_FREQ_25K  = const(0x6)
    PWM_FREQ_20K  = const(0x7)
    PWM_FREQ_15K  = const(0x8)
    PWM_FREQ_10K  = const(0x9)
    PWM_FREQ_7_5K = const(0xA)
    PWM_FREQ_5K   = const(0xB)
    PWM_FREQ_2_5K = const(0xC)

    # Slew rate settings
    SLEW_RATE_FAST = const(0)  # 800 V/us
    SLEW_RATE_400  = const(1)  # 400 V/us
    SLEW_RATE_200  = const(2)  # 200 V/us
    SLEW_RATE_100  = const(3)  # 100 V/us

    @classmethod
    def install_bitfield_accessors(cls, bitfields, *, exclude=()):
        """Attach property accessors for each bitfield definition."""
        excluded = {name.lower() for name in exclude}
        if not hasattr(cls, "__annotations__"):
            cls.__annotations__ = {}

        for key, bitfield in bitfields.items():
            attr_name = key.lower()
            if attr_name in excluded or attr_name in cls.__dict__:
                continue

            # Add type annotation for IDE support
            cls.__annotations__[attr_name] = int

            def getter(self, bf=bitfield):
                return self._read_bitfield(bf)

            def setter(self, value, bf=bitfield):
                self._write_bitfield(bf, value)

            doc = f"Bitfield accessor for {key}"
            setattr(cls, attr_name, property(getter, setter, doc=doc))

    def __init__(
        self,
        spi: SPI | SoftSPI,
        cs_pin: Pin,
        enable_pin: Pin,
        crc_enable: bool = False,
        device_variant: str = "MAX22216",
        fault_pin: Pin | None = None,
        cntl0_pin: Pin | None = None,
        cntl1_pin: Pin | None = None,
        cntl2_pin: Pin | None = None,
        cntl3_pin: Pin | None = None,
        stat0_pin: Pin | None = None,
        stat1_pin: Pin | None = None,
    ):
        self._spi = spi
        self._cs = cs_pin
        self._enable = enable_pin
        self._crc_enable = crc_enable
        self._device_variant = device_variant
        self.fault_pin = fault_pin
        self.cntl0_pin = cntl0_pin
        self.cntl1_pin = cntl1_pin
        self.cntl2_pin = cntl2_pin
        self.cntl3_pin = cntl3_pin
        self.stat0_pin = stat0_pin
        self.stat1_pin = stat1_pin

        # Device constants based on variant
        if self._device_variant == "MAX22216":
            self.K_CDR = 1.017  # mA constant
            self.IFSMAX_LS_FB = 3.2  # A
            self.IFSMAX_HS = 1.5  # A
        elif self._device_variant == "MAX22217":
            self.K_CDR = 0.339  # mA constant
            self.IFSMAX_LS_FB = 1.0  # A
            self.IFSMAX_HS = 0.5  # A
        else:
            raise ValueError("invalid device variant")

        self.K_VDR = 30.518e-6  # V constant
        self.K_VM = 9.73e-3  # V constant
        self.K_R = 8.437e-3  # Ω constant

    def enable(self, wait: bool = True) -> bool:
        """Enable the driver. Returns True if newly enabled.

        Args:
            wait: If True (default), sleep 1ms after enabling for stabilization.
                  Set False for batched enables where caller manages the sleep.
        """
        previous = self._enable.value()
        self._enable.value(1)
        if not previous and wait:
            time.sleep_ms(1)  # Wait for enable time (0.8ms min)
        return not previous

    def disable(self) -> None:
        self._enable.value(0)

    @staticmethod
    def _crc5(data: bytes | bytearray) -> int:
        """Calculate CRC-5-ITU for SPI data validation.

        Polynomial: x^5 + x^4 + x^2 + 1 (0x15 without leading term)
        Initial value: 0x1F
        Computed over all data bytes plus 3 padding zero bits
        (the three MSBs of the check byte per datasheet Figure 13).
        Returns 5-bit FCS in the low bits of the check byte
        (high 3 bits are zero).
        """
        crc = 0x1F  # starting value
        poly = 0x15  # x^4 + x^2 + 1 (without leading x^5 term)
        for byte in data:
            for i in range(7, -1, -1):
                bit = (byte >> i) & 1
                if ((crc >> 4) ^ bit) & 1:
                    crc = ((crc << 1) & 0x1F) ^ poly
                else:
                    crc = (crc << 1) & 0x1F
        # Process 3 padding zero bits (MSBs of CRC check byte)
        for _ in range(3):
            if crc & 0x10:
                crc = ((crc << 1) & 0x1F) ^ poly
            else:
                crc = (crc << 1) & 0x1F
        return crc

    _SPI_FLAG_NAMES = (
        (0x01, "STAT0"), (0x02, "STAT1"), (0x04, "DPM"), (0x08, "UVM"),
        (0x10, "COMER"), (0x20, "OLF"), (0x40, "OCP"), (0x80, "OVT"),
    )

    @staticmethod
    def spi_flags_str(flags: int) -> str:
        """Return human-readable string of active SPI status flags."""
        names = [n for m, n in MAX22216._SPI_FLAG_NAMES if flags & m]
        return ", ".join(names) if names else "none"

    @staticmethod
    def spi_flags(flags: int, print_flags: bool = False) -> None:
        """Check SPI status byte for errors.

        Raises MAX22216SPIError on communication error (COMER).
        """
        if print_flags and (flags & 0xFE):
            print("SPI flags:", MAX22216.spi_flags_str(flags))
        if flags & 0x10:
            raise MAX22216SPIError("Communication error (COMER)")

    def _spi_transaction(self, write_data: bytearray) -> bytearray:
        self._cs.on()
        time.sleep_us(_SPI_SETUP_TIME_US)

        self._cs.off()
        time.sleep_us(_SPI_SETUP_TIME_US)

        try:
            read_data = bytearray(len(write_data))
            self._spi.write_readinto(write_data, read_data)
            time.sleep_us(_SPI_HOLD_TIME_US)

        finally:
            self._cs.on()
            time.sleep_us(_SPI_CLK_MIN_PERIOD_US)

        self.spi_flags(read_data[0], print_flags=False)
        return read_data

    def read_register(self, reg_addr: int) -> int:
        """Read a 16-bit register value."""
        cmd = bytearray(3 if not self._crc_enable else 4)
        cmd[0] = reg_addr & 0x7F  # R/W=0 for read
        cmd[1] = 0x00
        cmd[2] = 0x00

        if self._crc_enable:
            cmd[3] = self._crc5(cmd[:3])

        _ = self._spi_transaction(cmd)
        cmd[0] = 0x00
        if self._crc_enable:
            cmd[3] = self._crc5(cmd[:3])
        response = self._spi_transaction(cmd)

        data = (response[1] << 8) | response[2]

        if self._crc_enable and len(response) >= 4:
            expected_crc = self._crc5(response[:3])
            if response[3] != expected_crc:
                raise MAX22216CRCError(
                    f"CRC mismatch: expected 0x{expected_crc:02X},"
                    f" got 0x{response[3]:02X}"
                )

        return data

    def write_register(self, reg_addr: int, value: int) -> None:
        """Write a 16-bit register value."""
        cmd = bytearray(3 if not self._crc_enable else 4)
        cmd[0] = 0x80 | (reg_addr & 0x7F)  # R/W=1 for write
        cmd[1] = (value >> 8) & 0xFF
        cmd[2] = value & 0xFF

        if self._crc_enable:
            cmd[3] = self._crc5(cmd[:3])

        self._spi_transaction(cmd)

    def _read_bitfield(self, bitfield: tuple) -> int:
        return (self.read_register(bitfield[0]) & bitfield[1]) >> bitfield[2]

    def _write_bitfield(self, bitfield: tuple, val: int) -> None:
        if "w" not in bitfield[3]:
            raise MAX22216BitfieldWriteError(f"{bitfield} is not writable")
        if "w1c" in bitfield[3]:
            # Write-1-to-clear: write only the target bit(s), don't
            # read-modify-write (that would clear other latched faults).
            self.write_register(
                bitfield[0],
                (val & (bitfield[1] >> bitfield[2])) << bitfield[2],
            )
        else:
            register = self.read_register(bitfield[0]) & ~bitfield[1] | (
                (val & (bitfield[1] >> bitfield[2])) << bitfield[2]
            )
            self.write_register(bitfield[0], register)

    # ----- register 0x00: global control

    @property
    def global_control(self) -> int:
        return self.read_register(_REG_GLOBAL_CTRL)

    @property
    def master_chopping_frequency(self) -> int:
        return self._read_bitfield(_BITFIELDS["F_PWM_M"])

    # ----- register 0x01: global configuration

    @property
    def global_config(self) -> int:
        return self.read_register(_REG_GLOBAL_CFG)

    # ----- register 0x05: motor voltage monitor

    @property
    def vm(self) -> float:
        vm = self.read_register(_REG_VM_MONITOR)
        return vm * self.K_VM

    # ----- register 0x07: F_AC (sine wave frequency)

    @property
    def f_ac(self) -> int:
        return self.read_register(_REG_F_AC) & 0x0FFF

    @f_ac.setter
    def f_ac(self, value: int) -> None:
        self.write_register(_REG_F_AC, value & 0x0FFF)

    # ----- register 0x08: U_AC_SCAN (AC amplitude)

    @property
    def u_ac_scan(self) -> int:
        return self.read_register(_REG_U_AC_SCAN) & 0x7FFF

    @u_ac_scan.setter
    def u_ac_scan(self, value: int) -> None:
        self.write_register(_REG_U_AC_SCAN, value & 0x7FFF)

    # ----- register 0x65: fault log 0

    @property
    def fault0(self) -> int:
        return self.read_register(_REG_FAULT0)

    def fault0_clear(self) -> None:
        self.write_register(_REG_FAULT0, 0xFFFF)

    # ----- register 0x66: fault log 1

    @property
    def fault1(self) -> int:
        return self.read_register(_REG_FAULT1)

    def fault1_clear(self) -> None:
        self.write_register(_REG_FAULT1, 0xFFFF)

    # ----- per-channel: RES_THLD (resistance threshold)

    def res_thld(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return self.read_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE))

    def set_res_thld(self, channel: int, value: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        self.write_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE), value & 0xFFFF)

    # ----- per-channel: IAC_THLD (inductance/I_AC threshold)

    def iac_thld(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return self.read_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE)) & 0x0FFF

    def set_iac_thld(self, channel: int, value: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        self.write_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE), value & 0x0FFF)

    # ----- unit conversion helpers

    def ohms_to_res_thld(self, ohms: float) -> int:
        """Convert resistance in ohms to RES_THLD register value."""
        return int(ohms / self.K_R)

    def ma_to_dpm_thld(self, delta_ma: float, channel: int = 0) -> int:
        """Convert current delta in mA to DPM_THLD register value.

        Accounts for the channel's SNSF and GAIN settings.
        """
        scale = self._channel_scale(channel)
        return int(delta_ma / (8 * scale * self.K_CDR))

    # ----- convenience functions

    @property
    def fault(self) -> int:
        return (self.fault1 << 16) | self.fault0

    def fault_clear(self) -> None:
        self.fault0_clear()
        self.fault1_clear()

    # F_PWM_M[3:0] → master PWM frequency in kHz (Table 4)
    _PWM_FREQ_KHZ = (
        100, 80, 60, 50, 40, 30, 25, 20, 15, 10, 7.5, 5, 2.5,
    )
    # F_PWM[1:0] → per-channel divider (Table 5)
    _PWM_DIV = (1, 2, 4, 8)

    # GAIN[1:0] and SNSF[1:0] register value → multiplier (Table 9)
    _GAIN_TABLE = (1.0, 0.5, 1 / 3, 0.25)
    _SNSF_TABLE = (1.0, 2 / 3, 1 / 3)

    def _channel_scale(self, channel: int) -> float:
        """Read SNSF and GAIN from CTRL1, return combined multiplier.

        SNSF selects the sense FET R_ON (MAX22216 only, always 1 for
        MAX22217).  GAIN scales the internal current measurement.
        Both affect all current-related diagnostics.
        """
        ctrl1 = self.read_register(
            _REG_CFG0_CTRL1 + (channel * _CH_STRIDE)
        )
        snsf_reg = ctrl1 & 0x03
        gain_reg = (ctrl1 >> 2) & 0x03
        gain = self._GAIN_TABLE[gain_reg]
        if (
            self._device_variant == "MAX22217"
            or snsf_reg >= len(self._SNSF_TABLE)
        ):
            snsf = 1.0
        else:
            snsf = self._SNSF_TABLE[snsf_reg]
        return snsf * gain

    def pwm_freq_khz(self, channel: int) -> float:
        """Return the actual PWM frequency for a channel in kHz.

        Reads F_PWM_M (master) from GLOBAL_CTRL and F_PWM (divider)
        from the channel's CTRL1 register.
        """
        master_reg = self._read_bitfield(_BITFIELDS["F_PWM_M"])
        if master_reg >= len(self._PWM_FREQ_KHZ):
            master_reg = 0  # 0xD/0xE/0xF alias to 100kHz
        master_khz = self._PWM_FREQ_KHZ[master_reg]
        ctrl1 = self.read_register(
            _REG_CFG0_CTRL1 + (channel * _CH_STRIDE)
        )
        div_reg = (ctrl1 >> 8) & 0x03
        return master_khz / self._PWM_DIV[div_reg]

    # Fault mask bit positions (9-bit mask, one per fault type)
    FMASK_UVM  = const(0x001)
    FMASK_COMF = const(0x002)
    FMASK_DPM  = const(0x004)
    FMASK_HHF  = const(0x008)
    FMASK_OLF  = const(0x010)
    FMASK_OCP  = const(0x020)
    FMASK_OVT  = const(0x040)
    FMASK_IND  = const(0x080)  # no HW mask bit (warning only)
    FMASK_RES  = const(0x100)  # no HW mask bit (warning only)

    # Map from mask bits to GLOBAL_CFG bitfield property names
    _FMASK_TO_HW = {
        0x001: "m_uvm",
        0x002: "m_comf",
        0x004: "m_dpm",
        0x008: "m_hhf",
        0x010: "m_olf",
        0x020: "m_ocp",
        0x040: "m_ovt",
    }

    def apply_fault_mask(self, mask: int) -> None:
        """Write fault mask bits to GLOBAL_CFG in a single register write.

        Single read-modify-write instead of 7 individual RMW cycles.

        Args:
            mask: 9-bit mask (bits 0-6 map to HW mask fields,
                  bits 7-8 are IND/RES with no HW equivalent).
        """
        reg = self.global_config
        reg &= ~0x7F00  # Clear M_UVM..M_OVT [14:8]
        reg |= (mask & 0x7F) << 8  # Set new mask bits
        self.write_register(_REG_GLOBAL_CFG, reg)

    def activate(
        self,
        vdr: bool = True,
        chs: int = 0,
        clear_flags: bool = True,
        fault_mask: int = 0,
    ) -> None:
        """Activate the device (set ACTIVE bit in GLOBAL_CFG).

        Sets CHS, VDR_NDUTY, ACTIVE, and fault mask bits in a single
        read-modify-write of GLOBAL_CFG instead of multiple separate writes.

        Args:
            vdr: Enable VDR duty mode (voltage proportional
                 to target, not supply).
            chs: Hardware configuration 0x0-0x8 (see Table 1).
                 0x0=4x independent HB (default), 0x5=2x full bridge, etc.
            clear_flags: Wait for supply to stabilize and
                         clear fault registers.
            fault_mask: 9-bit fault mask to apply after activation.
        """
        # Single read-modify-write: CHS + VDR + fault mask + ACTIVE
        reg = self.global_config
        reg &= 0x0060  # Preserve reserved[5], STAT_POL[6], CNTL_POL[7]
        reg |= chs & 0x0F  # CHS[3:0]
        reg |= int(vdr) << 4  # VDR_NDUTY[4]
        reg |= (fault_mask & 0x7F) << 8  # M_UVM..M_OVT[14:8]
        reg |= 0x8000  # ACTIVE[15]
        self.write_register(_REG_GLOBAL_CFG, reg)
        if clear_flags:
            time.sleep_ms(10)
            self.fault_clear()

    def config_channel(
        self,
        channel: int,
        dc_h: int,
        dc_l2h: int,
        time_l2h: int,
        dpm_enable: bool,
        l_meas_h: bool,
        f_ac: int,
        u_ac: int,
        dc_l: int | None = None,
        res_thld: int | None = None,
        iac_thld: int | None = None,
        dpm_thld: int | None = None,
        dpm_start: int | None = None,
        dpm_min_nbr: int | None = None,
        end_hit_auto: bool | None = None,
        end_hit_to_hiz_auto: bool | None = None,
        ol_en: bool = True,
        l_meas_l2h: bool = False,
        l_meas_wcycles: int | None = None,
        l_nbr_calc: int | None = None,
    ) -> None:
        """Configure a channel's sequencer, DPM, and inductance settings.

        Args:
            channel: Channel 0-3.
            dc_h: Hold duty cycle (16-bit raw). Applied after excitation.
            dc_l2h: Excitation (hit) duty cycle (16-bit raw). Initial pull-in.
            time_l2h: Excitation duration (16-bit raw PWM cycles).
            dpm_enable: Enable plunger movement detection during excitation.
            l_meas_h: Enable inductance measurement during hold phase.
            f_ac: Sine wave frequency for inductance/dither (12-bit raw).
                Global register — shared across all channels.
            u_ac: Sine wave amplitude (15-bit raw).
                Global register — shared across all channels.
            dc_l: Off-phase duty cycle (16-bit raw); None=keep current.
            res_thld: Resistance fault threshold (16-bit raw); None=keep current.
            iac_thld: AC current fault threshold (12-bit raw); None=keep current.
            dpm_thld: DPM current delta threshold (12-bit); None=chip default.
            dpm_start: DPM start delay (8-bit); None=chip default.
            dpm_min_nbr: DPM deglitch count (4-bit); None=chip default.
            end_hit_auto: Auto hit→hold on DPM detection; None=keep current.
            end_hit_to_hiz_auto: Auto hit→HiZ on DPM detection; None=keep current.
            ol_en: Open-load detection enable (default True).
            l_meas_l2h: Enable inductance measurement during hit phase.
            l_meas_wcycles: Inductance measurement wait cycles (0-15); None=chip default.
            l_nbr_calc: Inductance measurement averaging periods (0-15); None=chip default.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        self.write_register(_REG_CFG0_DC_H + (channel * _CH_STRIDE), dc_h)
        self.write_register(_REG_CFG0_DC_L2H + (channel * _CH_STRIDE), dc_l2h)
        self.write_register(_REG_CFG0_L2H_TIME + (channel * _CH_STRIDE), time_l2h)
        if dc_l is not None:
            self.write_register(_REG_CFG0_DC_L + (channel * _CH_STRIDE), dc_l)
        if dpm_enable:
            self.dpm_config(
                channel, dpm_thld, dpm_start, dpm_min_nbr,
                end_hit_auto, end_hit_to_hiz_auto,
            )
        # L_MEAS_H and L_MEAS_L2H select which phases to measure during.
        # L_MEAS_EN (master enable) is toggled separately at fire time to
        # avoid IND faults while the channel is off.
        if l_meas_h or l_meas_l2h:
            self.write_register(_REG_F_AC, f_ac)
            self.write_register(_REG_U_AC_SCAN, u_ac)
        if l_meas_h:
            setattr(self, f"l_meas_h_{channel}", 1)
        if l_meas_l2h:
            setattr(self, f"l_meas_l2h_{channel}", 1)
        if l_meas_wcycles is not None:
            setattr(self, f"l_meas_wcycles_{channel}", l_meas_wcycles & 0x0F)
        if l_nbr_calc is not None:
            setattr(self, f"l_nbr_calc_{channel}", l_nbr_calc & 0x0F)
        if res_thld is not None:
            self.write_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE), res_thld)
        if iac_thld is not None:
            self.write_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE), iac_thld & 0x0FFF)
        self.config_ctrl0(channel, ol_en=ol_en)

    def config_ctrl0(
        self,
        channel: int,
        ctrl_mode: int = 0,
        hhf_en: bool = False,
        ol_en: bool = True,
        h2l_en: bool = False,
        ramp: int = 0,
        rupe: bool = False,
        rmde: bool = False,
        rdwe: bool = False,
    ) -> None:
        """Configure CFG_CTRLx_0 register for a channel.

        Args:
            channel: Channel 0-3
            ctrl_mode: 0=VDR/VDR (solenoid), 1=CDR/CDR (proportional valve),
                       2=Current limiter/VDR (DC motor), 3=VDR/CDR (mixed)
            hhf_en: Hit current not reached fault enable
            ol_en: Open-load detection enable (detects disconnected solenoids when OFF)
            h2l_en: Fast demagnetization enable (bridge operation only)
            ramp: Ramp slew rate value (8-bit, 0-255)
            rupe: Ramp-up enable
            rmde: Ramp-mid enable
            rdwe: Ramp-down enable
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        value = (
            (ramp & 0xFF)
            | (int(rupe) << 8)
            | (int(rmde) << 9)
            | (int(rdwe) << 10)
            | (int(h2l_en) << 11)
            | (int(ol_en) << 12)
            | (int(hhf_en) << 13)
            | ((ctrl_mode & 0x3) << 14)
        )
        self.write_register(_REG_CFG0_CTRL0 + (channel * _CH_STRIDE), value)

    def dpm_enable(self, channel: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        setattr(self, f"dpm_en_{channel}", 1)

    def dpm_disable(self, channel: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        setattr(self, f"dpm_en_{channel}", 0)

    def dpm_config(
        self,
        channel: int,
        thld: int | None = None,
        start: int | None = None,
        min_nbr: int | None = None,
        end_hit_auto: bool | None = None,
        end_hit_to_hiz_auto: bool | None = None,
    ) -> None:
        """Configure DPM (Detection of Plunger Movement) for a channel.

        Args:
            channel: Channel 0-3
            thld: DPM threshold (12-bit, 0-4095) - current delta for detection, None=keep default
            start: DPM start delay in 10µs units (8-bit, 0-255), None=keep default
            min_nbr: Minimum samples before detection valid (4-bit, 0-15), None=keep default
            end_hit_auto: Auto DC_L2H→DC_H switchover on valid BEMF dip detection.
                Cuts hit phase short once plunger movement confirmed, saving power.
            end_hit_to_hiz_auto: Auto DC_L2H→HiZ switchover on valid BEMF dip detection.
                For latching valves that only need a pulse to change state.

        If all parameters are None, just enables DPM with chip defaults.
        """

        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")

        # Configure DPM_THLD in DPM0 register
        if thld is not None:
            self.write_register(
                _REG_CFG0_DPM0 + (channel * _CH_STRIDE),
                thld & 0x0FFF,
            )

        # Configure DPM_START, DPM_MIN_NBR, END_HIT flags, and DPM_EN in DPM1
        dpm1_reg = _REG_CFG0_DPM1 + (channel * _CH_STRIDE)
        need_rmw = (
            start is not None
            or min_nbr is not None
            or end_hit_auto is not None
            or end_hit_to_hiz_auto is not None
        )

        if need_rmw:
            dpm1_val = self.read_register(dpm1_reg)
            if start is not None:
                dpm1_val = (dpm1_val & ~0x00FF) | (start & 0xFF)
            if min_nbr is not None:
                dpm1_val = (dpm1_val & ~0x0F00) | ((min_nbr & 0x0F) << 8)
            if end_hit_auto is not None:
                if end_hit_auto:
                    dpm1_val |= 0x1000
                else:
                    dpm1_val &= ~0x1000
            if end_hit_to_hiz_auto is not None:
                if end_hit_to_hiz_auto:
                    dpm1_val |= 0x2000
                else:
                    dpm1_val &= ~0x2000
            dpm1_val |= 0x4000  # DPM_EN bit 14
            self.write_register(dpm1_reg, dpm1_val)
        else:
            self.dpm_enable(channel)

    def dpm(self, channel: int) -> tuple:
        """Read DPM diagnostics: (peak_mA, valley_mA, travel_ms, react_ms).

        Current: raw × 8 × GAIN × SNSF × K_CDR (mA).
        Time: raw / (F_PWM_M × F_PWM_divider) in ms.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _DIAG_STRIDE
        peak = self.read_register(_REG_DIA0_I_DPM_PEAK + offset)
        valley = self.read_register(_REG_DIA0_I_DPM_VALLEY + offset)
        travel = self.read_register(_REG_DIA0_TRAVEL_TIME + offset)
        react = self.read_register(_REG_DIA0_REACTION_TIME + offset)
        scale = self._channel_scale(channel)
        k = _DPM_CURRENT_SCALE * scale * self.K_CDR
        f_khz = self.pwm_freq_khz(channel)
        return (
            round(peak * k, 2),
            round(valley * k, 2),
            round(travel / f_khz, 2) if f_khz else 0,
            round(react / f_khz, 2) if f_khz else 0,
        )

    def _read_diag(self, reg: int) -> int:
        """Read diagnostic register, treating 0xFF00+ as 0 (invalid mid-ADC)."""
        val = self.read_register(reg)
        return 0 if val >= 0xFF00 else val

    def diagnostics_raw(self, channel: int) -> dict:
        """Get raw diagnostic register values for debugging."""
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _DIAG_STRIDE
        return {
            "peak": self.read_register(_REG_DIA0_I_DPM_PEAK + offset),
            "valley": self.read_register(_REG_DIA0_I_DPM_VALLEY + offset),
            "travel": self.read_register(_REG_DIA0_TRAVEL_TIME + offset),
            "react": self.read_register(_REG_DIA0_REACTION_TIME + offset),
            "i_mon": self.read_register(_REG_DIA0_I_MONITOR + offset),
            "i_ac": self.read_register(_REG_DIA0_I_AC + offset),
            "res": self.read_register(_REG_DIA0_RES + offset),
            "pwm": self.read_register(_REG_DIA0_PWM_DUTY + offset),
        }

    def diagnostics(self, channel: int, vm: float | None = None) -> dict:
        """Read fully-scaled diagnostics for a channel.

        Args:
            channel: Channel 0-3.
            vm: Pre-read VM voltage (avoids extra SPI read when
                caller already has it). Read from hardware if None.

        Returns dict with keys:
            voltage, peak_ma, valley_ma, travel_ms, reaction_ms,
            i_monitor_ma, i_ac_ma, resistance_ohm, inductance_mh,
            pwm_duty

        Current scaling uses SNSF and GAIN read from CTRL1.
        Inductance uses U_AC_SCAN amplitude and F_AC frequency.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _DIAG_STRIDE
        scale = self._channel_scale(channel)
        k = scale * self.K_CDR
        k_dpm = _DPM_CURRENT_SCALE * k

        f_khz = self.pwm_freq_khz(channel)

        peak_ma = round(
            self._read_diag(_REG_DIA0_I_DPM_PEAK + offset) * k_dpm,
            2,
        )
        valley_ma = round(
            self._read_diag(_REG_DIA0_I_DPM_VALLEY + offset) * k_dpm,
            2,
        )
        # Timer registers are PWM counters, not ADC values — don't apply _read_diag filter
        travel_raw = self.read_register(_REG_DIA0_TRAVEL_TIME + offset)
        travel_ms = round(travel_raw / f_khz, 2) if f_khz else 0
        react_raw = self.read_register(_REG_DIA0_REACTION_TIME + offset)
        reaction_ms = round(react_raw / f_khz, 2) if f_khz else 0
        i_mon_ma = round(
            self._read_diag(_REG_DIA0_I_MONITOR + offset) * k, 2
        )
        i_ac_ma = round(
            self._read_diag(_REG_DIA0_I_AC + offset) * k, 2
        )
        # Resistance: R(mΩ) = raw × K_R / (SNSF × GAIN)
        res_raw = self._read_diag(_REG_DIA0_RES + offset)
        res_ohm = round(res_raw * (self.K_R / scale), 2) if scale else 0
        pwm_raw = self._read_diag(_REG_DIA0_PWM_DUTY + offset)
        pwm_pct = round((pwm_raw / _PWM_DUTY_MAX) * 100, 1) if pwm_raw else 0

        if vm is None:
            vm = self.vm

        # Inductance: L = U_AC_SCAN / (2π × F_AC × I_AC)
        # F_AC (Hz) = F_PWM_M (Hz) × (F_AC_SCAN / 65535)
        # U_AC_SCAN voltage = K_VDR × 36 × U_AC_SCAN_reg (VDR mode)
        if i_ac_ma > 1:
            f_ac_reg = self.read_register(_REG_F_AC) & 0x0FFF
            f_hz = f_khz * 1000 * (f_ac_reg / _F_AC_DIVISOR)
            u_ac_reg = self.read_register(_REG_U_AC_SCAN) & 0x7FFF
            u_ac_v = self.K_VDR * 36 * u_ac_reg
            if f_hz > 0:
                ind_mh = round(
                    (u_ac_v / (_TWO_PI * f_hz * (i_ac_ma / 1000)))
                    * 1000,
                    2,
                )
            else:
                ind_mh = 0
        else:
            ind_mh = 0

        return {
            "voltage": round(vm, 2),
            "peak_ma": peak_ma,
            "valley_ma": valley_ma,
            "travel_ms": travel_ms,
            "reaction_ms": reaction_ms,
            "i_monitor_ma": i_mon_ma,
            "i_ac_ma": i_ac_ma,
            "resistance_ohm": res_ohm,
            "inductance_mh": ind_mh,
            "pwm_duty": pwm_pct,
        }

    # ----- channel on/off

    def set_channel(self, channel: int, enabled: bool) -> None:
        """Enable or disable a channel via the CNTL bit in GLOBAL_CTRL.

        Args:
            channel: Channel 0-3.
            enabled: True to enable, False to disable.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        reg = self.read_register(_REG_GLOBAL_CTRL)
        bit = 1 << channel
        if enabled:
            reg |= bit
        else:
            reg &= ~bit
        self.write_register(_REG_GLOBAL_CTRL, reg)

    def channel_enabled(self, channel: int) -> bool:
        """Return True if the CNTL bit for a channel is set."""
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return bool(self.read_register(_REG_GLOBAL_CTRL) & (1 << channel))

    # ----- single-channel current monitor

    def i_monitor(self, channel: int) -> float:
        """Read instantaneous current for a channel in mA.

        I_MONITOR(mA) = K_CDR × GAIN × SNSF × I_MONITOR[15:0]
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        raw = self.read_register(
            _REG_DIA0_I_MONITOR + channel * _DIAG_STRIDE
        )
        scale = self._channel_scale(channel)
        return round(raw * scale * self.K_CDR, 2)

    # ----- CTRL1 configuration

    def config_ctrl1(
        self,
        channel: int,
        hsnls: int | None = None,
        f_pwm: int | None = None,
        slew_rate: int | None = None,
        t_blanking: int | None = None,
        gain: int | None = None,
        snsf: int | None = None,
    ) -> None:
        """Configure CFG_CTRLx_1 register for a channel.

        Args:
            channel: Channel 0-3
            hsnls: 0=low-side driver, 1=high-side driver
            f_pwm: Per-channel PWM divider (0=F_PWM_M, 1=/2, 2=/4, 3=/8)
            slew_rate: 0=fast, 1=400V/µs, 2=200V/µs, 3=100V/µs
            t_blanking: Blanking time (0=0, 1=24, 2=48, 3=96 clock cycles)
            gain: CDR gain (0=1, 1=1/2, 2=1/3, 3=1/4)
            snsf: Sense scaling (0=full, 1=2/3, 2=1/3) — MAX22216 only

        None values leave the current setting unchanged.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        reg_addr = _REG_CFG0_CTRL1 + (channel * _CH_STRIDE)
        val = self.read_register(reg_addr)
        if snsf is not None:
            val = (val & ~0x0003) | (snsf & 0x03)
        if gain is not None:
            val = (val & ~0x000C) | ((gain & 0x03) << 2)
        if slew_rate is not None:
            val = (val & ~0x0030) | ((slew_rate & 0x03) << 4)
        if t_blanking is not None:
            val = (val & ~0x00C0) | ((t_blanking & 0x03) << 6)
        if f_pwm is not None:
            val = (val & ~0x0300) | ((f_pwm & 0x03) << 8)
        if hsnls is not None:
            val = (val & ~0x0400) | ((hsnls & 0x01) << 10)
        self.write_register(reg_addr, val)

    # ----- PI controller configuration

    def config_pi(
        self,
        channel: int,
        kp: float,
        ki: float,
    ) -> None:
        """Configure PI controller gains for CDR mode.

        Args:
            channel: Channel 0-3.
            kp: Proportional gain in Q8.8 format (0-255.996).
            ki: Integral gain in Q8.8 format (0-255.996).
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _CH_STRIDE
        self.write_register(
            _REG_CFG0_P + offset, int(kp * 256) & 0xFFFF
        )
        self.write_register(
            _REG_CFG0_I + offset, int(ki * 256) & 0xFFFF
        )

    # ----- status register

    def read_status(self) -> dict:
        """Read and decode the STATUS register (0x02).

        Returns dict with all status bits as booleans.
        """
        raw = self.read_register(_REG_STATUS)
        return {
            "ovt": bool(raw & 0x0080),
            "ocp": bool(raw & 0x0040),
            "olf": bool(raw & 0x0020),
            "hhf": bool(raw & 0x0010),
            "dpm": bool(raw & 0x0008),
            "comer": bool(raw & 0x0004),
            "uvm": bool(raw & 0x0002),
            "ind": bool(raw & 0x0100),
            "res": bool(raw & 0x0200),
            "min_t_on": bool(raw & 0x0400),
            "stat0": bool(raw & 0x0800),
            "stat1": bool(raw & 0x1000),
            "stat2": bool(raw & 0x2000),
            "stat3": bool(raw & 0x4000),
            "raw": raw,
        }

    # ----- register dump for debugging

    _GLOBAL_REG_NAMES = (
        (0x00, "GLOBAL_CTRL"),
        (0x01, "GLOBAL_CFG"),
        (0x02, "STATUS"),
        (0x03, "STATUS_CFG"),
        (0x04, "DC_H2L"),
        (0x05, "VM_MONITOR"),
        (0x06, "VM_THRESHOLD"),
        (0x07, "F_AC"),
        (0x08, "U_AC_SCAN"),
    )

    _CH_REG_NAMES = (
        "DC_L2H", "DC_H", "DC_L", "L2H_TIME",
        "CTRL0", "CTRL1", "DPM0", "DPM1",
        "IDC_THLD", "R_THLD", "IND_0", "IND_1", "P", "I",
    )

    def dump_registers(self) -> None:
        """Print all global and per-channel config registers."""
        print("--- Global Registers ---")
        for addr, name in self._GLOBAL_REG_NAMES:
            val = self.read_register(addr)
            print(f"  0x{addr:02X} {name:14s} = 0x{val:04X}")
        for ch in range(4):
            print(f"--- Channel {ch} Config ---")
            base = _REG_CFG0_DC_L2H + ch * _CH_STRIDE
            for i, name in enumerate(self._CH_REG_NAMES):
                val = self.read_register(base + i)
                print(f"  0x{base + i:02X} {name:14s} = 0x{val:04X}")
        f0 = self.fault0
        f1 = self.fault1
        print(f"--- Faults ---")
        print(f"  0x65 FAULT0         = 0x{f0:04X}")
        print(f"  0x66 FAULT1         = 0x{f1:04X}")

    def __repr__(self) -> str:
        return (
            f"<{self._device_variant}"
            f" active={bool(self.active)}"
            f" crc={self._crc_enable}>"
        )

    def test(self) -> None:
        """Print diagnostic summary."""
        print(self._device_variant)
        print(f"  VM: {self.vm:.2f}V")
        status = self.read_register(_REG_STATUS)
        gcfg = self.read_register(_REG_GLOBAL_CFG)
        gctrl = self.read_register(_REG_GLOBAL_CTRL)
        print(f"  status: 0x{status:04X}  active: {bool(gcfg & 0x8000)}")
        print(
            f"  global_ctrl: 0x{gctrl:04X}"
            f"  global_cfg: 0x{gcfg:04X}"
        )
        f0 = self.fault0
        f1 = self.fault1
        if f0 or f1:
            print(f"  *** FAULTS: 0x{f0:04X} 0x{f1:04X} ***")
        else:
            print("  faults: none")
        _MODES = ("VDR/VDR", "CDR/CDR", "CLIM/VDR", "VDR/CDR")
        for ch in range(4):
            off = ch * _CH_STRIDE
            ctrl0 = self.read_register(_REG_CFG0_CTRL0 + off)
            ctrl1 = self.read_register(_REG_CFG0_CTRL1 + off)
            dpm1 = self.read_register(_REG_CFG0_DPM1 + off)
            mode = _MODES[(ctrl0 >> 14) & 0x3]
            on = bool(gctrl & (1 << ch))
            dpm_on = bool(dpm1 & 0x4000)
            scale = self._channel_scale(ch)
            f_khz = self.pwm_freq_khz(ch)
            print(
                f"  ch{ch}: {mode}  on={on}  dpm={dpm_on}"
                f"  scale={scale:.3f}  pwm={f_khz}kHz"
            )


# Install bitfield property accessors once at module load time
MAX22216.install_bitfield_accessors(_BITFIELDS)
