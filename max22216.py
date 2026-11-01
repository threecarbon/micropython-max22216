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


__version__ = "1.0.0"


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
# 0x46 is not in the data sheet register map
_REG_DIA0_I_AC          = const(0x47)
_REG_DIA0_RES           = const(0x48)
_REG_DIA0_PWM_DUTY      = const(0x49)

# diagnostics channel 1
_REG_DIA1_I_DPM_PEAK    = const(0x4A)
_REG_DIA1_I_DPM_VALLEY  = const(0x4B)
_REG_DIA1_TRAVEL_TIME   = const(0x4C)
_REG_DIA1_REACTION_TIME = const(0x4D)
_REG_DIA1_I_MONITOR     = const(0x4E)
# 0x4F is not in the data sheet register map
_REG_DIA1_I_AC          = const(0x50)
_REG_DIA1_RES           = const(0x51)
_REG_DIA1_PWM_DUTY      = const(0x52)

# diagnostics channel 2
_REG_DIA2_I_DPM_PEAK    = const(0x53)
_REG_DIA2_I_DPM_VALLEY  = const(0x54)
_REG_DIA2_TRAVEL_TIME   = const(0x55)
_REG_DIA2_REACTION_TIME = const(0x56)
_REG_DIA2_I_MONITOR     = const(0x57)
# 0x58 is not in the data sheet register map
_REG_DIA2_I_AC          = const(0x59)
_REG_DIA2_RES           = const(0x5A)
_REG_DIA2_PWM_DUTY      = const(0x5B)

# diagnostics channel 3
_REG_DIA3_I_DPM_PEAK    = const(0x5C)
_REG_DIA3_I_DPM_VALLEY  = const(0x5D)
_REG_DIA3_TRAVEL_TIME   = const(0x5E)
_REG_DIA3_REACTION_TIME = const(0x5F)
_REG_DIA3_I_MONITOR     = const(0x60)
# 0x61 is not in the data sheet register map
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
    "I_AC_0": (_REG_DIA0_I_AC, 0xFFFF, 0, "r"),
    "RES_0": (_REG_DIA0_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_0": (_REG_DIA0_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_1": (_REG_DIA1_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_1": (_REG_DIA1_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_1": (_REG_DIA1_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_1": (_REG_DIA1_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_1": (_REG_DIA1_I_MONITOR, 0xFFFF, 0, "r"),
    "I_AC_1": (_REG_DIA1_I_AC, 0xFFFF, 0, "r"),
    "RES_1": (_REG_DIA1_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_1": (_REG_DIA1_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_2": (_REG_DIA2_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_2": (_REG_DIA2_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_2": (_REG_DIA2_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_2": (_REG_DIA2_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_2": (_REG_DIA2_I_MONITOR, 0xFFFF, 0, "r"),
    "I_AC_2": (_REG_DIA2_I_AC, 0xFFFF, 0, "r"),
    "RES_2": (_REG_DIA2_RES, 0xFFFF, 0, "r"),
    "PWM_DUTYCYCLE_2": (_REG_DIA2_PWM_DUTY, 0xFFFF, 0, "r"),
    "I_DPM_PEAK_3": (_REG_DIA3_I_DPM_PEAK, 0xFFFF, 0, "r"),
    "I_DPM_VALLEY_3": (_REG_DIA3_I_DPM_VALLEY, 0xFFFF, 0, "r"),
    "TRAVEL_TIME_3": (_REG_DIA3_TRAVEL_TIME, 0xFFFF, 0, "r"),
    "REACTION_TIME_3": (_REG_DIA3_REACTION_TIME, 0xFFFF, 0, "r"),
    "I_MONITOR_3": (_REG_DIA3_I_MONITOR, 0xFFFF, 0, "r"),
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

# Diagnostic scaling constants
_TWO_PI                = 6.283185307
_F_AC_DIVISOR          = const(65535)  # F_AC (Hz) = F_PWM_M × (F_AC_SCAN / 65535)
_DPM_CURRENT_SCALE     = const(8)     # DPM current: raw × 8 × GAIN × SNSF × K_CDR
_PWM_DUTY_MAX          = const(0x7FFF)


def _check_range(name: str, value: int, high: int) -> None:
    """Raise ValueError unless 0 <= value <= high. A setter refuses a value
    outside its field: masking it wrote a different value than the one
    asked for."""
    if not 0 <= value <= high:
        raise ValueError(f"{name} must be 0-{high}")


class MAX22216SPIError(Exception):
    """Exception raised for MAX22216 SPI communication errors."""


class MAX22216CRCError(MAX22216SPIError):
    """Exception raised for CRC validation errors.

    A subclass of the SPI error: a CRC-corrupted read is a communication
    failure like any other, so a caller that retries or clears faults on
    MAX22216SPIError handles it the same way.
    """


class MAX22216BitfieldWriteError(Exception):
    """Exception raised for writing to a no-write bitfield."""


def _build_crc5_tables():
    """State-transition tables for the table-driven CRC-5: ``table[(state
    << 8) | byte]`` is the state after one data byte, ``pad[state]`` the
    check value after the three padding bits. Built from the bitwise
    reference so the two can never disagree."""
    poly = 0x15
    table = bytearray(32 * 256)
    for state in range(32):
        for byte in range(256):
            crc = state
            for i in range(7, -1, -1):
                bit = (byte >> i) & 1
                if ((crc >> 4) ^ bit) & 1:
                    crc = ((crc << 1) & 0x1F) ^ poly
                else:
                    crc = (crc << 1) & 0x1F
            table[(state << 8) | byte] = crc
    pad = bytearray(32)
    for state in range(32):
        crc = state
        for _ in range(3):
            if crc & 0x10:
                crc = ((crc << 1) & 0x1F) ^ poly
            else:
                crc = (crc << 1) & 0x1F
        pad[state] = crc
    return bytes(table), bytes(pad)


_CRC5_TABLE, _CRC5_PAD = _build_crc5_tables()


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
        self._scale_cache: dict = {}
        self._kint_cache: dict = {}  # channel -> (k_milli, k_dpm_centi) integer current scales
        self._pwm_cache: dict = {}
        self._ac_cache: tuple | None = None  # (F_AC, U_AC_SCAN) until written
        # One output dict per channel, filled in place by diagnostics(),
        # so a periodic read allocates neither a dict nor boxed floats.
        self._diag_out: list = [None, None, None, None]
        # Event registers (DPM peak/valley, travel, reaction, resistance)
        # latch at a switching edge and do not move while a channel is
        # held or idle; the last full read is kept per channel so a
        # live-only read can return a complete dict.
        self._event_cache: dict = {}
        self._rx4 = bytearray(4)  # reply buffer for the 4-byte (CRC) frames
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
    def _crc5(data: bytes | bytearray, n: int = -1) -> int:
        """CRC-5-ITU of ``data`` (its first ``n`` bytes when given, so the
        hot path slices nothing) plus the three padding bits, table-driven.

        Bit-exact with ``_crc5_bitwise`` (the table is built from it at
        import); ~20 µs instead of ~420 µs per call on the RP2350.
        """
        crc = 0x1F
        table = _CRC5_TABLE
        if n < 0:
            n = len(data)
        for i in range(n):
            crc = table[(crc << 8) | data[i]]
        return _CRC5_PAD[crc]

    @staticmethod
    def _crc5_bitwise(data: bytes | bytearray) -> int:
        """Calculate CRC-5-ITU for SPI data validation (reference).

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

    def _spi_transaction(self, write_data) -> bytearray:
        """One chip-select cycle: clock ``write_data`` out, the status byte
        and the reply in.

        No explicit setup/hold waits: the datasheet's CS-to-SCK and
        SCK-to-CS minimums are tens of nanoseconds, and each MicroPython
        statement between the pin writes and the transfer already takes
        microseconds (measured: the three ``sleep_us`` calls and the
        per-call buffer were a third of a 360 µs transaction). CRC on every
        frame would show a timing problem as a CRC error.
        """
        cs = self._cs
        read_data = self._rx4 if len(write_data) == 4 else bytearray(len(write_data))
        cs.off()
        try:
            self._spi.write_readinto(write_data, read_data)
        finally:
            cs.on()
        if read_data[0] & 0x10:
            raise MAX22216SPIError("Communication error (COMER)")
        return read_data

    reads = 0  # read_register calls across all instances, for profiling (the caller resets it)
    # Read command frames by register address, CRC included: the two
    # frames of a read never change for a given address, so they are
    # built once. The second frame (address 0, data 0) is the same for
    # every read.
    _read_cmds: dict = {}
    _read_cmd2: dict = {}

    def _read_cmd(self, reg_addr: int) -> bytes:
        key = (reg_addr & 0x7F, self._crc_enable)
        cmd = MAX22216._read_cmds.get(key)
        if cmd is None:
            buf = bytearray(4 if self._crc_enable else 3)
            buf[0] = reg_addr & 0x7F  # R/W=0 for read
            if self._crc_enable:
                buf[3] = self._crc5(buf[:3])
            cmd = MAX22216._read_cmds[key] = bytes(buf)
        return cmd

    def read_register(self, reg_addr: int) -> int:
        """Read a 16-bit register value (two transactions, one CRC check)."""
        MAX22216.reads += 1
        self._spi_transaction(self._read_cmd(reg_addr))
        response = self._spi_transaction(self._read_cmd(0))
        # ``response`` is the shared reply buffer: decode before any
        # further transaction.
        data = (response[1] << 8) | response[2]

        if self._crc_enable and len(response) >= 4:
            expected_crc = self._crc5(response, 3)
            if response[3] != expected_crc:
                raise MAX22216CRCError(
                    f"CRC mismatch: expected 0x{expected_crc:02X},"
                    f" got 0x{response[3]:02X}"
                )

        return data

    def write_register(self, reg_addr: int, value: int) -> None:
        """Write a 16-bit register value (0-65535, else ValueError)."""
        if not 0 <= value <= 0xFFFF:
            raise ValueError("value must be 0-65535")
        # A CTRL1 write may change the scale and PWM divider the
        # diagnostics cache, a GLOBAL_CTRL write the master PWM frequency
        # (GAIN and SNSF live in CTRL1 only, so a channel switching on or
        # off keeps the scales); forget them.
        if reg_addr == _REG_GLOBAL_CTRL:
            self._pwm_cache.clear()
        elif reg_addr >= _REG_CFG0_CTRL1 and (reg_addr - _REG_CFG0_CTRL1) % _CH_STRIDE == 0:
            self._scale_cache.clear()
            self._kint_cache.clear()
            self._pwm_cache.clear()
        if reg_addr in (_REG_F_AC, _REG_U_AC_SCAN):
            self._ac_cache = None
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
        limit = bitfield[1] >> bitfield[2]
        if not 0 <= val <= limit:
            raise ValueError(f"bitfield value must be 0-{limit}")
        if "w1c" in bitfield[3]:
            # Write-1-to-clear: write only the target bit(s), don't
            # read-modify-write (that would clear other latched faults).
            self.write_register(bitfield[0], val << bitfield[2])
        else:
            register = self.read_register(bitfield[0]) & ~bitfield[1] | (
                val << bitfield[2]
            )
            self.write_register(bitfield[0], register)

    # ----- register 0x00: global control

    @property
    def global_control(self) -> int:
        return self.read_register(_REG_GLOBAL_CTRL)

    @property
    def master_chopping_frequency(self) -> int:
        return self._read_bitfield(_BITFIELDS["F_PWM_M"])

    @property
    def f_pwm_m(self) -> int:
        return self._read_bitfield(_BITFIELDS["F_PWM_M"])

    @f_pwm_m.setter
    def f_pwm_m(self, value: int) -> None:
        # 0x0-0xC: 100 kHz to 2.5 kHz (Table 4); 0xD-0xF are undefined
        if not 0 <= value <= 12:
            raise ValueError("f_pwm_m must be 0-12")
        self._write_bitfield(_BITFIELDS["F_PWM_M"], value)

    # ----- register 0x01: global configuration

    @property
    def global_config(self) -> int:
        return self.read_register(_REG_GLOBAL_CFG)

    # ----- register 0x02: status (live, not latched)

    @property
    def status(self) -> int:
        return self.read_register(_REG_STATUS)

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
        if not 0 <= value <= 0x0FFF:
            raise ValueError("f_ac must be 0-4095")
        self.write_register(_REG_F_AC, value)

    # ----- register 0x08: U_AC_SCAN (AC amplitude)

    @property
    def u_ac_scan(self) -> int:
        return self.read_register(_REG_U_AC_SCAN) & 0x7FFF

    @u_ac_scan.setter
    def u_ac_scan(self, value: int) -> None:
        if not 0 <= value <= 0x7FFF:
            raise ValueError("u_ac_scan must be 0-32767")
        self.write_register(_REG_U_AC_SCAN, value)

    # ----- register 0x65: fault log 0

    @property
    def fault0(self) -> int:
        return self.read_register(_REG_FAULT0)

    def fault0_clear(self) -> None:
        self._write_fault_clear(_REG_FAULT0)

    # ----- register 0x66: fault log 1

    @property
    def fault1(self) -> int:
        return self.read_register(_REG_FAULT1)

    def fault1_clear(self) -> None:
        self._write_fault_clear(_REG_FAULT1)

    def fault_clear_channel(self, channel: int) -> None:
        """Write-1-to-clear the latched faults of one channel only.

        FAULT0 holds OCP, HHF, OLF and DPM per channel and FAULT1 IND and
        RES; the other channels' bits and the device-wide UVM, COMER and
        OVT stay latched.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        self._write_fault_clear(_REG_FAULT0, 0x1111 << channel)
        self._write_fault_clear(_REG_FAULT1, (1 << channel) | (1 << (7 + channel)))

    def _write_fault_clear(self, reg: int, mask: int = 0xFFFF) -> None:
        """Write-1-to-clear a fault register, ignoring the COMER status.

        The SPI status byte returned with a frame describes the state
        *before* that frame took effect, so clearing a latched COMER always
        reports COMER once more. A device whose supply was just switched on
        latches COMER at power-on, and a caller that treated that report as
        a failure abandoned its enable sequence part-way. The write itself
        went out; a persistent fault shows up on the next read.
        """
        try:
            self.write_register(reg, mask)
        except MAX22216SPIError:
            pass

    # ----- per-channel: RES_THLD (resistance threshold)

    def res_thld(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return self.read_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE))

    def set_res_thld(self, channel: int, value: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        _check_range("res_thld", value, 0xFFFF)
        self.write_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE), value)

    # ----- per-channel: IAC_THLD (inductance/I_AC threshold)

    def iac_thld(self, channel: int) -> int:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return self.read_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE)) & 0x0FFF

    def set_iac_thld(self, channel: int, value: int) -> None:
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        _check_range("iac_thld", value, 0x0FFF)
        self.write_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE), value)

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

    def ma_to_dpm_start(self, ma: float, channel: int = 0) -> int:
        """Convert DPM start current in mA to DPM_START register value.

        DPM_START is the current level above which the DPM algorithm
        begins searching for the BEMF plunger-movement dip. Per
        datasheet page 38:

            DPM_START(mA) = 64 × K_CDR × GAIN × SNSF × DPM_START[7:0]

        Returns a value in range [1, 255] (never 0: setting 0 would
        trigger immediate search on power-up and produce spurious
        results during the pre-ramp settling period).
        """
        scale = self._channel_scale(channel)
        return max(1, min(255, int(ma / (64 * scale * self.K_CDR))))

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
        Both affect all current-related diagnostics. Cached per channel
        until CTRL1 is written, since only a configuration write changes
        it.
        """
        cached = self._scale_cache.get(channel)
        if cached is not None:
            return cached
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
        scale = snsf * gain
        self._scale_cache[channel] = scale
        return scale

    def pwm_freq_khz(self, channel: int) -> float:
        """Return the actual PWM frequency for a channel in kHz.

        Reads F_PWM_M (master) from GLOBAL_CTRL and F_PWM (divider)
        from the channel's CTRL1 register; cached per channel until
        either register is written.
        """
        cached = self._pwm_cache.get(channel)
        if cached is not None:
            return cached
        master_reg = self._read_bitfield(_BITFIELDS["F_PWM_M"])
        if master_reg >= len(self._PWM_FREQ_KHZ):
            master_reg = 0  # 0xD/0xE/0xF alias to 100kHz
        master_khz = self._PWM_FREQ_KHZ[master_reg]
        ctrl1 = self.read_register(
            _REG_CFG0_CTRL1 + (channel * _CH_STRIDE)
        )
        div_reg = (ctrl1 >> 8) & 0x03
        khz = master_khz / self._PWM_DIV[div_reg]
        self._pwm_cache[channel] = khz
        return khz

    def master_pwm_khz(self) -> float:
        """Return the master PWM frequency F_PWM_M in kHz (cached until
        GLOBAL_CTRL is written)."""
        cached = self._pwm_cache.get("m")
        if cached is not None:
            return cached
        master_reg = self._read_bitfield(_BITFIELDS["F_PWM_M"])
        if master_reg >= len(self._PWM_FREQ_KHZ):
            master_reg = 0  # 0xD/0xE/0xF alias to 100kHz
        khz = self._PWM_FREQ_KHZ[master_reg]
        self._pwm_cache["m"] = khz
        return khz

    def pwm_counts_to_ms(self, counts: int, channel: int) -> float | None:
        """Convert a count of channel PWM periods (TIME_L2H, TRAVEL_TIME,
        REACTION_TIME) to milliseconds; None without a PWM frequency."""
        f_khz = self.pwm_freq_khz(channel)
        return counts / f_khz if f_khz else None

    def ms_to_pwm_counts(self, ms: float, channel: int) -> int:
        """Convert milliseconds to channel PWM periods (TIME_L2H), clamped
        to the 16-bit register."""
        return max(0, min(0xFFFF, int(ms * self.pwm_freq_khz(channel) + 0.5)))

    def ma_to_iac_thld(self, ma: float, channel: int = 0) -> int:
        """Convert an AC current in mA to IAC_THLD register codes.

        IAC_THLD has the scale of I_AC: K_CDR x GAIN x SNSF per code, so
        the channel's GAIN and SNSF settings apply. Clamped to 12 bits.
        """
        scale = self._channel_scale(channel)
        return max(0, min(0x0FFF, int(ma / (scale * self.K_CDR) + 0.5)))

    def register_file(self, first: int = _REG_GLOBAL_CTRL, last: int = _REG_CFG3_I) -> bytearray:
        """Read registers ``first`` to ``last`` (inclusive) and return them
        as consecutive little-endian 16-bit words.

        The default range is the global and per-channel configuration
        registers (0x00 to 0x40).
        """
        out = bytearray(2 * (last - first + 1))
        for i in range(last - first + 1):
            val = self.read_register(first + i)
            out[2 * i] = val & 0xFF
            out[2 * i + 1] = val >> 8
        return out

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
        # 0x0-0x8 (Table 1); 0x9-0xF are undefined
        _check_range("chs", chs, 8)
        # Single read-modify-write: CHS + VDR + fault mask + ACTIVE
        reg = self.global_config
        reg &= 0x0060  # Preserve reserved[5], STAT_POL[6], CNTL_POL[7]
        reg |= chs  # CHS[3:0]
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
        ramp: int | None = None,
        rupe: bool | None = None,
        rdwe: bool | None = None,
    ) -> None:
        """Configure a channel's sequencer, DPM, and inductance settings.

        The sine-wave generator's frequency and amplitude are one register
        each per device, set with the ``f_ac`` and ``u_ac_scan`` properties.

        Args:
            channel: Channel 0-3.
            dc_h: Hold duty cycle (16-bit raw). Applied after excitation.
            dc_l2h: Excitation (hit) duty cycle (16-bit raw). Initial pull-in.
            time_l2h: Excitation duration (16-bit raw PWM cycles).
            dpm_enable: Enable plunger movement detection during excitation.
            l_meas_h: Enable inductance measurement during hold phase.
            dc_l: Off-phase duty cycle (16-bit raw); None=keep current.
            res_thld: Resistance fault threshold (16-bit raw); None=keep current.
            iac_thld: AC current fault threshold (12-bit raw); None=keep current.
            dpm_thld: DPM current delta threshold (12-bit); None=chip default.
            dpm_start: DPM start current level (8-bit, ~65 mA per unit at default
                scaling); None=chip default.
            dpm_min_nbr: DPM deglitch count (4-bit); None=chip default.
            end_hit_auto: Auto hit→hold on DPM detection; None=keep current.
            end_hit_to_hiz_auto: Auto hit→HiZ on DPM detection; None=keep current.
            ol_en: Open-load detection enable (default True).
            l_meas_l2h: Enable inductance measurement during hit phase.
            l_meas_wcycles: Inductance measurement wait cycles (0-15); None=chip default.
            l_nbr_calc: Inductance measurement averaging periods (0-15); None=chip default.
            ramp: Ramp slew rate value for CTRL0 (8-bit); None=0.
            rupe: Ramp-up enable; None=off.
            rdwe: Ramp-down enable; None=off.

        Every value is checked before the first write: one out of range
        raises ValueError and the channel keeps its configuration.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        _check_range("dc_h", dc_h, 0xFFFF)
        _check_range("dc_l2h", dc_l2h, 0xFFFF)
        _check_range("time_l2h", time_l2h, 0xFFFF)
        for name, value, high in (
            ("dc_l", dc_l, 0xFFFF),
            ("res_thld", res_thld, 0xFFFF),
            ("iac_thld", iac_thld, 0x0FFF),
            ("dpm_thld", dpm_thld, 0x0FFF),
            ("dpm_start", dpm_start, 0xFF),
            ("dpm_min_nbr", dpm_min_nbr, 0x0F),
            ("l_meas_wcycles", l_meas_wcycles, 0x0F),
            ("l_nbr_calc", l_nbr_calc, 0x0F),
            ("ramp", ramp, 0xFF),
        ):
            if value is not None:
                _check_range(name, value, high)
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
        else:
            # DPM_EN persists until the device is reset, so it is written
            # both ways: a configuration with DPM off used to keep faulting
            # at the last threshold and silently block the channel.
            self.dpm_disable(channel)
        # The sine-wave generator (F_AC, U_AC_SCAN: one per device) runs
        # during DC_L whenever L_MEAS_EN is set, not only during
        # DC_L2H/DC_H when l_meas_l2h/l_meas_h are enabled. L_MEAS_EN
        # itself is toggled by the caller while the channel is driven, to
        # avoid IND faults while idle.
        # Written both ways: these bits were only ever set, so a config
        # that turned a measurement phase off left it running in hardware.
        setattr(self, f"l_meas_h_{channel}", 1 if l_meas_h else 0)
        setattr(self, f"l_meas_l2h_{channel}", 1 if l_meas_l2h else 0)
        if l_meas_wcycles is not None:
            setattr(self, f"l_meas_wcycles_{channel}", l_meas_wcycles)
        if l_nbr_calc is not None:
            setattr(self, f"l_nbr_calc_{channel}", l_nbr_calc)
        if res_thld is not None:
            self.write_register(_REG_CFG0_R_THLD + (channel * _CH_STRIDE), res_thld)
        if iac_thld is not None:
            self.write_register(_REG_CFG0_IND_1 + (channel * _CH_STRIDE), iac_thld)
        self.config_ctrl0(
            channel,
            ol_en=ol_en,
            ramp=0 if ramp is None else ramp,
            rupe=bool(rupe),
            rdwe=bool(rdwe),
        )

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
        _check_range("ctrl_mode", ctrl_mode, 3)
        _check_range("ramp", ramp, 0xFF)
        value = (
            ramp
            | (int(rupe) << 8)
            | (int(rmde) << 9)
            | (int(rdwe) << 10)
            | (int(h2l_en) << 11)
            | (int(ol_en) << 12)
            | (int(hhf_en) << 13)
            | (ctrl_mode << 14)
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
            start: DPM start current level (8-bit, 0-255; ~65 mA per unit at default
                scaling, see ma_to_dpm_start), None=keep default
            min_nbr: Minimum samples before detection valid (4-bit, 0-15), None=keep default
            end_hit_auto: Auto DC_L2H→DC_H switchover on valid BEMF dip detection.
                Cuts hit phase short once plunger movement confirmed, saving power.
            end_hit_to_hiz_auto: Auto DC_L2H→HiZ switchover on valid BEMF dip detection.
                For latching valves that only need a pulse to change state.

        If all parameters are None, just enables DPM with chip defaults.
        """

        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        if thld is not None:
            _check_range("thld", thld, 0x0FFF)
        if start is not None:
            _check_range("start", start, 0xFF)
        if min_nbr is not None:
            _check_range("min_nbr", min_nbr, 0x0F)

        # Configure DPM_THLD in DPM0 register
        if thld is not None:
            self.write_register(_REG_CFG0_DPM0 + (channel * _CH_STRIDE), thld)

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
                dpm1_val = (dpm1_val & ~0x00FF) | start
            if min_nbr is not None:
                dpm1_val = (dpm1_val & ~0x0F00) | (min_nbr << 8)
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

        A value is None, never 0, when there is no measurement behind it,
        as in diagnostics(): a current register held the invalid
        mid-conversion value (0xFF00 and above), or (times) the PWM
        frequency is zero.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _DIAG_STRIDE
        peak = self._read_diag(_REG_DIA0_I_DPM_PEAK + offset)
        valley = self._read_diag(_REG_DIA0_I_DPM_VALLEY + offset)
        travel = self.read_register(_REG_DIA0_TRAVEL_TIME + offset)
        react = self.read_register(_REG_DIA0_REACTION_TIME + offset)
        scale = self._channel_scale(channel)
        k = _DPM_CURRENT_SCALE * scale * self.K_CDR
        f_khz = self.pwm_freq_khz(channel)
        return (
            None if peak is None else round(peak * k, 2),
            None if valley is None else round(valley * k, 2),
            round(travel / f_khz, 2) if f_khz else None,
            round(react / f_khz, 2) if f_khz else None,
        )

    def _read_diag(self, reg: int) -> int | None:
        """Read diagnostic register, None for 0xFF00+ (invalid mid-ADC)."""
        val = self.read_register(reg)
        return None if val >= 0xFF00 else val

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

    def diagnostics(self, channel: int, vm: float | None = None, live_only: bool = False) -> dict:
        """Read fully-scaled diagnostics for a channel.

        Args:
            channel: Channel 0-3.
            vm: The supply voltage in volts, when the caller has just read
                ``vm``. VM is measured once per device, not per channel, so
                a pass over the four channels reads it once and passes it
                in rather than paying the SPI read four times. Read here
                when None.
            live_only: Read only the registers that move while a channel
                is held (monitor current, AC current, PWM duty: three
                reads) and take the event registers (DPM peak/valley,
                travel, reaction, resistance) from the last full read;
                the caller does a full read after each switching edge.

        Returns the channel's dict (one per channel, filled in place, so
        copy it to keep a snapshot) of integers: voltage_mv, peak_ma,
        valley_ma, travel_us, reaction_us, i_monitor_ma (signed), i_ac_ma,
        resistance_mohm, inductance_uh, pwm_duty_permille, and the raw
        counts travel_raw, reaction_raw (PWM periods) and pwm_duty_raw.
        Integers so
        that a periodic read allocates nothing; scale them where they are
        displayed. A field is None, never 0, when there is no measurement
        behind it: its register held the invalid mid-conversion value
        (0xFF00 and above), the PWM frequency or the current scale it
        needs is zero, (resistance) its register reads 0, which is no
        measurement: the chip measures resistance only during an
        inductance measurement, or (inductance) the AC current, F_AC or
        U_AC_SCAN it is computed from is not usable. Any other 0 is a
        reading of 0.

        Current scaling uses SNSF and GAIN read from CTRL1.
        Inductance uses U_AC_SCAN amplitude and F_AC frequency.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        offset = channel * _DIAG_STRIDE
        scale = self._channel_scale(channel)
        k_milli, k_dpm_centi = self._current_scales(channel, scale)
        f_khz = self.pwm_freq_khz(channel)

        out = self._diag_out[channel]
        if out is None:
            out = self._diag_out[channel] = {
                "voltage_mv": None, "peak_ma": None, "valley_ma": None,
                "travel_us": None, "reaction_us": None, "i_monitor_ma": None,
                "i_ac_ma": None, "resistance_mohm": None, "inductance_uh": None,
                "pwm_duty_permille": None, "travel_raw": None, "reaction_raw": None,
                "pwm_duty_raw": None,
            }

        # Every value is an integer (mA, µs, mΩ, µH, per mille, mV) or
        # None: the currents from integer scaling (raw < 0xFF00 times a
        # scale ≤ 32544 centi-mA stays a small int), so a pass allocates
        # neither a dict nor boxed floats. The event registers are read
        # after a switching edge, not per pass, and may spend a float each.
        if not (live_only and self._event_cache.get(channel) is not None):
            peak_raw = self._read_diag(_REG_DIA0_I_DPM_PEAK + offset)
            out["peak_ma"] = (
                None if peak_raw is None else (peak_raw * k_dpm_centi + 50) // 100
            )
            valley_raw = self._read_diag(_REG_DIA0_I_DPM_VALLEY + offset)
            out["valley_ma"] = (
                None if valley_raw is None else (valley_raw * k_dpm_centi + 50) // 100
            )
            # Timer registers are PWM counters, not ADC values: no _read_diag filter
            travel_raw = out["travel_raw"] = self.read_register(_REG_DIA0_TRAVEL_TIME + offset)
            out["travel_us"] = int(travel_raw * 1000 / f_khz + 0.5) if f_khz else None
            react_raw = out["reaction_raw"] = self.read_register(_REG_DIA0_REACTION_TIME + offset)
            out["reaction_us"] = int(react_raw * 1000 / f_khz + 0.5) if f_khz else None
            # Resistance: R(mΩ) = raw × K_R / (SNSF × GAIN). The chip
            # measures it only during an inductance measurement (average V
            # over average I); a code of 0 is no measurement, not 0 ohm.
            res_raw = self._read_diag(_REG_DIA0_RES + offset)
            out["resistance_mohm"] = (
                int(res_raw * (self.K_R / scale) * 1000 + 0.5)
                if scale and res_raw else None
            )
            self._event_cache[channel] = True
        # I_MONITOR is signed (13 bits per channel, the 16-bit sum in
        # parallel configurations), so the 0xFF00 filter of the unsigned
        # registers does not apply: 0xFF00 and above are small negatives.
        mon_raw = self.read_register(_REG_DIA0_I_MONITOR + offset)
        if mon_raw & 0x8000:
            out["i_monitor_ma"] = -(((0x10000 - mon_raw) * k_milli + 500) // 1000)
        else:
            out["i_monitor_ma"] = (mon_raw * k_milli + 500) // 1000
        ac_raw = self._read_diag(_REG_DIA0_I_AC + offset)
        i_ac_ma = out["i_ac_ma"] = (
            None if ac_raw is None else (ac_raw * k_milli + 500) // 1000
        )
        pwm_raw = out["pwm_duty_raw"] = self._read_diag(_REG_DIA0_PWM_DUTY + offset)
        out["pwm_duty_permille"] = (
            None if pwm_raw is None else pwm_raw * 1000 // _PWM_DUTY_MAX
        )

        if vm is None:
            vm = self.vm
        out["voltage_mv"] = int(vm * 1000 + 0.5)

        # Inductance: L = U_AC_SCAN / (2π × F_AC × I_AC)
        # F_AC (Hz) = F_PWM_M (Hz) × (F_AC_SCAN / 65535)
        # U_AC_SCAN voltage = K_VDR × 36 × U_AC_SCAN_reg (VDR mode)
        # All three inputs (I_AC, F_AC, U_AC_SCAN) must be non-zero
        # for the calculation to be meaningful. The two configuration
        # registers are cached until written. None when any input is not
        # usable.
        ind_uh = None
        if i_ac_ma is not None and i_ac_ma > 1:
            if self._ac_cache is None:
                self._ac_cache = (
                    self.read_register(_REG_F_AC) & 0x0FFF,
                    self.read_register(_REG_U_AC_SCAN) & 0x7FFF,
                )
            f_ac_reg, u_ac_reg = self._ac_cache
            # F_AC derives from the master frequency F_PWM_M, not the
            # channel's divided PWM frequency.
            master_khz = self.master_pwm_khz()
            if f_ac_reg > 0 and u_ac_reg > 0:
                f_hz = master_khz * 1000 * (f_ac_reg / _F_AC_DIVISOR)
                u_ac_v = self.K_VDR * 36 * u_ac_reg
                ind_uh = int(
                    (u_ac_v / (_TWO_PI * f_hz * (i_ac_ma / 1000))) * 1_000_000 + 0.5
                )
        out["inductance_uh"] = ind_uh
        return out

    def _current_scales(self, channel: int, scale: float) -> tuple:
        """Integer current scales for a channel: (milli-mA per LSB of the
        monitor/AC registers, centi-mA per LSB of the DPM registers),
        cached with the channel scale."""
        cached = self._kint_cache.get(channel)
        if cached is not None:
            return cached
        k = scale * self.K_CDR
        cached = (int(k * 1000 + 0.5), int(_DPM_CURRENT_SCALE * k * 100 + 0.5))
        self._kint_cache[channel] = cached
        return cached

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

    def all_channels_off(self) -> None:
        """Clear CNTL0-3 in GLOBAL_CTRL (all channels off).

        Preserves F_PWM_M. Used before (re)activation so a CNTL bit left
        set across a disable or a supply loss cannot come back live.
        """
        reg = self.read_register(_REG_GLOBAL_CTRL)
        if reg & 0x000F:
            self.write_register(_REG_GLOBAL_CTRL, reg & ~0x000F)

    def channel_enabled(self, channel: int) -> bool:
        """Return True if the CNTL bit for a channel is set."""
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        return bool(self.read_register(_REG_GLOBAL_CTRL) & (1 << channel))

    # ----- single-channel current monitor

    def i_monitor(self, channel: int) -> float:
        """Read instantaneous current for a channel in mA.

        I_MONITOR(mA) = K_CDR × GAIN × SNSF × I_MONITOR[15:0], signed.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        raw = self.read_register(
            _REG_DIA0_I_MONITOR + channel * _DIAG_STRIDE
        )
        if raw & 0x8000:
            raw -= 0x10000  # signed: the current can read below zero
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
            snsf: Sense scaling (0=full, 1=2/3, 2=1/3) (MAX22216 only)

        None values leave the current setting unchanged. A value outside
        its field raises ValueError and nothing is written.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        for name, value, high in (
            ("hsnls", hsnls, 1),
            ("f_pwm", f_pwm, 3),
            ("slew_rate", slew_rate, 3),
            ("t_blanking", t_blanking, 3),
            ("gain", gain, 3),
            ("snsf", snsf, 3),
        ):
            if value is not None:
                _check_range(name, value, high)
        reg_addr = _REG_CFG0_CTRL1 + (channel * _CH_STRIDE)
        val = self.read_register(reg_addr)
        if snsf is not None:
            val = (val & ~0x0003) | snsf
        if gain is not None:
            val = (val & ~0x000C) | (gain << 2)
        if slew_rate is not None:
            val = (val & ~0x0030) | (slew_rate << 4)
        if t_blanking is not None:
            val = (val & ~0x00C0) | (t_blanking << 6)
        if f_pwm is not None:
            val = (val & ~0x0300) | (f_pwm << 8)
        if hsnls is not None:
            val = (val & ~0x0400) | (hsnls << 10)
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

        A gain outside the range raises ValueError and nothing is written.
        """
        if not 0 <= channel <= 3:
            raise ValueError("channel must be 0-3")
        kp_raw = int(kp * 256)
        ki_raw = int(ki * 256)
        if not 0 <= kp_raw <= 0xFFFF:
            raise ValueError("kp must be 0-255.996")
        if not 0 <= ki_raw <= 0xFFFF:
            raise ValueError("ki must be 0-255.996")
        offset = channel * _CH_STRIDE
        self.write_register(_REG_CFG0_P + offset, kp_raw)
        self.write_register(_REG_CFG0_I + offset, ki_raw)

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
        print("--- Faults ---")
        print(f"  0x65 FAULT0         = 0x{f0:04X}")
        print(f"  0x66 FAULT1         = 0x{f1:04X}")

    def __repr__(self) -> str:
        return (
            f"<{self._device_variant}"
            f" active={bool(self.global_config & 0x8000)}"
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
