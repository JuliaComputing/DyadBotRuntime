module ADS7142

using ..I2C
using FieldFlags

"""
    Registers

FieldFlags.jl bit-layout descriptions and constants for the registers of the
Texas Instruments ADS7142 — 12-bit nanopower dual-channel programmable sensor
monitor with an I²C interface, digital window comparator, FIFO data buffer,
and per-channel accumulator (for "16-bit" high-precision mode).

Conventions
-----------
* All structs follow FieldFlags.jl's LSB-first layout: the FIRST field
  written occupies the LEAST significant bit. This matches how the datasheet
  numbers individual bits within each register.
* `_` / `_:n` is reserved/padding, exactly as defined in the datasheet.
* All ADS7142 registers are 8 bits wide; FieldFlags-generated structs therefore
  pack into `UInt8`. Threshold pairs (LSB/MSB) and accumulator pairs are 8-bit
  registers that combine into a 12- or 16-bit value — those are handled with
  block reads/writes and the helper functions below.
* Register reads/writes are not direct I²C bytes; every transaction is prefixed
  by an opcode (Single Register Read/Write, Set Bit, Clear Bit, Block Read,
  Block Write — see `Opcode*` constants and datasheet Table 4).
* Data buffer (FIFO) reads use a different I²C transaction shape: the host
  issues just `[ADDR | R]` with no register prefix and clocks out N × 16-bit
  words (datasheet §7.3.7.2 / Figure 46). Handled by `read_data_buffer!` below.

Reference: ADS7142 datasheet, SBAS773A (Texas Instruments, Dec 2017).
"""
module Registers

using FieldFlags

#=============================================================================
  Opcodes (datasheet Table 4) — first byte of every register-access frame
=============================================================================#

const OPCODE_SINGLE_READ  = 0x10  # 00010000b — set up address for a single read
const OPCODE_SINGLE_WRITE = 0x08  # 00001000b — write one register
const OPCODE_SET_BIT      = 0x18  # 00011000b — atomic OR  (1 bits in data set)
const OPCODE_CLEAR_BIT    = 0x20  # 00100000b — atomic AND-NOT (1 bits clear)
const OPCODE_BLOCK_READ   = 0x30  # 00110000b — set up address for block read
const OPCODE_BLOCK_WRITE  = 0x28  # 00101000b — write multiple consecutive regs

#=============================================================================
  I²C General-Call commands (datasheet §7.3.10.1-3)
=============================================================================#

# Sent at the 7-bit general-call address 0x00. Not used directly by this
# driver — exposed for callers that open a separate I2C handle at 0x00.
const GENERAL_CALL_ADDR          = 0x00
const GENERAL_CALL_SW_RESET      = 0x06  # full reset (clears latched flags)
const GENERAL_CALL_PROGRAM_ADDR  = 0x04  # latch ADDR-pin resistor selection

# High-Speed mode master codes (datasheet §7.3.10.4). Any one of 0x08..0x0F
# acts as the HS-mode entry code; the device sets HS_MODE in
# OPMODE_I2CMODE_STATUS and stays in HS mode until the next STOP.
const HS_MODE_MASTER_CODE = 0x08

#=============================================================================
  WKEY @ 0x17  (W, reset = 0x00, but NOT reset by DEVICE_RESET)
=============================================================================#

# WKEY: write 0xA (1010b) into bits [3:0] to enable writes to DEVICE_RESET.
# The datasheet specifically calls out that WKEY is NOT cleared by software
# reset; the driver therefore writes 0x00 to WKEY after every reset to prevent
# subsequent erroneous DEVICE_RESET writes.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 3:0   | KEYWORD        | write 1010b (0xA) to unlock DEVICE_RESET         |
# | 7:4   | (reserved)     |                                                  |
@bitfield struct WKEY
    KEYWORD:4
    _:4
end

const WKEY_UNLOCK = 0x0A  # KEYWORD value that arms DEVICE_RESET

#=============================================================================
  DEVICE_RESET @ 0x14  (W, reset = 0x00)
=============================================================================#

# Writing 1 to DEV_RST resets all configurations except WKEY and the digital
# window comparator latched flags. The device does NOT re-run offset
# calibration and does NOT re-evaluate its I²C address (datasheet §7.4.1).
# For a full reset (incl. offset cal + address re-evaluation), use a power
# cycle or the General Call reset (00h, 06h).
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 0     | DEV_RST        | write 1 → soft reset (write-only)                |
# | 7:1   | (reserved)     |                                                  |

#=============================================================================
  OFFSET_CAL @ 0x15  (W, reset = 0x00)
=============================================================================#

# Writing 1 to TRIG_OFFCAL starts an internal offset calibration cycle. The
# sampling switches are opened during cal and BUSY/RDY goes high. The device
# auto-calibrates at power-up; periodic recalibration is recommended to track
# temperature and AVDD drift (datasheet §7.3.2).

#=============================================================================
  OPMODE_SEL @ 0x1C  (R/W, reset = 0x00)
=============================================================================#

# OPMODE_SEL: top-level functional-mode selector (datasheet §7.4 + §7.6.2.2).
# Auto-sequence variants scan the channels enabled in AUTO_SEQ_CHEN.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 2:0   | SEL_OPMODE     | 000/001 Manual CH0 only; 100/101 Manual + AUTO;  |
# |       |                | 110 Autonomous + AUTO; 111 High Precision + AUTO |
# | 7:3   | (reserved)     |                                                  |
@bitfield struct OPMODE_SEL
    SEL_OPMODE:3
    _:5
end

# Values for SEL_OPMODE.
@enum OperationMode::UInt8 begin
    OPMODE_MANUAL_CH0      = 0b000  # power-up default
    OPMODE_MANUAL_AUTO     = 0b100  # manual with AUTO sequencing
    OPMODE_AUTONOMOUS_AUTO = 0b110  # autonomous monitoring with AUTO sequencing
    OPMODE_HIGH_PRECISION  = 0b111  # 16-bit-effective accumulation mode
end

#=============================================================================
  OPMODE_I2CMODE_STATUS @ 0x00  (R, reset = 0x00)
=============================================================================#

# Read-only mirror of the present operating mode and the High-Speed I²C state.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 1:0   | DEV_OPMODE     | 00 Manual; 10 Autonomous; 11 High Precision      |
# | 2     | HS_MODE        | 1 = High-Speed I²C mode latched                  |
# | 7:3   | (reserved)     |                                                  |
@bitfield struct OPMODE_I2CMODE_STATUS
    DEV_OPMODE:2
    HS_MODE:1
    _:5
end

@enum DeviceOpMode::UInt8 begin
    DEVMODE_MANUAL         = 0b00
    DEVMODE_AUTONOMOUS     = 0b10
    DEVMODE_HIGH_PRECISION = 0b11
end

#=============================================================================
  CHANNEL_INPUT_CFG @ 0x24  (R/W, reset = 0x00)
=============================================================================#

# Selects how AINP/AIN0 and AINM/AIN1 are wired into the SAR (datasheet
# §7.3.1). Values 00 and 11 both select two-channel single-ended; the device
# powers up in this mode.
#
# | bits  | name              | meaning                                       |
# |-------|-------------------|-----------------------------------------------|
# | 1:0   | CH0_CH1_IP_CFG    | input-mux configuration (see InputConfig)     |
# | 7:2   | (reserved)        |                                               |
@bitfield struct CHANNEL_INPUT_CFG
    CH0_CH1_IP_CFG:2
    _:6
end

@enum InputConfig::UInt8 begin
    TWO_CH_SE            = 0b00  # default — CH0 and CH1 are independent SE inputs
    SINGLE_CH_SE_REMOTEGND = 0b01  # CH0 input, CH1 used as remote ground
    SINGLE_CH_PSEUDO_DIFF  = 0b10  # CH0 swings around CH1 ≈ AVDD/2
    TWO_CH_SE_ALT          = 0b11  # alias for TWO_CH_SE
end

#=============================================================================
  AUTO_SEQ_CHEN @ 0x20  (R/W, reset = 0x03 — both channels enabled)
=============================================================================#

# AUTO_SEQ_CHEN: per-channel enables for AUTO sequencing. Selecting CH1 while
# CH1 is being used as a remote-ground or pseudo-differential reference will
# set SEQ_ERR_ST in SEQUENCE_STATUS (datasheet §7.3.1.2/3).
#
# | bits  | name              | meaning                                       |
# |-------|-------------------|-----------------------------------------------|
# | 0     | AUTOSEQ_EN_CH0    | 1 = CH0 scanned                               |
# | 1     | AUTOSEQ_EN_CH1    | 1 = CH1 scanned                               |
# | 7:2   | (reserved)        |                                               |
@bitfield struct AUTO_SEQ_CHEN
    AUTOSEQ_EN_CH0:1
    AUTOSEQ_EN_CH1:1
    _:6
end

#=============================================================================
  START_SEQUENCE @ 0x1E  /  ABORT_SEQUENCE @ 0x1F  (W, reset = 0x00)
=============================================================================#

# Single-bit one-shot triggers. SEQ_START raises BUSY/RDY and kicks off the
# configured AUTO sequence; SEQ_ABORT aborts the current sequence and lowers
# BUSY/RDY. The data buffer and accumulator status registers are cleared
# whenever SEQ_START is set.

#=============================================================================
  SEQUENCE_STATUS @ 0x04  (R, reset = 0x00)
=============================================================================#

# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 0     | (reserved)     |                                                  |
# | 2:1   | SEQ_ERR_ST     | 00 disabled; 01 enabled OK; 11 enabled w/ error  |
# | 7:3   | (reserved)     |                                                  |
@bitfield struct SEQUENCE_STATUS
    _:1
    SEQ_ERR_ST:2
    _:5
end

@enum SequenceErrorState::UInt8 begin
    SEQ_ERR_DISABLED = 0b00
    SEQ_ERR_OK       = 0b01
    SEQ_ERR_ERROR    = 0b11
end

#=============================================================================
  OSC_SEL @ 0x18  /  nCLK_SEL @ 0x19  (R/W, reset = 0x00)
=============================================================================#

# OSC_SEL: choose between the High-Speed (~50 ns period) and Low-Power
# (~95-300 µs period) internal oscillators (datasheet §7.3.5).
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 0     | HSZ_LP         | 0 = High Speed, 1 = Low Power                    |
# | 7:1   | (reserved)     |                                                  |
@bitfield struct OSC_SEL
    HSZ_LP:1
    _:7
end

@enum Oscillator::UInt8 begin
    OSC_HIGH_SPEED = 0
    OSC_LOW_POWER  = 1
end

# nCLK_SEL: number of oscillator clocks per conversion cycle, full 8-bit
# value. Writing values below the floor saturates to the floor on-chip
# (21 for HS, 18 for LP — see datasheet §7.6.5.2).
const NCLK_FLOOR_HS = UInt8(21)
const NCLK_FLOOR_LP = UInt8(18)

#=============================================================================
  DATA_BUFFER_OPMODE @ 0x2C  (R/W, reset = 0x01 — Start Burst)
=============================================================================#

# Controls how the 16×16-bit FIFO is filled during AUTONOMOUS modes
# (datasheet §7.4.3 / §7.6.6.1).
#
# | bits  | name              | meaning                                       |
# |-------|-------------------|-----------------------------------------------|
# | 2:0   | STARTSTOP_CNTRL   | 000 Stop Burst; 001 Start Burst (default);    |
# |       |                   | 100 Pre-Alert; 110 Post-Alert                 |
# | 7:3   | (reserved)        |                                               |
@bitfield struct DATA_BUFFER_OPMODE
    STARTSTOP_CNTRL:3
    _:5
end

@enum DataBufferMode::UInt8 begin
    DBUF_STOP_BURST  = 0b000  # writes data until SEQ_ABORT is set
    DBUF_START_BURST = 0b001  # writes 16 samples after SEQ_START, then stops
    DBUF_PRE_ALERT   = 0b100  # writes continuously until alert; keeps last 16
    DBUF_POST_ALERT  = 0b110  # waits for alert, then captures 16 samples
end

#=============================================================================
  DOUT_FORMAT_CFG @ 0x28  (R/W, reset = 0x00)
=============================================================================#

# Sets how the 12-bit conversion result is packed into each 16-bit FIFO word.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 1:0   | DOUT_FORMAT    | 00/11 code<<4; 01 code|chid<<1; 10 code|chid<<1| |
# |       |                | DATA_VALID                                       |
# | 7:2   | (reserved)     |                                                  |
@bitfield struct DOUT_FORMAT_CFG
    DOUT_FORMAT:2
    _:6
end

@enum DataOutFormat::UInt8 begin
    DOUT_12BIT_ZEROS      = 0b00  # code<<4, low nibble = 0
    DOUT_12BIT_CHID       = 0b01  # code<<4 | chid<<1
    DOUT_12BIT_CHID_VALID = 0b10  # code<<4 | chid<<1 | DATA_VALID
    DOUT_12BIT_ZEROS_ALT  = 0b11  # alias for DOUT_12BIT_ZEROS
end

#=============================================================================
  DATA_BUFFER_STATUS @ 0x01  (R, reset = 0x00)
=============================================================================#

# Snapshot of how many FIFO entries the device has populated since the last
# SEQ_START. Caps at 16; cleared on power-up, reset, general-call reset, or
# SEQ_START.
#
# | bits  | name              | meaning                                       |
# |-------|-------------------|-----------------------------------------------|
# | 4:0   | DATA_WORDCOUNT    | 0..16 entries filled                          |
# | 7:5   | (reserved)        |                                               |
@bitfield struct DATA_BUFFER_STATUS
    DATA_WORDCOUNT:5
    _:3
end

#=============================================================================
  ACC_EN @ 0x30  (R/W, reset = 0x00)
=============================================================================#

# Accumulator master enable. Per the datasheet, only 0000 (disabled) and 1111
# (enabled) are legal — the intermediate codes are reserved.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 3:0   | EN_ACC         | 0000 disabled, 1111 enabled (Reserved otherwise) |
# | 7:4   | (reserved)     |                                                  |
@bitfield struct ACC_EN
    EN_ACC:4
    _:4
end

const ACC_EN_DISABLE = UInt8(0b0000)
const ACC_EN_ENABLE  = UInt8(0b1111)

#=============================================================================
  ACCUMULATOR_STATUS @ 0x02  (R, reset = 0x00)
=============================================================================#

# Number of 12-bit conversions accumulated since the last SEQ_START. Caps at
# 16 (in High Precision Mode, BUSY/RDY drops when this reaches 16 — datasheet
# §7.4.4).
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 3:0   | ACC_COUNT      | 0..16 accumulations                              |
# | 7:4   | (reserved)     |                                                  |
@bitfield struct ACCUMULATOR_STATUS
    ACC_COUNT:4
    _:4
end

#=============================================================================
  Digital Window Comparator registers (DWC_*, datasheet §7.3.9)

  Thresholds are 12-bit values split across two consecutive 8-bit registers:
  the LSB register holds bits [7:0]; the MSB register holds bits [11:8] in
  its low nibble (bits [7:4] are reserved). Hysteresis is a 6-bit value in
  the low bits of its register.

  The LSB-then-MSB ordering matches the natural ascending order of register
  addresses, so two-byte block writes (LSB at low addr, MSB at high addr)
  work nicely with the BLOCK_WRITE opcode.
=============================================================================#

# DWC_HTH_CHx_MSB / DWC_LTH_CHx_MSB: 4 LSBs of the register are the 4 MSBs of
# the 12-bit threshold.
@bitfield struct DWC_THRESHOLD_MSB
    THRESHOLD_HI:4
    _:4
end

# DWC_HYS_CHx: 6-bit hysteresis value (datasheet §7.6.8.7/12).
@bitfield struct DWC_HYS
    HYS:6
    _:2
end

#=============================================================================
  ALERT control / status (datasheet §7.3.9, §7.6.8.1/2)
=============================================================================#

# ALERT_DWC_EN: master enable for the entire digital-window-comparator block.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 0     | DWC_BLOCK_EN   | 1 = comparator enabled                           |
# | 7:1   | (reserved)     |                                                  |
@bitfield struct ALERT_DWC_EN
    DWC_BLOCK_EN:1
    _:7
end

# ALERT_CHEN: per-channel alert enables.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 0     | ALERT_EN_CH0   | 1 = CH0 contributes to ALERT pin                 |
# | 1     | ALERT_EN_CH1   | 1 = CH1 contributes to ALERT pin                 |
# | 7:2   | (reserved)     |                                                  |
@bitfield struct ALERT_CHEN
    ALERT_EN_CH0:1
    ALERT_EN_CH1:1
    _:6
end

# PRE_ALT_MAX_EVENT_COUNT: how many *consecutive* threshold-crossing
# conversions are needed before the ALERT latches (datasheet Figure 48 /
# §7.3.9). Effective count = PREALERT_COUNT[3:0] + 1.
#
# | bits  | name           | meaning                                          |
# |-------|----------------|--------------------------------------------------|
# | 3:0   | (reserved)     |                                                  |
# | 7:4   | PREALERT_COUNT | event-count threshold minus 1                    |
@bitfield struct PRE_ALT_MAX_EVENT_COUNT
    _:4
    PREALERT_COUNT:4
end

# ALERT_LOW_FLAGS / ALERT_HIGH_FLAGS: latched per-channel alert flags. Write 1
# to a bit to clear it; write 0xFF to clear all flags at once.
#
# | bits  | name             | meaning                                        |
# |-------|------------------|------------------------------------------------|
# | 0     | ALERT_CH0        | latched alert flag for CH0                     |
# | 1     | ALERT_CH1        | latched alert flag for CH1                     |
# | 7:2   | (reserved)       |                                                |
@bitfield struct ALERT_FLAGS
    ALERT_CH0:1
    ALERT_CH1:1
    _:6
end

# ALERT_TRIG_CHID: read-only — which channel first triggered the alert in the
# current sequence. Cleared on reset / SEQ_START.
#
# | bits  | name              | meaning                                       |
# |-------|-------------------|-----------------------------------------------|
# | 3:0   | (reserved)        |                                               |
# | 7:4   | ALERT_TRIG_CHID   | 0 = CH0, 1 = CH1                              |
@bitfield struct ALERT_TRIG_CHID_REG
    _:4
    ALERT_TRIG_CHID:4
end

#=============================================================================
  Register addresses (datasheet Table 5)
=============================================================================#

const OPMODE_I2CMODE_STATUS_ADDR  = 0x00
const DATA_BUFFER_STATUS_ADDR     = 0x01
const ACCUMULATOR_STATUS_ADDR     = 0x02
const ALERT_TRIG_CHID_ADDR        = 0x03
const SEQUENCE_STATUS_ADDR        = 0x04
const ACC_CH0_LSB_ADDR            = 0x08
const ACC_CH0_MSB_ADDR            = 0x09
const ACC_CH1_LSB_ADDR            = 0x0A
const ACC_CH1_MSB_ADDR            = 0x0B
const ALERT_LOW_FLAGS_ADDR        = 0x0C
const ALERT_HIGH_FLAGS_ADDR       = 0x0E
const DEVICE_RESET_ADDR           = 0x14
const OFFSET_CAL_ADDR             = 0x15
const WKEY_ADDR                   = 0x17
const OSC_SEL_ADDR                = 0x18
const NCLK_SEL_ADDR               = 0x19
const OPMODE_SEL_ADDR             = 0x1C
const START_SEQUENCE_ADDR         = 0x1E
const ABORT_SEQUENCE_ADDR         = 0x1F
const AUTO_SEQ_CHEN_ADDR          = 0x20
const CHANNEL_INPUT_CFG_ADDR      = 0x24
const DOUT_FORMAT_CFG_ADDR        = 0x28
const DATA_BUFFER_OPMODE_ADDR     = 0x2C
const ACC_EN_ADDR                 = 0x30
const ALERT_CHEN_ADDR             = 0x34
const PRE_ALT_MAX_EVENT_COUNT_ADDR = 0x36
const ALERT_DWC_EN_ADDR           = 0x37
const DWC_HTH_CH0_LSB_ADDR        = 0x38
const DWC_HTH_CH0_MSB_ADDR        = 0x39
const DWC_LTH_CH0_LSB_ADDR        = 0x3A
const DWC_LTH_CH0_MSB_ADDR        = 0x3B
const DWC_HTH_CH1_LSB_ADDR        = 0x3C
const DWC_HTH_CH1_MSB_ADDR        = 0x3D
const DWC_LTH_CH1_LSB_ADDR        = 0x3E
const DWC_LTH_CH1_MSB_ADDR        = 0x3F
const DWC_HYS_CH0_ADDR            = 0x40
const DWC_HYS_CH1_ADDR            = 0x41

#=============================================================================
  I²C 7-bit slave addresses (datasheet Table 2)

  The ADS7142 latches its 7-bit address from resistors on the ADDR pin at
  power-up. R1 ties ADDR to AVDD, R2 ties ADDR to GND. Up to eight choices.
=============================================================================#

const ADDR_R1_0     = 0x1F  # R1 = 0 Ω,    R2 = DNP
const ADDR_R1_11K   = 0x1E  # R1 = 11 kΩ,  R2 = DNP
const ADDR_R1_33K   = 0x1D  # R1 = 33 kΩ,  R2 = DNP
const ADDR_R1_100K  = 0x1C  # R1 = 100 kΩ, R2 = DNP
const ADDR_R2_0     = 0x18  # R1 = DNP,    R2 = 0 Ω or DNP (DEFAULT)
const ADDR_R2_11K   = 0x19  # R1 = DNP,    R2 = 11 kΩ
const ADDR_R2_33K   = 0x1A  # R1 = DNP,    R2 = 33 kΩ
const ADDR_R2_100K  = 0x1B  # R1 = DNP,    R2 = 100 kΩ
const ADDR_DEFAULT  = ADDR_R2_0

#=============================================================================
  ADC code helpers
=============================================================================#

# 12-bit code → fraction of AVDD (single-ended). Pseudo-differential codes use
# a half-scale centre at AVDD/2 — see datasheet Table 1 and Figure 44.
@inline code_to_fraction(code::Integer) = Float64(code & 0xFFF) / 4096.0

# 16-bit accumulated code → fraction of AVDD (High Precision Mode,
# datasheet Eq. 5: 1 LSB = AVDD / 2^16).
@inline acc_to_fraction(acc::Integer) = Float64(acc & 0xFFFF) / 65536.0

# Parse one 16-bit FIFO word according to the active DOUT_FORMAT_CFG. Returns
# `(code::UInt16, channel::UInt8, valid::Bool)`. For the "12-bit + zeros"
# formats the channel ID is taken as CH0 and valid as `true`.
function parse_data_word(word::UInt16, fmt::DataOutFormat)
    code = (word >> 4) & 0x0FFF
    if fmt === DOUT_12BIT_ZEROS || fmt === DOUT_12BIT_ZEROS_ALT
        return (UInt16(code), UInt8(0), true)
    elseif fmt === DOUT_12BIT_CHID
        chid = UInt8((word >> 1) & 0x07)
        return (UInt16(code), chid, true)
    else  # DOUT_12BIT_CHID_VALID
        chid  = UInt8((word >> 1) & 0x07)
        valid = (word & 0x01) != 0
        return (UInt16(code), chid, valid)
    end
end

#=============================================================================
  Exports
=============================================================================#

export
    # FieldFlags struct types
    WKEY, OPMODE_SEL, OPMODE_I2CMODE_STATUS, CHANNEL_INPUT_CFG, AUTO_SEQ_CHEN,
    SEQUENCE_STATUS, OSC_SEL, DATA_BUFFER_OPMODE, DOUT_FORMAT_CFG,
    DATA_BUFFER_STATUS, ACC_EN, ACCUMULATOR_STATUS,
    DWC_THRESHOLD_MSB, DWC_HYS, ALERT_DWC_EN, ALERT_CHEN,
    PRE_ALT_MAX_EVENT_COUNT, ALERT_FLAGS, ALERT_TRIG_CHID_REG,
    # Opcodes
    OPCODE_SINGLE_READ, OPCODE_SINGLE_WRITE,
    OPCODE_SET_BIT, OPCODE_CLEAR_BIT,
    OPCODE_BLOCK_READ, OPCODE_BLOCK_WRITE,
    # General-call constants
    GENERAL_CALL_ADDR, GENERAL_CALL_SW_RESET, GENERAL_CALL_PROGRAM_ADDR,
    HS_MODE_MASTER_CODE,
    # Reset constants
    WKEY_UNLOCK,
    # Register addresses
    OPMODE_I2CMODE_STATUS_ADDR, DATA_BUFFER_STATUS_ADDR,
    ACCUMULATOR_STATUS_ADDR, ALERT_TRIG_CHID_ADDR, SEQUENCE_STATUS_ADDR,
    ACC_CH0_LSB_ADDR, ACC_CH0_MSB_ADDR, ACC_CH1_LSB_ADDR, ACC_CH1_MSB_ADDR,
    ALERT_LOW_FLAGS_ADDR, ALERT_HIGH_FLAGS_ADDR,
    DEVICE_RESET_ADDR, OFFSET_CAL_ADDR, WKEY_ADDR,
    OSC_SEL_ADDR, NCLK_SEL_ADDR, OPMODE_SEL_ADDR,
    START_SEQUENCE_ADDR, ABORT_SEQUENCE_ADDR,
    AUTO_SEQ_CHEN_ADDR, CHANNEL_INPUT_CFG_ADDR,
    DOUT_FORMAT_CFG_ADDR, DATA_BUFFER_OPMODE_ADDR,
    ACC_EN_ADDR, ALERT_CHEN_ADDR, PRE_ALT_MAX_EVENT_COUNT_ADDR,
    ALERT_DWC_EN_ADDR,
    DWC_HTH_CH0_LSB_ADDR, DWC_HTH_CH0_MSB_ADDR,
    DWC_LTH_CH0_LSB_ADDR, DWC_LTH_CH0_MSB_ADDR,
    DWC_HTH_CH1_LSB_ADDR, DWC_HTH_CH1_MSB_ADDR,
    DWC_LTH_CH1_LSB_ADDR, DWC_LTH_CH1_MSB_ADDR,
    DWC_HYS_CH0_ADDR, DWC_HYS_CH1_ADDR,
    # I²C addresses
    ADDR_R1_0, ADDR_R1_11K, ADDR_R1_33K, ADDR_R1_100K,
    ADDR_R2_0, ADDR_R2_11K, ADDR_R2_33K, ADDR_R2_100K, ADDR_DEFAULT,
    # nCLK floors
    NCLK_FLOOR_HS, NCLK_FLOOR_LP,
    # Accumulator enable constants
    ACC_EN_DISABLE, ACC_EN_ENABLE,
    # Enums and members
    OperationMode,
    OPMODE_MANUAL_CH0, OPMODE_MANUAL_AUTO,
    OPMODE_AUTONOMOUS_AUTO, OPMODE_HIGH_PRECISION,
    DeviceOpMode,
    DEVMODE_MANUAL, DEVMODE_AUTONOMOUS, DEVMODE_HIGH_PRECISION,
    InputConfig,
    TWO_CH_SE, SINGLE_CH_SE_REMOTEGND, SINGLE_CH_PSEUDO_DIFF, TWO_CH_SE_ALT,
    SequenceErrorState,
    SEQ_ERR_DISABLED, SEQ_ERR_OK, SEQ_ERR_ERROR,
    Oscillator,
    OSC_HIGH_SPEED, OSC_LOW_POWER,
    DataBufferMode,
    DBUF_STOP_BURST, DBUF_START_BURST, DBUF_PRE_ALERT, DBUF_POST_ALERT,
    DataOutFormat,
    DOUT_12BIT_ZEROS, DOUT_12BIT_CHID, DOUT_12BIT_CHID_VALID,
    DOUT_12BIT_ZEROS_ALT,
    # Helpers
    code_to_fraction, acc_to_fraction, parse_data_word

end # module Registers

using .Registers

#=============================================================================
  Device struct
=============================================================================#

"""
    ADSDevice

Wrapper for a Texas Instruments ADS7142 over I²C.

# Fields
- `device::I2C.I2CDevice`: I²C device handle.
- `dout_fmt::DataOutFormat`: locally-tracked FIFO word format (mirrors
  DOUT_FORMAT_CFG). Used by `read_data_buffer!` to parse FIFO words.
- `tx_buf::Vector{UInt8}`: pre-allocated TX scratch (always 2 bytes for
  single-register or set/clear-bit transactions; grows for block writes).

Unlike the DAC43701, the ADS7142 has dedicated `Set Bit` and `Clear Bit`
opcodes, so the driver doesn't need to cache configuration registers for
atomic RMW — the chip handles that for us.
"""
mutable struct ADSDevice
    device::I2C.I2CDevice
    dout_fmt::DataOutFormat
    tx_buf::Vector{UInt8}
end

"""
    ADSDevice(device::I2C.I2CDevice)

Wrap an already-opened I²C device. The local DOUT_FORMAT_CFG mirror is
initialised to the chip's POR default (`DOUT_12BIT_ZEROS`).
"""
ADSDevice(device::I2C.I2CDevice) = ADSDevice(device, DOUT_12BIT_ZEROS, zeros(UInt8, 2))

"""
    open_ads(bus::Integer, address::Integer = ADDR_DEFAULT) -> ADSDevice

Open `/dev/i2c-<bus>` at the given 7-bit address and return a wrapped
`ADSDevice`. The default address `0x18` matches a board with R2 = 0 Ω (or
R2 = DNP with R1 = DNP) on the ADDR pin.
"""
function open_ads(bus::Integer, address::Integer = ADDR_DEFAULT)
    return ADSDevice(I2C.open_device(Int(bus), UInt8(address)))
end

"""
    close_ads(dev::ADSDevice)

Close the underlying I²C device.
"""
close_ads(dev::ADSDevice) = I2C.close_device(dev.device)

"""
    close!(dev::ADSDevice)

Alias for `close_ads`.
"""
close!(dev::ADSDevice) = close_ads(dev)

#=============================================================================
  Low-level I²C register operations (datasheet §7.3.11)

  Every transaction carries an opcode as the first byte after the device
  address. The I2C wrapper's `write_bytes(dev, opcode, payload)` packs the
  opcode and the payload bytes back-to-back, which matches the ADS7142
  command frame layout.
=============================================================================#

"""
    write_reg(dev::ADSDevice, reg::Integer, value::Integer) -> UInt8

Single-register write (opcode 0x08). On-wire frame:
`[addr|W, 0x08, reg, value]`.
"""
function write_reg(dev::ADSDevice, reg::Integer, value::Integer)
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    dev.tx_buf[2] = UInt8(value & 0xFF)
    I2C.write_bytes(dev.device, OPCODE_SINGLE_WRITE, dev.tx_buf)
    return UInt8(value & 0xFF)
end

"""
    read_reg(dev::ADSDevice, reg::Integer) -> UInt8

Single-register read (opcode 0x10). On-wire:
`[addr|W, 0x10, reg]  then  [addr|R, data, NACK, STOP]`.
"""
function read_reg(dev::ADSDevice, reg::Integer)
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    # `write_bytes(opcode, payload)` → `[opcode, payload...]` on the wire.
    # We need just one payload byte (the register address), so we pass a
    # 1-element view into the scratch buffer.
    I2C.write_bytes(dev.device, OPCODE_SINGLE_READ, view(dev.tx_buf, 1:1))
    return I2C.read_byte(dev.device)
end

"""
    set_bits!(dev::ADSDevice, reg::Integer, mask::Integer) -> UInt8

Atomic OR of `mask` into `reg` (opcode 0x18). Bits set to 1 in `mask` are
set in the target register; 0 bits are left untouched.
"""
function set_bits!(dev::ADSDevice, reg::Integer, mask::Integer)
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    dev.tx_buf[2] = UInt8(mask & 0xFF)
    I2C.write_bytes(dev.device, OPCODE_SET_BIT, dev.tx_buf)
    return UInt8(mask & 0xFF)
end

"""
    clear_bits!(dev::ADSDevice, reg::Integer, mask::Integer) -> UInt8

Atomic AND-NOT of `mask` from `reg` (opcode 0x20). Bits set to 1 in `mask`
are cleared in the target register; 0 bits are left untouched.
"""
function clear_bits!(dev::ADSDevice, reg::Integer, mask::Integer)
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    dev.tx_buf[2] = UInt8(mask & 0xFF)
    I2C.write_bytes(dev.device, OPCODE_CLEAR_BIT, dev.tx_buf)
    return UInt8(mask & 0xFF)
end

"""
    read_block!(dev::ADSDevice, reg::Integer, dest::Vector{UInt8}) -> Int

Read `length(dest)` consecutive register bytes starting at `reg` (opcode 0x30).
Addresses beyond the register map return zero. Returns the number of bytes
read. Restricted to `Vector{UInt8}` so the data phase can hit the raw `read()`
syscall without an intermediate copy.
"""
function read_block!(dev::ADSDevice, reg::Integer, dest::Vector{UInt8})
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    I2C.write_bytes(dev.device, OPCODE_BLOCK_READ, view(dev.tx_buf, 1:1))
    # Data phase is a plain `[addr|R, byte0, byte1, ..., NACK, STOP]` with no
    # register prefix — exactly what the raw read() syscall in I2C.i2c_read
    # produces.
    return I2C.i2c_read(dev.device, dest)
end

"""
    write_block(dev::ADSDevice, reg::Integer, data::AbstractVector{UInt8}) -> Int

Write `data` to consecutive registers starting at `reg` (opcode 0x28).
Writing to addresses outside the register map is silently ignored by the
device (datasheet §7.3.11.2.4).
"""
function write_block(dev::ADSDevice, reg::Integer, data::AbstractVector{UInt8})
    n = length(data)
    # Grow the scratch buffer if needed (most calls fit in 2 bytes).
    if n + 1 > length(dev.tx_buf)
        resize!(dev.tx_buf, n + 1)
    end
    dev.tx_buf[1] = UInt8(reg & 0xFF)
    @inbounds for i in 1:n
        dev.tx_buf[i + 1] = UInt8(data[i] & 0xFF)
    end
    return I2C.write_bytes(dev.device, OPCODE_BLOCK_WRITE, view(dev.tx_buf, 1:(n + 1)))
end

"""
    read_data_buffer!(dev::ADSDevice, dest::Vector{UInt8}) -> Int

Read raw bytes from the FIFO data buffer (no register prefix; datasheet
§7.3.7.2 / Figure 46). The FIFO holds up to sixteen 16-bit words, so up to 32
bytes can be read in one transaction. Each entry arrives MSB-first.

If you call this while a sequence is still active (`BUSY/RDY` high) the
device returns zeroes with the DATA_VALID flag clear.

For decoded samples, see `read_data_buffer(dev; count)`.
"""
read_data_buffer!(dev::ADSDevice, dest::Vector{UInt8}) =
    I2C.i2c_read(dev.device, dest)

"""
    read_manual_samples!(dev::ADSDevice, dest::Vector{UInt8}) -> Int

Read raw bytes during Manual Mode conversion (datasheet §7.4.2 / Figure 56).
Each sample is 16 bits, MSB-first: bits [15:4] = 12-bit code, bits [3:0] = 0.
On-wire shape is identical to a data-buffer read — `dest` must be sized to
`2 * num_samples`.
"""
read_manual_samples!(dev::ADSDevice, dest::Vector{UInt8}) =
    I2C.i2c_read(dev.device, dest)

#=============================================================================
  FieldFlags ↔ UInt8 helpers
=============================================================================#

@inline as_byte(x) = reinterpret(UInt8, getfield(x, :fields))

#=============================================================================
  Probe / status
=============================================================================#

"""
    probe(dev::ADSDevice) -> Bool

Verify the device responds on the bus by reading the OPMODE_I2CMODE_STATUS
register. Returns `true` on success; on a NAK the underlying `I2C` layer
throws (see `I2C.i2c_error`).

The ADS7142 has no dedicated device-ID register, so this only confirms that
*something* on the configured 7-bit address ACKed an opcode-prefixed read —
not the silicon identity.
"""
function probe(dev::ADSDevice)
    read_reg(dev, OPMODE_I2CMODE_STATUS_ADDR)
    return true
end

"""
    read_opmode_status(dev::ADSDevice) -> OPMODE_I2CMODE_STATUS

Decode the OPMODE_I2CMODE_STATUS register (0x00).
"""
read_opmode_status(dev::ADSDevice) =
    convert(OPMODE_I2CMODE_STATUS, read_reg(dev, OPMODE_I2CMODE_STATUS_ADDR))

"""
    read_sequence_status(dev::ADSDevice) -> SEQUENCE_STATUS

Decode the SEQUENCE_STATUS register (0x04). Use `is_sequence_error` for a
quick boolean.
"""
read_sequence_status(dev::ADSDevice) =
    convert(SEQUENCE_STATUS, read_reg(dev, SEQUENCE_STATUS_ADDR))

"""
    is_sequence_error(dev::ADSDevice) -> Bool

`true` when SEQ_ERR_ST == 11b (auto-sequencing enabled and in error,
typically because a channel that's wired as a remote-ground or pseudo-diff
input was nevertheless selected via AUTO_SEQ_CHEN — see datasheet §7.3.1.2).
"""
is_sequence_error(dev::ADSDevice) =
    UInt8(read_sequence_status(dev).SEQ_ERR_ST) == UInt8(SEQ_ERR_ERROR)

"""
    data_buffer_count(dev::ADSDevice) -> Int

Number of FIFO entries populated since the last `start_sequence` (0..16).
"""
data_buffer_count(dev::ADSDevice) =
    Int(convert(DATA_BUFFER_STATUS, read_reg(dev, DATA_BUFFER_STATUS_ADDR)).DATA_WORDCOUNT)

"""
    accumulator_count(dev::ADSDevice) -> Int

Number of conversions accumulated since the last `start_sequence` (0..16). In
High-Precision Mode this hits 16 and the device drops BUSY/RDY.
"""
accumulator_count(dev::ADSDevice) =
    Int(convert(ACCUMULATOR_STATUS, read_reg(dev, ACCUMULATOR_STATUS_ADDR)).ACC_COUNT)

"""
    alert_trigger_channel(dev::ADSDevice) -> Int

Channel ID (0 or 1) of the channel that latched the alert first in the current
sequence. Cleared on reset / `start_sequence`.
"""
alert_trigger_channel(dev::ADSDevice) =
    Int(convert(ALERT_TRIG_CHID_REG, read_reg(dev, ALERT_TRIG_CHID_ADDR)).ALERT_TRIG_CHID)

#=============================================================================
  Reset / calibration
=============================================================================#

"""
    software_reset(dev::ADSDevice; wait_s = 0.005)

Soft-reset the device: write `0x0A` to WKEY (KEYWORD = 1010b), then write 1
to DEVICE_RESET.DEV_RST. Per the datasheet (§7.4.1) this resets all
configurations except the digital-window-comparator latched flags and the
WKEY register itself; offset calibration is NOT re-run and the I²C address
is NOT re-evaluated. The driver also clears WKEY back to 0x00 after the
reset to avoid an accidental re-reset, and re-mirrors `dout_fmt` to the POR
default.

`wait_s` (5 ms by default) is an empirical settling time — the datasheet
does not quote a soft-reset duration. Tune if needed.
"""
function software_reset(dev::ADSDevice; wait_s::Real = 0.005)
    write_reg(dev, WKEY_ADDR, WKEY_UNLOCK)
    write_reg(dev, DEVICE_RESET_ADDR, 0x01)
    sleep(wait_s)
    write_reg(dev, WKEY_ADDR, 0x00)
    dev.dout_fmt = DOUT_12BIT_ZEROS
    return nothing
end

"""
    trigger_offset_calibration(dev::ADSDevice; wait_s = 0.001)

Start an internal offset-calibration cycle (writes 1 to OFFSET_CAL.TRIG_OFFCAL).
The device holds BUSY/RDY high while calibrating. `wait_s` is a conservative
delay before returning; for high precision applications, also poll BUSY/RDY
externally. Run periodically to track AVDD / temperature drift.
"""
function trigger_offset_calibration(dev::ADSDevice; wait_s::Real = 0.001)
    write_reg(dev, OFFSET_CAL_ADDR, 0x01)
    sleep(wait_s)
    return nothing
end

#=============================================================================
  Input mux / channel sequencing
=============================================================================#

"""
    configure_input(dev::ADSDevice, cfg::InputConfig)

Select the analog-input configuration:
- `TWO_CH_SE`              — CH0 and CH1 both single-ended (POR default)
- `SINGLE_CH_SE_REMOTEGND` — CH0 SE, CH1 used as remote ground (±100 mV)
- `SINGLE_CH_PSEUDO_DIFF`  — CH0 swings about CH1 ≈ AVDD/2

When you pick a single-channel config, also disable CH1 in AUTO_SEQ_CHEN
(otherwise SEQUENCE_STATUS will set the error flag — datasheet §7.3.1.2/3).
"""
function configure_input(dev::ADSDevice, cfg::InputConfig)
    write_reg(dev, CHANNEL_INPUT_CFG_ADDR, UInt8(cfg))
    return nothing
end

"""
    configure_auto_sequence(dev::ADSDevice; ch0 = true, ch1 = true)

Enable / disable each channel for AUTO sequencing. Both are enabled by
default after power-up.
"""
function configure_auto_sequence(dev::ADSDevice; ch0::Bool = true, ch1::Bool = true)
    val = (ch0 ? UInt8(1) : UInt8(0)) | (ch1 ? UInt8(2) : UInt8(0))
    write_reg(dev, AUTO_SEQ_CHEN_ADDR, val)
    return nothing
end

#=============================================================================
  Oscillator / sample-rate control
=============================================================================#

"""
    configure_oscillator(dev::ADSDevice, osc::Oscillator, nclk::Integer)

Select the conversion oscillator and program the per-conversion clock count.
`nclk` is the number of oscillator periods per conversion cycle and is
clamped on-chip to a per-oscillator floor (`NCLK_FLOOR_HS` = 21,
`NCLK_FLOOR_LP` = 18). `nclk` must fit in 8 bits.

Sample rate `fs = osc_freq / nclk` (datasheet §7.3.5 Eq. 2). For the LP
oscillator (~10 kHz typ) and `nclk = 18` you get ≈550 SPS; for HS (~20 MHz
typ) and `nclk = 21` you get ≈950 kSPS.
"""
function configure_oscillator(dev::ADSDevice, osc::Oscillator, nclk::Integer)
    (0 <= nclk <= 0xFF) ||
        throw(ArgumentError("nCLK must fit in a UInt8, got $nclk"))
    floor = osc === OSC_HIGH_SPEED ? NCLK_FLOOR_HS : NCLK_FLOOR_LP
    if nclk < floor
        @warn "nCLK below the on-chip floor; the device will saturate it" requested=nclk floor=floor
    end
    write_reg(dev, OSC_SEL_ADDR, UInt8(osc))
    write_reg(dev, NCLK_SEL_ADDR, UInt8(nclk))
    return nothing
end

#=============================================================================
  Operation-mode selection / sequence control
=============================================================================#

"""
    set_operation_mode(dev::ADSDevice, mode::OperationMode)

Write OPMODE_SEL. The recommended sequence is documented in
datasheet Figure 54: first set offset calibration, then channel input
configuration, then operation mode.
"""
function set_operation_mode(dev::ADSDevice, mode::OperationMode)
    write_reg(dev, OPMODE_SEL_ADDR, UInt8(mode))
    return nothing
end

"""
    start_sequence(dev::ADSDevice)

Write 1 to START_SEQUENCE.SEQ_START. Brings BUSY/RDY high and kicks off the
first conversion / FIFO fill / accumulation for the currently-selected
operation mode. Also clears DATA_BUFFER_STATUS and ACCUMULATOR_STATUS.
"""
start_sequence(dev::ADSDevice) = (write_reg(dev, START_SEQUENCE_ADDR, 0x01); nothing)

"""
    abort_sequence(dev::ADSDevice)

Write 1 to ABORT_SEQUENCE.SEQ_ABORT. Stops conversions, brings BUSY/RDY low.
Recommended before switching operation modes (datasheet §7.4.3 / §7.4.4).
"""
abort_sequence(dev::ADSDevice) = (write_reg(dev, ABORT_SEQUENCE_ADDR, 0x01); nothing)

#=============================================================================
  Manual-mode reads
=============================================================================#

"""
    read_manual_samples(dev::ADSDevice, n::Integer) -> Vector{UInt16}

Convenience wrapper around `read_manual_samples!`. Allocates a fresh
`Vector{UInt16}` of length `n`, performs an I²C read of `2*n` bytes, and
returns the 12-bit codes (MSB-aligned bits already shifted down — i.e. each
element is the conversion result in `0..4095`).

In Manual Mode with AUTO sequencing across both channels, samples alternate
CH0 → CH1 → CH0 → ...

The device must be in Manual Mode with a sequence in progress. For one-shot
reads, see `read_manual` for a single sample.
"""
function read_manual_samples(dev::ADSDevice, n::Integer)
    n >= 0 || throw(ArgumentError("sample count must be non-negative"))
    raw = Vector{UInt8}(undef, 2 * n)
    read_manual_samples!(dev, raw)
    out = Vector{UInt16}(undef, n)
    @inbounds for i in 1:n
        word = (UInt16(raw[2i - 1]) << 8) | UInt16(raw[2i])
        out[i] = (word >> 4) & 0x0FFF
    end
    return out
end

"""
    read_manual(dev::ADSDevice) -> UInt16

Read a single 12-bit sample from Manual Mode. Wrapper around
`read_manual_samples(dev, 1)[1]`.
"""
read_manual(dev::ADSDevice) = read_manual_samples(dev, 1)[1]

#=============================================================================
  Data-buffer (autonomous) reads
=============================================================================#

"""
    set_data_buffer_mode(dev::ADSDevice, mode::DataBufferMode)

Set DATA_BUFFER_OPMODE.STARTSTOP_CNTRL to one of:
- `DBUF_STOP_BURST`  — capture until SEQ_ABORT
- `DBUF_START_BURST` — capture 16 samples after SEQ_START (default)
- `DBUF_PRE_ALERT`   — capture continuously, retaining last 16 before alert
- `DBUF_POST_ALERT`  — capture 16 samples after alert
"""
function set_data_buffer_mode(dev::ADSDevice, mode::DataBufferMode)
    write_reg(dev, DATA_BUFFER_OPMODE_ADDR, UInt8(mode))
    return nothing
end

"""
    set_dout_format(dev::ADSDevice, fmt::DataOutFormat)

Configure how each FIFO word is packed. Updates the chip register AND the
local mirror used by `read_data_buffer` to parse words. Callers that bypass
this helper (e.g. by writing DOUT_FORMAT_CFG directly via `write_reg`) must
follow up with `refresh_dout_format!` or the local cache will desync from
the chip.
"""
function set_dout_format(dev::ADSDevice, fmt::DataOutFormat)
    write_reg(dev, DOUT_FORMAT_CFG_ADDR, UInt8(fmt))
    dev.dout_fmt = fmt
    return nothing
end

"""
    refresh_dout_format!(dev::ADSDevice) -> DataOutFormat

Re-read DOUT_FORMAT_CFG from the chip and update `dev.dout_fmt`. Useful after
external register writes, NVM-like state restores, or a fresh `ADSDevice`
wrapping an already-configured chip.
"""
function refresh_dout_format!(dev::ADSDevice)
    raw = read_reg(dev, DOUT_FORMAT_CFG_ADDR)
    dev.dout_fmt = DataOutFormat(UInt8(raw & 0x03))
    return dev.dout_fmt
end

"""
    read_data_buffer(dev::ADSDevice; count = nothing)
        -> Vector{NamedTuple{(:code, :channel, :valid), Tuple{UInt16, UInt8, Bool}}}

Read `count` entries from the FIFO and decode them according to the locally-
tracked `dev.dout_fmt`. When `count` is `nothing`, the driver first reads
`DATA_BUFFER_STATUS` and reads that many entries (capped at 16).

Returns a vector of named tuples; for `DOUT_12BIT_ZEROS` the channel is
reported as 0 and `valid` as `true` (the format provides no channel ID).
"""
function read_data_buffer(dev::ADSDevice; count::Union{Integer, Nothing} = nothing)
    # `data_buffer_count` issues a SINGLE_READ for register 0x01. Per datasheet
    # §7.3.11.1.1 the STOP at the end of that transaction terminates the
    # register-read command, so the subsequent plain READ inside
    # `read_data_buffer!` is interpreted as a FIFO read (§7.3.7.2) rather than
    # a repeat of DATA_BUFFER_STATUS.
    n = count === nothing ? data_buffer_count(dev) : Int(count)
    (0 <= n <= 16) || throw(ArgumentError("FIFO count must be 0..16, got $n"))
    raw = Vector{UInt8}(undef, 2 * n)
    if n > 0
        read_data_buffer!(dev, raw)
    end
    out = Vector{NamedTuple{(:code, :channel, :valid),
                            Tuple{UInt16, UInt8, Bool}}}(undef, n)
    @inbounds for i in 1:n
        word = (UInt16(raw[2i - 1]) << 8) | UInt16(raw[2i])
        code, chid, valid = parse_data_word(word, dev.dout_fmt)
        out[i] = (code = code, channel = chid, valid = valid)
    end
    return out
end

#=============================================================================
  Accumulator (high-precision) reads
=============================================================================#

"""
    enable_accumulator(dev::ADSDevice, enable::Bool = true)

Set ACC_EN.EN_ACC to `0b1111` (enabled) or `0b0000` (disabled). Required for
High Precision Mode (datasheet §7.3.8 / §7.4.4); only 0000/1111 are legal.
"""
function enable_accumulator(dev::ADSDevice, enable::Bool = true)
    write_reg(dev, ACC_EN_ADDR, enable ? ACC_EN_ENABLE : ACC_EN_DISABLE)
    return nothing
end

"""
    read_accumulator(dev::ADSDevice, channel::Integer = 0) -> UInt16

Read the 16-bit accumulated result for `channel` (0 or 1). Each channel's
accumulator lives in two consecutive 8-bit registers (LSB at low address,
MSB at high address); the driver block-reads both bytes in one transaction.
"""
function read_accumulator(dev::ADSDevice, channel::Integer = 0)
    channel in (0, 1) || throw(ArgumentError("channel must be 0 or 1"))
    base = channel == 0 ? ACC_CH0_LSB_ADDR : ACC_CH1_LSB_ADDR
    buf = Vector{UInt8}(undef, 2)
    read_block!(dev, base, buf)
    return UInt16(buf[1]) | (UInt16(buf[2]) << 8)
end

"""
    read_accumulators(dev::ADSDevice) -> (UInt16, UInt16)

Read both channel accumulators in one 4-byte block read. Returns
`(ch0, ch1)`.
"""
function read_accumulators(dev::ADSDevice)
    buf = Vector{UInt8}(undef, 4)
    read_block!(dev, ACC_CH0_LSB_ADDR, buf)
    ch0 = UInt16(buf[1]) | (UInt16(buf[2]) << 8)
    ch1 = UInt16(buf[3]) | (UInt16(buf[4]) << 8)
    return (ch0, ch1)
end

#=============================================================================
  Digital window comparator: thresholds, hysteresis, event count
=============================================================================#

# Lookup tables: addresses depend on channel + which threshold pair we touch.
const _HTH_LSB = (DWC_HTH_CH0_LSB_ADDR, DWC_HTH_CH1_LSB_ADDR)
const _LTH_LSB = (DWC_LTH_CH0_LSB_ADDR, DWC_LTH_CH1_LSB_ADDR)
const _HYS     = (DWC_HYS_CH0_ADDR,     DWC_HYS_CH1_ADDR)

"""
    set_high_threshold(dev::ADSDevice, channel::Integer, value::Integer)

Set the 12-bit high-side threshold for `channel` (0 or 1) using a single
two-byte block write to the LSB+MSB register pair. `value` must fit in 12
bits (0..4095).
"""
function set_high_threshold(dev::ADSDevice, channel::Integer, value::Integer)
    channel in (0, 1) || throw(ArgumentError("channel must be 0 or 1"))
    (0 <= value <= 0xFFF) ||
        throw(ArgumentError("threshold must be 0..4095, got $value"))
    lsb = UInt8(value & 0xFF)
    msb = UInt8((UInt16(value) >> 8) & 0x0F)
    write_block(dev, _HTH_LSB[channel + 1], UInt8[lsb, msb])
    return nothing
end

"""
    set_low_threshold(dev::ADSDevice, channel::Integer, value::Integer)

Set the 12-bit low-side threshold for `channel` (0 or 1).
"""
function set_low_threshold(dev::ADSDevice, channel::Integer, value::Integer)
    channel in (0, 1) || throw(ArgumentError("channel must be 0 or 1"))
    (0 <= value <= 0xFFF) ||
        throw(ArgumentError("threshold must be 0..4095, got $value"))
    lsb = UInt8(value & 0xFF)
    msb = UInt8((UInt16(value) >> 8) & 0x0F)
    write_block(dev, _LTH_LSB[channel + 1], UInt8[lsb, msb])
    return nothing
end

"""
    set_hysteresis(dev::ADSDevice, channel::Integer, value::Integer)

Set the 6-bit hysteresis applied to both high- and low-side comparators on
`channel` (datasheet Figure 48). The high-side comparator releases when the
sample drops below `HTH - HYS`; the low-side comparator releases when the
sample rises above `LTH + HYS`.
"""
function set_hysteresis(dev::ADSDevice, channel::Integer, value::Integer)
    channel in (0, 1) || throw(ArgumentError("channel must be 0 or 1"))
    (0 <= value <= 0x3F) ||
        throw(ArgumentError("hysteresis must be 0..63, got $value"))
    write_reg(dev, _HYS[channel + 1], UInt8(value & 0x3F))
    return nothing
end

"""
    set_pre_alert_event_count(dev::ADSDevice, count::Integer)

Number of *consecutive* threshold-crossing conversions required before the
ALERT pin asserts (datasheet Figure 48). `count` must be 1..16; the on-chip
register holds `count - 1` in PREALERT_COUNT[3:0].
"""
function set_pre_alert_event_count(dev::ADSDevice, count::Integer)
    (1 <= count <= 16) ||
        throw(ArgumentError("event count must be 1..16, got $count"))
    write_reg(dev, PRE_ALT_MAX_EVENT_COUNT_ADDR, UInt8((count - 1) << 4))
    return nothing
end

#=============================================================================
  Alert / comparator master enables
=============================================================================#

"""
    enable_comparator(dev::ADSDevice, enable::Bool = true)

Toggle ALERT_DWC_EN.DWC_BLOCK_EN, the master enable for the digital window
comparator. Disabled at power-up.
"""
function enable_comparator(dev::ADSDevice, enable::Bool = true)
    if enable
        set_bits!(dev, ALERT_DWC_EN_ADDR, 0x01)
    else
        clear_bits!(dev, ALERT_DWC_EN_ADDR, 0x01)
    end
    return nothing
end

"""
    enable_channel_alert(dev::ADSDevice; ch0 = false, ch1 = false)

Set ALERT_CHEN.ALERT_EN_CHx for each channel. A channel must have its enable
set in addition to DWC_BLOCK_EN for its alert flags to contribute to the
ALERT pin.
"""
function enable_channel_alert(dev::ADSDevice; ch0::Bool = false, ch1::Bool = false)
    val = (ch0 ? UInt8(1) : UInt8(0)) | (ch1 ? UInt8(2) : UInt8(0))
    write_reg(dev, ALERT_CHEN_ADDR, val)
    return nothing
end

#=============================================================================
  Alert flags
=============================================================================#

"""
    read_alert_low_flags(dev::ADSDevice) -> ALERT_FLAGS

Decode ALERT_LOW_FLAGS (0x0C) — latched per-channel flags for the low-side
comparator.
"""
read_alert_low_flags(dev::ADSDevice) =
    convert(ALERT_FLAGS, read_reg(dev, ALERT_LOW_FLAGS_ADDR))

"""
    read_alert_high_flags(dev::ADSDevice) -> ALERT_FLAGS

Decode ALERT_HIGH_FLAGS (0x0E) — latched per-channel flags for the high-side
comparator.
"""
read_alert_high_flags(dev::ADSDevice) =
    convert(ALERT_FLAGS, read_reg(dev, ALERT_HIGH_FLAGS_ADDR))

"""
    clear_alert_flags(dev::ADSDevice; low::Bool = true, high::Bool = true)

Clear the latched alert flags by writing `0xFF` (a write of 1 to a flag bit
clears it, per datasheet §7.6.8.15/16). It is recommended to do this while
BUSY/RDY is low. Pass `low = false` or `high = false` to skip one side.
"""
function clear_alert_flags(dev::ADSDevice; low::Bool = true, high::Bool = true)
    low  && write_reg(dev, ALERT_LOW_FLAGS_ADDR,  0xFF)
    high && write_reg(dev, ALERT_HIGH_FLAGS_ADDR, 0xFF)
    return nothing
end

#=============================================================================
  High-level mode setup helpers

  Each helper bundles the back half of the recommended power-up sequence
  from datasheet Figure 54 (input cfg → operation mode), aborts any
  in-flight sequence first per §7.4.3, and leaves the device armed but not
  yet started — the caller decides when to issue `start_sequence`.

  The offset-calibration step from Figure 54 is intentionally NOT bundled
  here: the device self-calibrates at POR and re-calibration is a periodic
  drift-tracking concern (AVDD / temperature) rather than something the
  setup path always needs. Call `trigger_offset_calibration` from the
  caller before `start_sequence` whenever a fresh cal is wanted.
=============================================================================#

"""
    configure_manual_mode(dev; auto_sequence = false, input = TWO_CH_SE,
                              ch0 = true, ch1 = true)

Put the device in Manual Mode. With `auto_sequence = false` (the default)
the device scans CH0 only; with `auto_sequence = true` it cycles through the
channels enabled in AUTO_SEQ_CHEN (`ch0`/`ch1`).

Manual Mode always uses the High-Speed oscillator and ignores nCLK
(datasheet §7.4.2). Sample rate is set by the I²C clock.
"""
function configure_manual_mode(dev::ADSDevice;
                               auto_sequence::Bool = false,
                               input::InputConfig  = TWO_CH_SE,
                               ch0::Bool           = true,
                               ch1::Bool           = true)
    abort_sequence(dev)
    configure_input(dev, input)
    if auto_sequence
        configure_auto_sequence(dev; ch0 = ch0, ch1 = ch1)
        set_operation_mode(dev, OPMODE_MANUAL_AUTO)
    else
        set_operation_mode(dev, OPMODE_MANUAL_CH0)
    end
    return nothing
end

"""
    configure_autonomous_mode(dev; oscillator = OSC_LOW_POWER, nclk = NCLK_FLOOR_LP,
                                  buffer_mode = DBUF_PRE_ALERT,
                                  format = DOUT_12BIT_CHID_VALID,
                                  input = TWO_CH_SE, ch0 = true, ch1 = true)

Put the device in Autonomous Monitoring Mode with thresholds + FIFO. The
caller still needs to program the thresholds (`set_high_threshold`,
`set_low_threshold`, `set_hysteresis`), the event count
(`set_pre_alert_event_count`), and enable the comparator
(`enable_comparator`, `enable_channel_alert`) before calling `start_sequence`.
"""
function configure_autonomous_mode(dev::ADSDevice;
                                   oscillator::Oscillator     = OSC_LOW_POWER,
                                   nclk::Integer              = NCLK_FLOOR_LP,
                                   buffer_mode::DataBufferMode = DBUF_PRE_ALERT,
                                   format::DataOutFormat      = DOUT_12BIT_CHID_VALID,
                                   input::InputConfig         = TWO_CH_SE,
                                   ch0::Bool                  = true,
                                   ch1::Bool                  = true)
    abort_sequence(dev)
    configure_input(dev, input)
    configure_auto_sequence(dev; ch0 = ch0, ch1 = ch1)
    configure_oscillator(dev, oscillator, nclk)
    set_data_buffer_mode(dev, buffer_mode)
    set_dout_format(dev, format)
    set_operation_mode(dev, OPMODE_AUTONOMOUS_AUTO)
    return nothing
end

"""
    configure_high_precision_mode(dev; oscillator = OSC_LOW_POWER, nclk = NCLK_FLOOR_LP,
                                       input = TWO_CH_SE, ch0 = true, ch1 = true)

Put the device in High Precision Mode (16 × 12-bit accumulation → 16-bit
result per channel). The accumulator is enabled; results are read via
`read_accumulator` / `read_accumulators` once BUSY/RDY drops.
"""
function configure_high_precision_mode(dev::ADSDevice;
                                       oscillator::Oscillator = OSC_LOW_POWER,
                                       nclk::Integer          = NCLK_FLOOR_LP,
                                       input::InputConfig     = TWO_CH_SE,
                                       ch0::Bool              = true,
                                       ch1::Bool              = true)
    abort_sequence(dev)
    configure_input(dev, input)
    configure_auto_sequence(dev; ch0 = ch0, ch1 = ch1)
    configure_oscillator(dev, oscillator, nclk)
    enable_accumulator(dev, true)
    set_operation_mode(dev, OPMODE_HIGH_PRECISION)
    return nothing
end

#=============================================================================
  Exports (outer module)
=============================================================================#

export
    # Device
    ADSDevice, open_ads, close_ads, close!,
    # Low-level
    read_reg, write_reg, set_bits!, clear_bits!,
    read_block!, write_block,
    read_data_buffer!, read_manual_samples!,
    as_byte,
    # Probe / status
    probe, read_opmode_status, read_sequence_status, is_sequence_error,
    data_buffer_count, accumulator_count, alert_trigger_channel,
    # Reset / calibration
    software_reset, trigger_offset_calibration,
    # Input mux / channel sequencing
    configure_input, configure_auto_sequence,
    # Oscillator
    configure_oscillator,
    # Operation mode / sequence control
    set_operation_mode, start_sequence, abort_sequence,
    # Manual mode
    read_manual, read_manual_samples,
    # Data buffer
    set_data_buffer_mode, set_dout_format, refresh_dout_format!,
    read_data_buffer,
    # Accumulator
    enable_accumulator, read_accumulator, read_accumulators,
    # Digital window comparator
    set_high_threshold, set_low_threshold, set_hysteresis,
    set_pre_alert_event_count,
    # Alert
    enable_comparator, enable_channel_alert,
    read_alert_low_flags, read_alert_high_flags, clear_alert_flags,
    # High-level setup
    configure_manual_mode, configure_autonomous_mode,
    configure_high_precision_mode

end # module ADS7142
