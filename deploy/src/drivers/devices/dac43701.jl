module DAC43701

using ..I2C
using FieldFlags

"""
    Registers

FieldFlags.jl bit-layout descriptions for the registers of the Texas Instruments
DAC43701 — 8-bit voltage-output smart DAC with NVM and PMBus-compatible I²C.

Conventions
-----------
* All structs follow FieldFlags.jl's LSB-first layout: the FIRST field written
  occupies the LEAST significant bit. This matches how the datasheet numbers
  individual bits within each register (bit 0 = LSB, bit 15 = MSB).
* `_` / `_:n` is reserved/padding, exactly as defined in the datasheet.
* Configuration registers are declared `mutable` because the host writes to
  them and the driver caches the last-written value. Read-only status
  registers are non-mutable.
* DAC43701 registers are all 16 bits wide. FieldFlags-generated structs are
  16-bit packed (storage type `UInt16`). The 16-bit data-aligned registers
  (`DAC_DATA`, `DAC_MARGIN_HIGH`, `DAC_MARGIN_LOW`) and the PMBus-style
  byte-in-a-word registers (`PMBUS_OPERATION`, `PMBUS_VERSION`) are NOT
  bit-packed and are handled as plain `UInt16` values with helpers below.

Reference: DAC53701/DAC43701 datasheet, SLASEY5 (Texas Instruments, Dec 2020).
"""
module Registers

using FieldFlags

#=============================================================================
  STATUS @ 0xD0  (read-only, reset = 0x0014 for DAC43701)
=============================================================================#

# STATUS: NVM/DAC busy flags, CRC alarms, device identification.
#
# | bits  | name                    | meaning                                 |
# |-------|-------------------------|-----------------------------------------|
# | 1:0   | VERSION_ID              | silicon revision                        |
# | 5:2   | DEVICE_ID               | 0x3 = DAC53701, 0x5 = DAC43701          |
# | 11:6  | (reserved)              |                                         |
# | 12    | DAC_UPDATE_BUSY         | 1 = DAC output update in progress       |
# | 13    | NVM_BUSY                | 1 = NVM write/reload in progress        |
# | 14    | NVM_CRC_ALARM_INTERNAL  | 1 = internal NVM CRC failure            |
# | 15    | NVM_CRC_ALARM_USER      | 1 = user NVM CRC failure                |
@bitfield struct STATUS
    VERSION_ID:2
    DEVICE_ID:4
    _:6
    DAC_UPDATE_BUSY:1
    NVM_BUSY:1
    NVM_CRC_ALARM_INTERNAL:1
    NVM_CRC_ALARM_USER:1
end

#=============================================================================
  GENERAL_CONFIG @ 0xD1  (R/W, reset = 0x01F0)
=============================================================================#

# GENERAL_CONFIG: reference, span, power-down, digital-slew control, PMBus
# enable, device lock, and continuous-waveform type.
#
# | bits  | name         | meaning                                            |
# |-------|--------------|----------------------------------------------------|
# | 1:0   | DAC_SPAN     | when REF_EN: 0=×1.5, 1=×2, 2=×3, 3=×4              |
# | 2     | REF_EN       | 0 = use VDD as reference, 1 = internal 1.21 V      |
# | 4:3   | DAC_PDN      | 0=normal, 1=PD-10k, 2=PD-HiZ (default), 3=PD-10k   |
# | 8:5   | SLEW_RATE    | per-step time (see SlewRate enum / Table 8-4)      |
# | 11:9  | CODE_STEP    | step size (see CodeStep enum / Table 8-3)          |
# | 12    | EN_PMBUS     | enable PMBus command set                           |
# | 13    | DEVICE_LOCK  | lock all registers (unlock via TRIGGER)            |
# | 15:14 | FUNC_CONFIG  | 0=tri, 1=saw-rising, 2=saw-falling, 3=square       |
@bitfield mutable struct GENERAL_CONFIG
    DAC_SPAN:2
    REF_EN:1
    DAC_PDN:2
    SLEW_RATE:4
    CODE_STEP:3
    EN_PMBUS:1
    DEVICE_LOCK:1
    FUNC_CONFIG:2
end

#=============================================================================
  CONFIG2 @ 0xD2  (R/W, reset = 0x0000)
=============================================================================#

# CONFIG2: medical-alarm timings and triggers, GPI configuration, slave-address
# select.
#
# | bits  | name             | meaning                                         |
# |-------|------------------|-------------------------------------------------|
# | 1:0   | PULSE_ON_TIME    | alarm pulse high time (Table 8-10)              |
# | 3:2   | PULSE_OFF_TIME   | alarm pulse low time  (Table 8-9)               |
# | 5:4   | INTERBURST_TIME  | alarm inter-burst gap (Table 8-8)               |
# | 7:6   | (reserved)       |                                                 |
# | 8     | MED_ALARM_LP     | 1 = low-priority medical alarm running          |
# | 9     | MED_ALARM_MP     | 1 = medium-priority medical alarm running       |
# | 10    | MED_ALARM_HP     | 1 = high-priority medical alarm running         |
# | 13:11 | GPI_CONFIG       | GPI function map (see GPIConfig / Table 8-1)    |
# | 15:14 | SLAVE_ADDRESS    | AD1-AD0 of 7-bit I²C address (Table 8-13)       |
@bitfield mutable struct CONFIG2
    PULSE_ON_TIME:2
    PULSE_OFF_TIME:2
    INTERBURST_TIME:2
    _:2
    MED_ALARM_LP:1
    MED_ALARM_MP:1
    MED_ALARM_HP:1
    GPI_CONFIG:3
    SLAVE_ADDRESS:2
end

#=============================================================================
  TRIGGER @ 0xD3  (R/W with self-clearing and write-only bits, reset = 0x0008)
=============================================================================#

# TRIGGER: one-shot action bits plus the persistent GPI_EN. SW_RESET and
# DEVICE_UNLOCK_CODE are write-only; the rest of the action bits (NVM_*,
# PMBUS_MARGIN_*, START_FUNC_GEN, DEVICE_CONFIG_RESET) self-clear after the
# action completes.
#
# | bits  | name                | meaning                                      |
# |-------|---------------------|----------------------------------------------|
# | 3:0   | SW_RESET            | write 0b1010 to issue software reset         |
# | 4     | NVM_PROG            | 1 = program NVM from registers (self-clears) |
# | 5     | NVM_RELOAD          | 1 = reload registers from NVM (self-clears)  |
# | 6     | PMBUS_MARGIN_LOW    | 1 = slew to MARGIN_LOW (self-clears at done) |
# | 7     | PMBUS_MARGIN_HIGH   | 1 = slew to MARGIN_HIGH (self-clears)        |
# | 8     | START_FUNC_GEN      | 1 = continuous waveform generation enabled   |
# | 9     | DEVICE_CONFIG_RESET | 1 = reload factory defaults (no NVM touch)   |
# | 10    | GPI_EN              | 1 = GPI active (sticky)                      |
# | 11    | (don't care)        |                                              |
# | 15:12 | DEVICE_UNLOCK_CODE  | write 0b0101 to bypass DEVICE_LOCK           |
@bitfield mutable struct TRIGGER
    SW_RESET:4
    NVM_PROG:1
    NVM_RELOAD:1
    PMBUS_MARGIN_LOW:1
    PMBUS_MARGIN_HIGH:1
    START_FUNC_GEN:1
    DEVICE_CONFIG_RESET:1
    GPI_EN:1
    _:1
    DEVICE_UNLOCK_CODE:4
end

#=============================================================================
  PMBUS_STATUS_BYTE @ 0x78  (R/W, only CML is meaningful)
=============================================================================#

# PMBUS_STATUS_BYTE: PMBus communication-fault flag. Lower byte is not
# applicable per the datasheet register map. Write 1 to CML to clear.
#
# | bits  | name        | meaning                                              |
# |-------|-------------|------------------------------------------------------|
# | 8:0   | (n/a)       |                                                      |
# | 9     | CML         | 1 = PMBus communication fault; write 1 to clear      |
# | 15:10 | (n/a)       |                                                      |
@bitfield mutable struct PMBUS_STATUS_BYTE
    _:9
    CML:1
    _:6
end

#=============================================================================
  Register addresses
=============================================================================#

const PMBUS_OPERATION_ADDR   = 0x01
const DAC_DATA_ADDR          = 0x21
const DAC_MARGIN_HIGH_ADDR   = 0x25
const DAC_MARGIN_LOW_ADDR    = 0x26
const PMBUS_STATUS_BYTE_ADDR = 0x78
const PMBUS_VERSION_ADDR     = 0x98
const STATUS_ADDR            = 0xD0
const GENERAL_CONFIG_ADDR    = 0xD1
const CONFIG2_ADDR           = 0xD2
const TRIGGER_ADDR           = 0xD3

#=============================================================================
  DAC43701 constants and enums
=============================================================================#

# DEVICE_ID values (STATUS bits 5:2). The DAC53701 sibling returns 0x3.
const DAC43701_DEVICE_ID = 0x05
const DAC53701_DEVICE_ID = 0x03

# PMBus version readback (upper byte of PMBUS_VERSION register).
const PMBUS_VERSION_VALUE = 0x22

# Datasheet §8.5.2.1: 7-bit address is 1001 0xx, default xx = 00. Broadcast is
# 1000111 (write-only, all devices respond regardless of SLAVE_ADDRESS bits).
const DAC43701_ADDR_DEFAULT   = 0x48  # SLAVE_ADDRESS = 00
const DAC43701_ADDR_OPT_01    = 0x49
const DAC43701_ADDR_OPT_10    = 0x4A
const DAC43701_ADDR_OPT_11    = 0x4B
const DAC43701_ADDR_BROADCAST = 0x47

# SW_RESET code (lower 4 bits of TRIGGER) — write 0b1010 to issue a soft reset.
# The "no reset" / readback code is 0b1000.
const SW_RESET_TRIGGER = 0x0A
const SW_RESET_NOOP    = 0x08

# DEVICE_UNLOCK_CODE (upper 4 bits of TRIGGER) — write 0b0101 to bypass lock.
const DEVICE_UNLOCK_CODE_VALUE = 0x05

# DAC reference selection (REF_EN field).
@enum DACReference::UInt8 begin
    VDD_REF      = 0
    INTERNAL_REF = 1
end

# Internal-reference output span (DAC_SPAN field).
@enum DACSpan::UInt8 begin
    SPAN_1_5X = 0b00
    SPAN_2X   = 0b01
    SPAN_3X   = 0b10
    SPAN_4X   = 0b11
end

# DAC power state (DAC_PDN field). Codes 01 and 11 both mean PD-10k; we expose
# just one canonical value.
@enum PowerDownMode::UInt8 begin
    NORMAL = 0b00
    PD_10K = 0b01
    PD_HIZ = 0b10
end

# Continuous-waveform type (FUNC_CONFIG field).
@enum FunctionType::UInt8 begin
    TRIANGLE          = 0b00
    SAWTOOTH_RISING   = 0b01
    SAWTOOTH_FALLING  = 0b10
    SQUARE            = 0b11
end

# Slew step size (CODE_STEP field, Table 8-3).
@enum CodeStep::UInt8 begin
    STEP_1  = 0b000
    STEP_2  = 0b001
    STEP_3  = 0b010
    STEP_4  = 0b011
    STEP_6  = 0b100
    STEP_8  = 0b101
    STEP_16 = 0b110
    STEP_32 = 0b111
end

# Slew rate / per-step time (SLEW_RATE field, Table 8-4).
@enum SlewRate::UInt8 begin
    SR_25_6US    = 0b0000
    SR_32US      = 0b0001
    SR_38_4US    = 0b0010
    SR_44_8US    = 0b0011
    SR_204_8US   = 0b0100
    SR_256US     = 0b0101
    SR_307_2US   = 0b0110
    SR_819_2US   = 0b0111
    SR_1_6384MS  = 0b1000
    SR_2_4576MS  = 0b1001
    SR_3_2768MS  = 0b1010
    SR_4_9152MS  = 0b1011
    SR_12US      = 0b1100
    SR_8US       = 0b1101
    SR_4US       = 0b1110
    SR_NONE      = 0b1111
end

# Per-step time in seconds, indexed by SlewRate (Table 8-4).
const SLEW_PERIOD_SECONDS = Dict{SlewRate, Float64}(
    SR_25_6US   => 25.6e-6,
    SR_32US     => 32.0e-6,
    SR_38_4US   => 38.4e-6,
    SR_44_8US   => 44.8e-6,
    SR_204_8US  => 204.8e-6,
    SR_256US    => 256.0e-6,
    SR_307_2US  => 307.2e-6,
    SR_819_2US  => 819.2e-6,
    SR_1_6384MS => 1638.4e-6,
    SR_2_4576MS => 2457.6e-6,
    SR_3_2768MS => 3276.8e-6,
    SR_4_9152MS => 4915.2e-6,
    SR_12US     => 12.0e-6,
    SR_8US      => 8.0e-6,
    SR_4US      => 4.0e-6,
    SR_NONE     => 0.0,
)

# Integer LSB count per CodeStep (Table 8-3).
const CODE_STEP_LSB = Dict{CodeStep, Int}(
    STEP_1 => 1, STEP_2 => 2, STEP_3 => 3, STEP_4 => 4,
    STEP_6 => 6, STEP_8 => 8, STEP_16 => 16, STEP_32 => 32,
)

# GPI behavior (GPI_CONFIG field, Table 8-1). The pin function only takes
# effect when GPI_EN = 1.
@enum GPIConfig::UInt8 begin
    GPI_PD_HIZ      = 0b000  # rising = power-up, falling = Hi-Z power-down
    GPI_PD_10K      = 0b001
    GPI_MARGIN      = 0b010
    GPI_FUNC_GEN    = 0b011
    GPI_ALARM_HP    = 0b100
    GPI_ALARM_MP    = 0b101
    GPI_ALARM_LP    = 0b110
    GPI_SLAVE_ADDR  = 0b111
end

# Medical-alarm priority.
@enum AlarmPriority::UInt8 begin
    ALARM_HIGH   = 0
    ALARM_MEDIUM = 1
    ALARM_LOW    = 2
end

# PMBus OPERATION command-byte values (datasheet Table 8-7). These sit in the
# upper byte of PMBUS_OPERATION; the lower byte is don't-care.
@enum PMBusOp::UInt8 begin
    PMBUS_TURN_OFF        = 0x00
    PMBUS_TURN_ON         = 0x80
    PMBUS_MARGIN_LOW_CMD  = 0x94
    PMBUS_MARGIN_HIGH_CMD = 0xA4
end

#=============================================================================
  Data-byte alignment helpers (8-bit DAC, 16-bit register)
=============================================================================#

# DAC_DATA, DAC_MARGIN_HIGH, DAC_MARGIN_LOW on the 8-bit DAC43701 carry the
# 8-bit code MSB-left-aligned within bits [11:4] of their 16-bit register
# (datasheet §8.6.5). Bits 15:12 and 3:0 are don't-care. This is equivalent to
# `code << 4`.
@inline encode_code8(code::Integer) = UInt16((UInt16(code & 0xFF) << 4))
@inline decode_code8(reg::UInt16)   = UInt8((reg >> 4) & 0xFF)

#=============================================================================
  Reset values (chip-side defaults after POR)
=============================================================================#

# GENERAL_CONFIG reset = 0x01F0: DAC_SPAN=0, REF_EN=0, DAC_PDN=PD_HIZ,
# SLEW_RATE=SR_NONE, CODE_STEP=0, EN_PMBUS=0, DEVICE_LOCK=0, FUNC_CONFIG=0.
general_config_reset() = GENERAL_CONFIG(
    UInt8(SPAN_1_5X),  # DAC_SPAN
    0,                 # REF_EN
    UInt8(PD_HIZ),     # DAC_PDN
    UInt8(SR_NONE),    # SLEW_RATE
    UInt8(STEP_1),     # CODE_STEP
    0,                 # EN_PMBUS
    0,                 # DEVICE_LOCK
    UInt8(TRIANGLE),   # FUNC_CONFIG
)

# CONFIG2 reset = 0x0000: all fields zero.
config2_reset() = CONFIG2(0, 0, 0, 0, 0, 0, 0, 0)

#=============================================================================
  Exports
=============================================================================#

export
    # FieldFlags struct types
    STATUS, GENERAL_CONFIG, CONFIG2, TRIGGER, PMBUS_STATUS_BYTE,
    # Register addresses
    PMBUS_OPERATION_ADDR, DAC_DATA_ADDR,
    DAC_MARGIN_HIGH_ADDR, DAC_MARGIN_LOW_ADDR,
    PMBUS_STATUS_BYTE_ADDR, PMBUS_VERSION_ADDR,
    STATUS_ADDR, GENERAL_CONFIG_ADDR, CONFIG2_ADDR, TRIGGER_ADDR,
    # Constants
    DAC43701_DEVICE_ID, DAC53701_DEVICE_ID, PMBUS_VERSION_VALUE,
    DAC43701_ADDR_DEFAULT, DAC43701_ADDR_OPT_01,
    DAC43701_ADDR_OPT_10, DAC43701_ADDR_OPT_11, DAC43701_ADDR_BROADCAST,
    SW_RESET_TRIGGER, SW_RESET_NOOP, DEVICE_UNLOCK_CODE_VALUE,
    SLEW_PERIOD_SECONDS, CODE_STEP_LSB,
    # Enums and their members
    DACReference, VDD_REF, INTERNAL_REF,
    DACSpan, SPAN_1_5X, SPAN_2X, SPAN_3X, SPAN_4X,
    PowerDownMode, NORMAL, PD_10K, PD_HIZ,
    FunctionType, TRIANGLE, SAWTOOTH_RISING, SAWTOOTH_FALLING, SQUARE,
    CodeStep, STEP_1, STEP_2, STEP_3, STEP_4, STEP_6, STEP_8, STEP_16, STEP_32,
    SlewRate,
    SR_25_6US, SR_32US, SR_38_4US, SR_44_8US,
    SR_204_8US, SR_256US, SR_307_2US, SR_819_2US,
    SR_1_6384MS, SR_2_4576MS, SR_3_2768MS, SR_4_9152MS,
    SR_12US, SR_8US, SR_4US, SR_NONE,
    GPIConfig,
    GPI_PD_HIZ, GPI_PD_10K, GPI_MARGIN, GPI_FUNC_GEN,
    GPI_ALARM_HP, GPI_ALARM_MP, GPI_ALARM_LP, GPI_SLAVE_ADDR,
    AlarmPriority, ALARM_HIGH, ALARM_MEDIUM, ALARM_LOW,
    PMBusOp, PMBUS_TURN_OFF, PMBUS_TURN_ON,
    PMBUS_MARGIN_LOW_CMD, PMBUS_MARGIN_HIGH_CMD,
    # Helpers
    encode_code8, decode_code8,
    general_config_reset, config2_reset

end # module Registers

using .Registers

#=============================================================================
  Device struct
=============================================================================#

"""
    DACDevice

Wrapper for a Texas Instruments DAC43701 over I²C.

# Fields
- `device::I2C.I2CDevice`: I²C device handle.
- `gc::GENERAL_CONFIG`:    cached GENERAL_CONFIG (D1h) contents.
- `c2::CONFIG2`:           cached CONFIG2 (D2h) contents.
- `tx_buf::Vector{UInt8}`: pre-allocated 2-byte TX buffer (MSDB, LSDB).
- `rx_buf::Vector{UInt8}`: pre-allocated 2-byte RX buffer for register reads.

The two cached config structs are updated by every high-level helper that
writes them, so per-field updates don't clobber neighbouring fields.
"""
mutable struct DACDevice
    device::I2C.I2CDevice
    gc::GENERAL_CONFIG
    c2::CONFIG2
    tx_buf::Vector{UInt8}
    rx_buf::Vector{UInt8}
end

"""
    DACDevice(device::I2C.I2CDevice)

Wrap an already-opened I²C device. The cached configuration structs are
initialised to the chip's POR defaults. If your DAC has had its NVM
reprogrammed, call `refresh_cache!(dev)` afterwards so the cache matches what
the chip currently holds.
"""
function DACDevice(device::I2C.I2CDevice)
    return DACDevice(device, general_config_reset(), config2_reset(),
                     zeros(UInt8, 2), zeros(UInt8, 2))
end

"""
    open_dac(bus::Integer, address::Integer = DAC43701_ADDR_DEFAULT) -> DACDevice

Open `/dev/i2c-<bus>` at the given 7-bit address and return a wrapped
`DACDevice`.
"""
function open_dac(bus::Integer, address::Integer = DAC43701_ADDR_DEFAULT)
    return DACDevice(I2C.open_device(Int(bus), UInt8(address)))
end

"""
    close_dac(dev::DACDevice)

Close the underlying I²C device.
"""
close_dac(dev::DACDevice) = I2C.close_device(dev.device)

#=============================================================================
  Low-level I²C register operations
=============================================================================#

"""
    write_reg(dev::DACDevice, reg::Integer, value::UInt16)

Write a 16-bit register, MSDB then LSDB on the wire (datasheet §8.5.2 update
sequence: address byte, command byte, MSDB, LSDB).
"""
function write_reg(dev::DACDevice, reg::Integer, value::UInt16)
    dev.tx_buf[1] = UInt8((value >> 8) & 0xFF)  # MSDB
    dev.tx_buf[2] = UInt8(value & 0xFF)         # LSDB
    I2C.write_bytes(dev.device, UInt8(reg), dev.tx_buf)
    return value
end

write_reg(dev::DACDevice, reg::Integer, value::Integer) =
    write_reg(dev, reg, UInt16(value & 0xFFFF))

"""
    read_reg(dev::DACDevice, reg::Integer) -> UInt16

Read a 16-bit register. The DAC sends MSDB first, then LSDB.
"""
function read_reg(dev::DACDevice, reg::Integer)
    I2C.read_bytes!(dev.device, UInt8(reg), dev.rx_buf)
    return (UInt16(dev.rx_buf[1]) << 8) | UInt16(dev.rx_buf[2])
end

"""
    modify_reg!(dev::DACDevice, reg::Integer, mask::UInt16, value::UInt16) -> UInt16

Read-modify-write: clear bits in `mask`, OR in `value & mask`, write back.
Useful for TRIGGER updates where the persistent GPI_EN bit must be preserved.
"""
function modify_reg!(dev::DACDevice, reg::Integer, mask::UInt16, value::UInt16)
    cur = read_reg(dev, reg)
    new = (cur & ~mask) | (value & mask)
    write_reg(dev, reg, new)
    return new
end

#=============================================================================
  Helpers: FieldFlags → UInt16
=============================================================================#

# FieldFlags structs wrap a primitive type stored in their internal `:fields`
# slot. For our 16-bit registers that primitive is `UInt16`. `as_word` pulls
# it back out for `write_reg`.
@inline as_word(x) = reinterpret(UInt16, getfield(x, :fields))

#=============================================================================
  Cache management
=============================================================================#

"""
    refresh_cache!(dev::DACDevice)

Read GENERAL_CONFIG and CONFIG2 from the chip and update the cached structs.
Call this after a soft reset, NVM reload, or whenever you suspect the cache
may have drifted (e.g. at startup, before trusting cached values for
field-level updates).
"""
function refresh_cache!(dev::DACDevice)
    dev.gc = convert(GENERAL_CONFIG, read_reg(dev, GENERAL_CONFIG_ADDR))
    dev.c2 = convert(CONFIG2,        read_reg(dev, CONFIG2_ADDR))
    return dev
end

"""
    flush_general_config(dev::DACDevice)

Write the cached `dev.gc` back to GENERAL_CONFIG (D1h).
"""
flush_general_config(dev::DACDevice) =
    write_reg(dev, GENERAL_CONFIG_ADDR, as_word(dev.gc))

"""
    flush_config2(dev::DACDevice)

Write the cached `dev.c2` back to CONFIG2 (D2h).
"""
flush_config2(dev::DACDevice) =
    write_reg(dev, CONFIG2_ADDR, as_word(dev.c2))

#=============================================================================
  Status / probe
=============================================================================#

"""
    read_status(dev::DACDevice) -> STATUS

Read the STATUS register (D0h) and decode it into a `STATUS` struct.
"""
read_status(dev::DACDevice) = convert(STATUS, read_reg(dev, STATUS_ADDR))

"""
    device_id(dev::DACDevice) -> UInt8

Return the 4-bit DEVICE_ID. Should be `DAC43701_DEVICE_ID` (0x5).
"""
device_id(dev::DACDevice) = UInt8(read_status(dev).DEVICE_ID)

is_nvm_busy(dev::DACDevice) = read_status(dev).NVM_BUSY != 0
is_dac_busy(dev::DACDevice) = read_status(dev).DAC_UPDATE_BUSY != 0

"""
    probe(dev::DACDevice) -> Bool

Verify the chip responds and reports DEVICE_ID == 0x5 (DAC43701). Throws on
mismatch.
"""
function probe(dev::DACDevice)
    id = device_id(dev)
    if id != DAC43701_DEVICE_ID
        msg = if id == DAC53701_DEVICE_ID
            "got DEVICE_ID = 0x3 (this is a DAC53701, the 10-bit sibling)"
        else
            "got DEVICE_ID = 0x$(string(id, base=16))"
        end
        error("DAC43701 probe failed: expected 0x5, $msg")
    end
    return true
end

"""
    wait_nvm_idle(dev::DACDevice; timeout_s = 0.1, poll_s = 0.001)

Block until `STATUS.NVM_BUSY` clears. Datasheet specifies NVM writes complete
in ≤20 ms (typ 10 ms).
"""
function wait_nvm_idle(dev::DACDevice; timeout_s::Real = 0.1, poll_s::Real = 0.001)
    t0 = time()
    while is_nvm_busy(dev)
        time() - t0 > timeout_s && error("DAC43701: timeout waiting for NVM_BUSY")
        sleep(poll_s)
    end
    return nothing
end

"""
    wait_dac_idle(dev::DACDevice; timeout_s = 5.0, poll_s = 0.001)

Block until `STATUS.DAC_UPDATE_BUSY` clears. The timeout is generous because
medical alarms and slow slews can hold this bit for seconds.
"""
function wait_dac_idle(dev::DACDevice; timeout_s::Real = 5.0, poll_s::Real = 0.001)
    t0 = time()
    while is_dac_busy(dev)
        time() - t0 > timeout_s && error("DAC43701: timeout waiting for DAC_UPDATE_BUSY")
        sleep(poll_s)
    end
    return nothing
end

#=============================================================================
  DAC data / margin registers (plain 8-bit codes, left-aligned in 16-bit reg)
=============================================================================#

"""
    set_dac_code(dev::DACDevice, code::Integer)

Write the 8-bit DAC code (0..255). With VDD reference and gain 1×:
`Vout = code / 256 * VDD`.
"""
function set_dac_code(dev::DACDevice, code::Integer)
    (0 <= code <= 0xFF) || throw(ArgumentError("DAC code must be 0..255, got $code"))
    write_reg(dev, DAC_DATA_ADDR, encode_code8(code))
    return UInt8(code)
end

"""
    get_dac_code(dev::DACDevice) -> UInt8

Read the currently-loaded DAC_DATA code.
"""
get_dac_code(dev::DACDevice) = decode_code8(read_reg(dev, DAC_DATA_ADDR))

"""
    set_margin_high(dev::DACDevice, code::Integer)

Set the upper code used by margin-high, GPI margin triggering, and function
generation.
"""
function set_margin_high(dev::DACDevice, code::Integer)
    (0 <= code <= 0xFF) || throw(ArgumentError("margin code must be 0..255"))
    write_reg(dev, DAC_MARGIN_HIGH_ADDR, encode_code8(code))
    return UInt8(code)
end

"""
    set_margin_low(dev::DACDevice, code::Integer)

Set the lower code used by margin-low, GPI margin triggering, and function
generation.
"""
function set_margin_low(dev::DACDevice, code::Integer)
    (0 <= code <= 0xFF) || throw(ArgumentError("margin code must be 0..255"))
    write_reg(dev, DAC_MARGIN_LOW_ADDR, encode_code8(code))
    return UInt8(code)
end

#=============================================================================
  GENERAL_CONFIG helpers (operate on dev.gc cache, then flush)
=============================================================================#

"""
    configure_reference(dev; ref = VDD_REF, span = SPAN_1_5X)

Select between VDD as reference (gain always 1×) and the internal 1.21 V
reference with the chosen span. `span` is ignored when `ref == VDD_REF`.
"""
function configure_reference(dev::DACDevice;
                             ref::DACReference = VDD_REF,
                             span::DACSpan    = SPAN_1_5X)
    dev.gc.REF_EN   = UInt8(ref)
    dev.gc.DAC_SPAN = UInt8(span)
    flush_general_config(dev)
    return nothing
end

"""
    power_up(dev; ref = nothing, span = nothing)

Bring the DAC out of power-down. If `ref` and/or `span` are supplied, the
reference is reconfigured at the same time (single register write). Otherwise
the existing cached reference settings are preserved.
"""
function power_up(dev::DACDevice;
                  ref::Union{DACReference, Nothing} = nothing,
                  span::Union{DACSpan, Nothing}    = nothing)
    dev.gc.DAC_PDN = UInt8(NORMAL)
    if ref !== nothing
        dev.gc.REF_EN = UInt8(ref)
    end
    if span !== nothing
        dev.gc.DAC_SPAN = UInt8(span)
    end
    flush_general_config(dev)
    return nothing
end

"""
    power_down(dev, mode::PowerDownMode = PD_HIZ)

Power the DAC output down to high-Z or 10 kΩ-to-GND.
"""
function power_down(dev::DACDevice, mode::PowerDownMode = PD_HIZ)
    dev.gc.DAC_PDN = UInt8(mode)
    flush_general_config(dev)
    return nothing
end

"""
    configure_slew(dev; code_step = STEP_1, slew_rate = SR_NONE)

Program the digital-slew CODE_STEP and SLEW_RATE fields. `SR_NONE` is the
reset default (immediate code transitions).
"""
function configure_slew(dev::DACDevice;
                        code_step::CodeStep  = STEP_1,
                        slew_rate::SlewRate = SR_NONE)
    dev.gc.CODE_STEP = UInt8(code_step)
    dev.gc.SLEW_RATE = UInt8(slew_rate)
    flush_general_config(dev)
    return nothing
end

"""
    enable_pmbus(dev, enable::Bool = true)

Toggle GENERAL_CONFIG.EN_PMBUS.
"""
function enable_pmbus(dev::DACDevice, enable::Bool = true)
    dev.gc.EN_PMBUS = enable ? 1 : 0
    flush_general_config(dev)
    return nothing
end

"""
    lock_device(dev)

Set DEVICE_LOCK to block register writes. Bypass via `unlock_device`.
"""
function lock_device(dev::DACDevice)
    dev.gc.DEVICE_LOCK = 1
    flush_general_config(dev)
    return nothing
end

#=============================================================================
  CONFIG2 helpers (operate on dev.c2 cache, then flush)
=============================================================================#

"""
    configure_gpi(dev, cfg::GPIConfig)

Map the GPI pin to a predefined function. Does NOT enable the GPI — follow
with `enable_gpi(dev)`.
"""
function configure_gpi(dev::DACDevice, cfg::GPIConfig)
    dev.c2.GPI_CONFIG = UInt8(cfg)
    flush_config2(dev)
    return nothing
end

"""
    configure_alarm_timing(dev; interburst = 0, pulse_off = 0, pulse_on = 0)

Program the medical-alarm timing fields in CONFIG2. Each value is 0..3 and
maps to different absolute times depending on alarm priority (Tables
8-8/8-9/8-10).
"""
function configure_alarm_timing(dev::DACDevice;
                                interburst::Integer = 0,
                                pulse_off::Integer  = 0,
                                pulse_on::Integer   = 0)
    all((0 <= x <= 3 for x in (interburst, pulse_off, pulse_on))) ||
        throw(ArgumentError("alarm timing fields must be 0..3"))
    dev.c2.INTERBURST_TIME = interburst
    dev.c2.PULSE_OFF_TIME  = pulse_off
    dev.c2.PULSE_ON_TIME   = pulse_on
    flush_config2(dev)
    return nothing
end

"""
    trigger_medical_alarm(dev, priority::AlarmPriority)

Start one of the IEC60601-1-8 medical-alarm tones. Mutually exclusive — the
other two alarm bits are cleared.
"""
function trigger_medical_alarm(dev::DACDevice, priority::AlarmPriority)
    dev.c2.MED_ALARM_HP = (priority == ALARM_HIGH)   ? 1 : 0
    dev.c2.MED_ALARM_MP = (priority == ALARM_MEDIUM) ? 1 : 0
    dev.c2.MED_ALARM_LP = (priority == ALARM_LOW)    ? 1 : 0
    flush_config2(dev)
    return nothing
end

"""
    stop_medical_alarm(dev)

Clear all three MED_ALARM_* bits. The device finishes the current burst
before going silent.
"""
function stop_medical_alarm(dev::DACDevice)
    dev.c2.MED_ALARM_HP = 0
    dev.c2.MED_ALARM_MP = 0
    dev.c2.MED_ALARM_LP = 0
    flush_config2(dev)
    return nothing
end

#=============================================================================
  TRIGGER actions
=============================================================================#

# Helper: build a TRIGGER with the given action bits, preserving GPI_EN by
# read-modify-writing the chip. SW_RESET reads back as 0b1000 ("no reset")
# so a plain RMW won't accidentally re-issue a reset.
function _trigger_action!(dev::DACDevice;
                          nvm_prog::Bool      = false,
                          nvm_reload::Bool    = false,
                          margin_low::Bool    = false,
                          margin_high::Bool   = false,
                          func_gen::Bool      = false,
                          config_reset::Bool  = false,
                          unlock::Bool        = false,
                          reset::Bool         = false)
    cur = read_reg(dev, TRIGGER_ADDR)
    trg = convert(TRIGGER, cur)
    # Preserve GPI_EN; everything else (SW_RESET reads as 0b1000, action bits
    # read 0 when idle) can be rewritten from desired flags.
    trg.SW_RESET            = reset ? SW_RESET_TRIGGER : SW_RESET_NOOP
    trg.NVM_PROG            = nvm_prog ? 1 : 0
    trg.NVM_RELOAD          = nvm_reload ? 1 : 0
    trg.PMBUS_MARGIN_LOW    = margin_low ? 1 : 0
    trg.PMBUS_MARGIN_HIGH   = margin_high ? 1 : 0
    trg.START_FUNC_GEN      = func_gen ? 1 : 0
    trg.DEVICE_CONFIG_RESET = config_reset ? 1 : 0
    # trg.GPI_EN preserved from RMW
    trg.DEVICE_UNLOCK_CODE  = unlock ? DEVICE_UNLOCK_CODE_VALUE : 0
    write_reg(dev, TRIGGER_ADDR, as_word(trg))
    return nothing
end

"""
    software_reset(dev)

Write SW_RESET = 0b1010 to issue a software reset, then sleep 35 ms for the
POR delay. All registers reload from NVM. The cached config structs are reset
to factory defaults; call `refresh_cache!` if your NVM holds non-default
values.
"""
function software_reset(dev::DACDevice)
    _trigger_action!(dev; reset = true)
    sleep(0.035)  # 30 ms POR delay + margin
    dev.gc = general_config_reset()
    dev.c2 = config2_reset()
    return nothing
end

"""
    device_config_reset(dev)

Reload factory-default register values without touching NVM. Refreshes the
local cache from the chip afterwards.
"""
function device_config_reset(dev::DACDevice)
    _trigger_action!(dev; config_reset = true)
    refresh_cache!(dev)
    return nothing
end

"""
    unlock_device(dev)

Write the 0b0101 unlock code to bypass DEVICE_LOCK. GPI_EN is preserved.
"""
unlock_device(dev::DACDevice) = _trigger_action!(dev; unlock = true)

"""
    program_nvm(dev; wait = true)

Snapshot the current register settings into NVM (EEPROM). Blocks until done
when `wait = true`.
"""
function program_nvm(dev::DACDevice; wait::Bool = true)
    _trigger_action!(dev; nvm_prog = true)
    if wait
        sleep(0.001)
        wait_nvm_idle(dev; timeout_s = 0.1)
    end
    return nothing
end

"""
    reload_nvm(dev; wait = true)

Reload registers from NVM. Refreshes the local cache when done.
"""
function reload_nvm(dev::DACDevice; wait::Bool = true)
    _trigger_action!(dev; nvm_reload = true)
    if wait
        sleep(0.001)
        wait_nvm_idle(dev; timeout_s = 0.1)
        refresh_cache!(dev)
    end
    return nothing
end

"""
    trigger_margin_high(dev)

Slew the DAC output to MARGIN_HIGH. The TRIGGER bit auto-clears at completion.
"""
trigger_margin_high(dev::DACDevice) = _trigger_action!(dev; margin_high = true)

"""
    trigger_margin_low(dev)

Slew the DAC output to MARGIN_LOW. The TRIGGER bit auto-clears at completion.
"""
trigger_margin_low(dev::DACDevice)  = _trigger_action!(dev; margin_low = true)

"""
    start_function_generator(dev, kind::FunctionType = TRIANGLE)

Begin continuous waveform generation between MARGIN_LOW and MARGIN_HIGH. Set
those margin codes and call `configure_slew` first to pick the frequency.

Frequency formulas (datasheet eqs. 3-5; SLEW period from `SLEW_PERIOD_SECONDS`):
- Square:   1 / (2 × SLEW)
- Triangle: 1 / (2 × SLEW × ((MH - ML + 1) / CODE_STEP))
- Sawtooth: 1 /     (SLEW × ((MH - ML + 1) / CODE_STEP))
"""
function start_function_generator(dev::DACDevice, kind::FunctionType = TRIANGLE)
    dev.gc.FUNC_CONFIG = UInt8(kind)
    flush_general_config(dev)
    _trigger_action!(dev; func_gen = true)
    return nothing
end

"""
    stop_function_generator(dev)

Clear START_FUNC_GEN. The DAC stops at the current code.
"""
stop_function_generator(dev::DACDevice) = _trigger_action!(dev)

#=============================================================================
  GPI enable/disable (the GPI_EN bit lives in TRIGGER but is sticky)
=============================================================================#

# Use a focused mask instead of going through _trigger_action! so we don't
# stomp on any in-flight one-shot bits.
const _M_GPI_EN = UInt16(1 << 10)

"""
    enable_gpi(dev)

Set TRIGGER.GPI_EN = 1.
"""
enable_gpi(dev::DACDevice)  = modify_reg!(dev, TRIGGER_ADDR, _M_GPI_EN, _M_GPI_EN)

"""
    disable_gpi(dev)
"""
disable_gpi(dev::DACDevice) = modify_reg!(dev, TRIGGER_ADDR, _M_GPI_EN, UInt16(0))

#=============================================================================
  PMBus
=============================================================================#

"""
    pmbus_operation(dev, op::PMBusOp)

Issue one of the PMBus OPERATION commands (Table 8-7). The command byte sits
in bits [15:8] of PMBUS_OPERATION; the low byte is don't-care. EN_PMBUS must
already be set — see `enable_pmbus(dev)`.
"""
function pmbus_operation(dev::DACDevice, op::PMBusOp)
    write_reg(dev, PMBUS_OPERATION_ADDR, UInt16(UInt8(op)) << 8)
    return nothing
end

"""
    pmbus_status(dev) -> PMBUS_STATUS_BYTE

Read PMBUS_STATUS_BYTE (78h).
"""
pmbus_status(dev::DACDevice) =
    convert(PMBUS_STATUS_BYTE, read_reg(dev, PMBUS_STATUS_BYTE_ADDR))

"""
    clear_pmbus_cml(dev)

Clear the CML (communication-fault) bit by writing 1 to it (per datasheet).
"""
function clear_pmbus_cml(dev::DACDevice)
    # PMBUS_STATUS_BYTE has two reserved (`_`) fields and one named field
    # (CML); FieldFlags' positional constructor skips the `_` entries.
    s = PMBUS_STATUS_BYTE(1)
    write_reg(dev, PMBUS_STATUS_BYTE_ADDR, as_word(s))
    return nothing
end

"""
    pmbus_version(dev) -> UInt8

Return the upper byte of PMBUS_VERSION (98h). The datasheet value is 0x22.
"""
pmbus_version(dev::DACDevice) =
    UInt8((read_reg(dev, PMBUS_VERSION_ADDR) >> 8) & 0xFF)

#=============================================================================
  Bulk slave-address configuration (datasheet §8.5.2.1.1)
=============================================================================#

"""
    set_dac_addresses(bus, pairs) -> Dict{UInt8, DACDevice}

Walk through datasheet §8.5.2.1.1 to assign distinct I²C slave addresses to up
to four DAC43701s sharing a bus, then snapshot each device's registers to NVM
so the new address survives power cycles.

Each entry of `pairs` is `desired_address => gpi_with`, where `desired_address`
is a 7-bit I²C address in 0x48..0x4B and `gpi_with(callback)` drives that
device's GPI line high, invokes the zero-arg `callback`, and then drives the
GPI line back low — for example:

    0x49 => function (cb)
        set_high(line)
        try; cb(); finally; set_low(line); end
    end

Returns a `Dict` mapping each assigned 7-bit address to a freshly-opened
`DACDevice`. The bulk steps (arming GPI, writing SLAVE_ADDRESS, restoring GPI
defaults) all go through the broadcast address 0x47, so the function assumes
every device's GPI pin is held low at entry and that CONFIG2 sits at its reset
value on every device — a broadcast CONFIG2 write rewrites the whole register,
so non-default alarm-timing fields would be clobbered.
"""
function set_dac_addresses(bus::Integer, pairs)
    for (addr, _) in pairs
        (UInt8(addr) >> 2) == 0x12 || throw(ArgumentError(
            "address 0x$(string(UInt8(addr), base=16, pad=2)) is not a valid " *
            "DAC43701 slave address (must be 0x48..0x4B)"))
    end

    bcast = DACDevice(I2C.open_device(Int(bus), DAC43701_ADDR_BROADCAST))
    result = Dict{UInt8, DACDevice}()
    try
        # Step 2: route GPI to the slave-address-program function on every
        # device.
        bcast.c2.GPI_CONFIG = UInt8(GPI_SLAVE_ADDR)
        flush_config2(bcast)

        # Step 3: arm GPI. Broadcast is write-only so we can't RMW TRIGGER;
        # write a fully specified value with every action bit cleared.
        write_reg(bcast, TRIGGER_ADDR,
                  as_word(TRIGGER(SW_RESET_NOOP, 0, 0, 0, 0, 0, 0, 1, 0)))

        # Steps 4-7: caller raises GPI on one device, we broadcast the new
        # SLAVE_ADDRESS (only that device latches it), caller drops GPI again.
        for (desired_addr, gpi_with) in pairs
            gpi_with() do
                bcast.c2.SLAVE_ADDRESS = UInt8(desired_addr) & 0x03
                flush_config2(bcast)
            end
        end

        # Step 8: GPI_EN back to 0.
        write_reg(bcast, TRIGGER_ADDR,
                  as_word(TRIGGER(SW_RESET_NOOP, 0, 0, 0, 0, 0, 0, 0, 0, 0)))
        # Step 9: restore GPI_CONFIG to its reset value.
        bcast.c2.GPI_CONFIG = UInt8(GPI_PD_HIZ)
        flush_config2(bcast)

        # Step 10: snapshot each device's registers into NVM. Done per-device
        # at its new address so we can poll NVM_BUSY — broadcast can't read.
        # `probe` confirms the address change actually landed before we touch
        # NVM; otherwise a missing/mis-addressed device would silently fall
        # through to the next bus participant.
        for (desired_addr, _) in pairs
            addr = UInt8(desired_addr)
            dev = DACDevice(I2C.open_device(Int(bus), addr))
            probe(dev)
            refresh_cache!(dev)
            program_nvm(dev)
            result[addr] = dev
        end
    finally
        close_dac(bcast)
    end
    return result
end

#=============================================================================
  Cleanup
=============================================================================#

"""
    close!(dev::DACDevice)

Close the I²C connection.
"""
close!(dev::DACDevice) = close_dac(dev)

#=============================================================================
  Exports (outer module)
=============================================================================#

export
    # Device
    DACDevice, open_dac, close_dac, close!, refresh_cache!,
    flush_general_config, flush_config2,
    # Low-level
    read_reg, write_reg, modify_reg!, as_word,
    # Status / probe
    probe, device_id, read_status,
    is_nvm_busy, is_dac_busy, wait_nvm_idle, wait_dac_idle,
    # DAC data
    set_dac_code, get_dac_code, set_margin_high, set_margin_low,
    # Reference / power
    configure_reference, power_up, power_down,
    # Slew
    configure_slew,
    # Reset / lock / NVM
    software_reset, device_config_reset, lock_device, unlock_device,
    program_nvm, reload_nvm,
    # Function generator
    start_function_generator, stop_function_generator,
    # Medical alarms
    trigger_medical_alarm, stop_medical_alarm, configure_alarm_timing,
    # GPI / margin
    configure_gpi, enable_gpi, disable_gpi,
    trigger_margin_high, trigger_margin_low,
    # PMBus
    enable_pmbus, pmbus_operation, pmbus_status, clear_pmbus_cml, pmbus_version,
    # Bulk slave-address assignment
    set_dac_addresses

end # module DAC43701
