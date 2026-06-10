module ICM42688PC

using ..SPI
using FieldFlags

"""
    Registers

FieldFlags.jl bit-layout descriptions for the registers of the Tokmas ICM-42688-PC
6-axis IMU (3-axis gyro + 3-axis accel + temp sensor + motion engines + FIFO).

Conventions
-----------
* All structs follow FieldFlags.jl's LSB-first layout: the FIRST field written
  occupies the LEAST significant bit. This matches how the datasheet numbers
  individual bits within each register (bit 0 = LSB).
* `_` / `_:n` is reserved/padding, exactly as defined in the datasheet.
* Configuration registers are declared `mutable` because the host writes to them.
  Status / read-only registers are non-mutable.
* The 8-bit "value" registers (WHO_AM_I, REVISION_ID, FIFO_WTM_TH, FIFO_SMPL_CNT,
  FIFO_DATA, CAL1_L … CAL4_H, RESET, CTRL9) and the 16-bit two's-complement data
  output registers (TEMP_[H,L], A[X,Y,Z]_[H,L], G[X,Y,Z]_[H,L], dQW_*, dV*_*,
  TIMESTAMP_*) are not bit-packed and so do not get FieldFlags wrappers — they
  are plain unsigned/signed integers. Constants for command codes, ODR settings,
  full-scale settings, and the special RESET value are provided at the bottom.

Reference: ICM-42688-PC datasheet (tokmas.com).
"""
module Registers

using FieldFlags

#=============================================================================
  General Purpose Registers (read-only IDs — plain 8-bit values, no bitfield)
=============================================================================#

const WHO_AM_I_ADDR    = 0x00  # default 0x05
const REVISION_ID_ADDR = 0x01  # default 0x7C

#=============================================================================
  CTRL1 — Serial Interface and Sensor Enable @ 0x02 (default 0x20)
=============================================================================#

# CTRL1: serial interface and sensor enable. Default `0x20` — i.e. `BE = 1`,
# all other bits 0. (FieldFlags-generated structs reject docstrings.)
#
# | bit | name         | meaning                                                  |
# |-----|--------------|----------------------------------------------------------|
# | 0   | SensorDisable| 0 = internal high-speed oscillator on, 1 = off (low-pwr) |
# | 1   | (reserved)   |                                                          |
# | 2   | FIFO_INT_SEL | 0 = FIFO interrupt → INT2, 1 = → INT1                    |
# | 3   | INT1_EN      | 0 = INT1 high-Z, 1 = INT1 output enabled                 |
# | 4   | INT2_EN      | 0 = INT2 high-Z, 1 = INT2 output enabled                 |
# | 5   | BE           | 0 = little-endian reads, 1 = big-endian reads            |
# | 6   | ADDR_AI      | 0 = address non-increment, 1 = auto-increment            |
# | 7   | SIM          | 0 = 4-wire SPI, 1 = 3-wire SPI                           |
@bitfield mutable struct CTRL1
    SensorDisable:1
    _:1               # reserved
    FIFO_INT_SEL:1
    INT1_EN:1
    INT2_EN:1
    BE:1
    ADDR_AI:1
    SIM:1
end

#=============================================================================
  CTRL2 — Accelerometer settings @ 0x03 (default 0x00)
=============================================================================#

# CTRL2: accelerometer ODR, full-scale and self-test.
#
# | bits | name       | meaning                                                |
# |------|------------|--------------------------------------------------------|
# | 3:0  | aODR       | output data rate (see `ACCEL_ODR_*` constants)         |
# | 6:4  | aFS        | full-scale: 0=±2g, 1=±4g, 2=±8g, 3=±16g (4–7 = N/A)    |
# | 7    | aST        | 0 = disable accel self-test, 1 = enable                |
@bitfield mutable struct CTRL2
    aODR:4
    aFS:3
    aST:1
end

#=============================================================================
  CTRL3 — Gyroscope settings @ 0x04 (default 0x00)
=============================================================================#

# CTRL3: gyroscope ODR, full-scale and self-test.
#
# | bits | name | meaning                                                          |
# |------|------|------------------------------------------------------------------|
# | 3:0  | gODR | output data rate (see `GYRO_ODR_*` constants)                    |
# | 6:4  | gFS  | full-scale: 0=±16, 1=±32, 2=±64, 3=±128, 4=±256, 5=±512, 6=±1024,|
# |      |      | 7=±2048 dps                                                      |
# | 7    | gST  | 0 = disable gyro self-test, 1 = enable                           |
@bitfield mutable struct CTRL3
    gODR:4
    gFS:3
    gST:1
end

#=============================================================================
  CTRL5 — Low-pass filter settings @ 0x06 (default 0x00)
=============================================================================#

# CTRL5: accelerometer and gyroscope low-pass filter configuration.
#
# | bits | name      | meaning                                                |
# |------|-----------|--------------------------------------------------------|
# | 0    | aLPF_EN   | 0 = accel LPF disabled, 1 = enabled                    |
# | 2:1  | aLPF_MODE | accel LPF bandwidth: 0=2.66%, 1=3.63%, 2=5.39%, 3=13.37%|
# | 3    | (rsvd)    |                                                        |
# | 4    | gLPF_EN   | 0 = gyro LPF disabled, 1 = enabled                     |
# | 6:5  | gLPF_MODE | gyro  LPF bandwidth: 0=2.66%, 1=3.63%, 2=5.39%, 3=13.37%|
# | 7    | (rsvd)    |                                                        |
@bitfield mutable struct CTRL5
    aLPF_EN:1
    aLPF_MODE:2
    _:1               # reserved
    gLPF_EN:1
    gLPF_MODE:2
    _:1               # reserved
end

#=============================================================================
  CTRL7 — Enable Sensors and Configure Data Reads @ 0x08 (default 0x00)
=============================================================================#

# CTRL7: master sensor enable, snooze, data-ready disable and SyncSample mode.
#
# | bits | name       | meaning                                            |
# |------|------------|----------------------------------------------------|
# | 0    | aEN        | 0 = accel disabled, 1 = enabled                    |
# | 1    | gEN        | 0 = gyro disabled, 1 = enabled                     |
# | 3:2  | (reserved) |                                                    |
# | 4    | gSN        | 0 = gyro full mode, 1 = snooze (drive only)        |
# | 5    | DRDY_DIS   | 0 = DRDY driven to INT2, 1 = blocked               |
# | 6    | (reserved) |                                                    |
# | 7    | SyncSample | 0 = disable SyncSample mode, 1 = enable            |
@bitfield mutable struct CTRL7
    aEN:1
    gEN:1
    _:2               # reserved
    gSN:1
    DRDY_DIS:1
    _:1               # reserved
    SyncSample:1
end

#=============================================================================
  CTRL8 — Motion Detection Control @ 0x09 (default 0x00) — all bits are 1-bit
=============================================================================#

# CTRL8: motion-engine enables and CTRL9 handshake routing. All fields are
# single bits, so `@bitflags` is the natural choice. (`@bitflags`-generated
# structs reject docstrings, so this layout table lives as a plain comment.)
#
# | bit | name                  | meaning                                       |
# |-----|-----------------------|-----------------------------------------------|
# | 0   | Tap_EN                | enable Tap engine                             |
# | 1   | AnyMotion_EN          | enable Any-Motion engine                      |
# | 2   | NoMotion_EN           | enable No-Motion engine                       |
# | 3   | SigMotion_EN          | enable Significant-Motion engine              |
# | 4   | (reserved)            |                                               |
# | 5   | (reserved)            |                                               |
# | 6   | ACTIVITY_INT_SEL      | 0 = motion events → INT2, 1 = → INT1          |
# | 7   | CTRL9_HandShake_Type  | 0 = INT1 used as CTRL9 handshake,             |
# |     |                       | 1 = poll STATUSINT.bit7                       |
@bitflags mutable struct CTRL8
    Tap_EN
    AnyMotion_EN
    NoMotion_EN
    SigMotion_EN
    _                 # reserved
    _                 # reserved
    ACTIVITY_INT_SEL
    CTRL9_HandShake_Type
end

#=============================================================================
  FIFO_CTRL — FIFO Control @ 0x14 (default 0x00)
=============================================================================#

# FIFO_CTRL: FIFO mode/size selector and FIFO read-mode bit.
#
# | bits | name         | meaning                                              |
# |------|--------------|------------------------------------------------------|
# | 1:0  | FIFO_MODE    | 0=bypass, 1=FIFO, 2=Stream, 3=reserved               |
# | 3:2  | FIFO_SIZE    | 0=16, 1=32, 2=64, 3=128 samples                      |
# | 6:4  | (reserved)   |                                                      |
# | 7    | FIFO_RD_MODE | 0=FIFO write mode, 1=FIFO read mode                  |
@bitfield mutable struct FIFO_CTRL
    FIFO_MODE:2
    FIFO_SIZE:2
    _:3               # reserved
    FIFO_RD_MODE:1
end

#=============================================================================
  FIFO_STATUS — FIFO Status @ 0x16 (read-only, default 0x00)
=============================================================================#

# FIFO_STATUS: FIFO status flags. The two LS bits are the MSBs of the 10-bit
# FIFO sample count (the low 8 bits live in `FIFO_SMPL_CNT` @ 0x15).
#
# | bits | name              | meaning                                         |
# |------|-------------------|-------------------------------------------------|
# | 1:0  | FIFO_SMPL_CNT_MSB | upper 2 bits of FIFO sample count (in words)    |
# | 3:2  | (reserved)        |                                                 |
# | 4    | FIFO_NOT_EMPTY    | 0 = empty, 1 = not empty                        |
# | 5    | FIFO_OVFLOW       | 1 = overflow occurred (data dropped)            |
# | 6    | FIFO_WTM          | 1 = watermark level reached                     |
# | 7    | FIFO_FULL         | 1 = FIFO full                                   |
@bitfield struct FIFO_STATUS
    FIFO_SMPL_CNT_MSB:2
    _:2               # reserved
    FIFO_NOT_EMPTY:1
    FIFO_OVFLOW:1
    FIFO_WTM:1
    FIFO_FULL:1
end

#=============================================================================
  STATUSINT — Sensor Data Available / Locking @ 0x2D (read-only)
=============================================================================#

# STATUSINT: sensor-data-available and CTRL9 handshake. When `SyncSample` in
# CTRL7 is 0, `Avail` mirrors INT2 level and `Locked` mirrors INT1 level.
#
# | bit | name        | meaning                                              |
# |-----|-------------|------------------------------------------------------|
# | 0   | Avail       | sensor data available (or INT2 mirror if !syncSmpl)  |
# | 1   | Locked      | sensor data locked (or INT1 mirror if !syncSmpl)     |
# | 6:2 | (reserved)  |                                                      |
# | 7   | CmdDone     | 1 = CTRL9 command completed                          |
@bitfield struct STATUSINT
    Avail:1
    Locked:1
    _:5               # reserved
    CmdDone:1
end

#=============================================================================
  STATUS0 — Output Data Status @ 0x2E (read-only)
=============================================================================#

# STATUS0: per-sensor new-data flags.
#
# | bit | name        | meaning                                              |
# |-----|-------------|------------------------------------------------------|
# | 0   | aDA         | 1 = new accel data since last read                   |
# | 1   | gDA         | 1 = new gyro  data since last read                   |
# | 7:2 | (reserved)  |                                                      |
@bitfield struct STATUS0
    aDA:1
    gDA:1
    _:6               # reserved
end

#=============================================================================
  STATUS1 — Miscellaneous (motion + tap) status @ 0x2F (read-only)
=============================================================================#

# STATUS1: tap and motion-engine event flags. The datasheet leaves several bits
# reserved; only the named ones are meaningful.
#
# | bit | name              | meaning                              |
# |-----|-------------------|--------------------------------------|
# | 0   | (reserved)        |                                      |
# | 1   | TAP               | 1 = Tap (single or double) detected  |
# | 4:2 | (reserved)        |                                      |
# | 5   | AnyMotion         | 1 = Any-Motion detected              |
# | 6   | NoMotion          | 1 = No-Motion detected               |
# | 7   | SignificantMotion | 1 = Significant-Motion detected      |
@bitfield struct STATUS1
    _:1               # reserved
    TAP:1
    _:3               # reserved
    AnyMotion:1
    NoMotion:1
    SignificantMotion:1
end

#=============================================================================
  COD_STATUS — Calibration-On-Demand result @ 0x46 (read-only)
=============================================================================#

# COD_STATUS: Calibration-On-Demand result for gyro X/Y sensitivity. 0x00 means
# COD succeeded; any non-zero value indicates a failure mode. All fields are
# single bits. (`@bitflags`-generated structs reject docstrings.)
#
# | bit | name             | meaning (1 = failure unless noted)            |
# |-----|------------------|-----------------------------------------------|
# | 0   | COD_Failed       | 0 = COD ok, new gains applied                 |
# | 1   | Gyro_Enabled     | COD called while gyro was enabled — illegal   |
# | 2   | Startup_Failed   | gyro startup failure during COD               |
# | 3   | Accel_Check      | accel saw significant vibration during COD    |
# | 4   | Y_Limit_H_Fail   | Y high-sensitivity-limit check failed         |
# | 5   | Y_Limit_L_Fail   | Y low-sensitivity-limit check failed          |
# | 6   | X_Limit_H_Fail   | X high-sensitivity-limit check failed         |
# | 7   | X_Limit_L_Fail   | X low-sensitivity-limit check failed          |
@bitflags struct COD_STATUS
    COD_Failed
    Gyro_Enabled
    Startup_Failed
    Accel_Check
    Y_Limit_H_Fail
    Y_Limit_L_Fail
    X_Limit_H_Fail
    X_Limit_L_Fail
end

#=============================================================================
  TAP_STATUS — Tap detection result @ 0x59 (read-only)
=============================================================================#

# TAP_STATUS: tap-engine output.
#
# | bits | name         | meaning                                          |
# |------|--------------|--------------------------------------------------|
# | 1:0  | TAP_NUM      | 0 = no tap, 1 = single, 2 = double, 3 = N/A      |
# | 3:2  | (reserved)   |                                                  |
# | 5:4  | TAP_AXIS     | 0 = none, 1 = X, 2 = Y, 3 = Z                    |
# | 6    | (reserved)   |                                                  |
# | 7    | TAP_POLARITY | 0 = tap toward + direction, 1 = toward −         |
@bitfield struct TAP_STATUS
    TAP_NUM:2
    _:2               # reserved
    TAP_AXIS:2
    _:1               # reserved
    TAP_POLARITY:1
end

#=============================================================================
  Auxiliary bytes that travel through the CTRL9 / CAL register protocol.
  These aren't "registers" per se — they're the structured payloads written
  into CAL1_L (etc.) before issuing a CTRL_CMD_SET_RPU or
  CTRL_CMD_CONFIGURE_MOTION CTRL9 command. They have clean bit layouts, so
  wrapping them with FieldFlags is convenient.
=============================================================================#

# RPU_CTRL: payload for `CTRL_CMD_SET_RPU` (CTRL9 command 0x11), written into
# `CAL1_L`. By default all pull-ups are enabled (0); writing 1 to a bit disables
# the corresponding pull-ups. (`@bitflags`-generated structs reject docstrings.)
#
# | bit | name        | pins affected           |
# |-----|-------------|-------------------------|
# | 0   | aux_rpu_dis | SDx, SCx, RESV (pin 10) |
# | 1   | icm_rpu_dis | SDx                     |
# | 2   | cs_rpu_dis  | CS                      |
# | 3   | i2c_rpu_dis | SCL, SDA                |
# | 7:4 | (reserved)  |                         |
@bitflags mutable struct RPU_CTRL
    aux_rpu_dis
    icm_rpu_dis
    cs_rpu_dis
    i2c_rpu_dis
    _
    _
    _
    _
end

# MOTION_MODE_CTRL: per-axis enables and AND/OR logic for the motion engines.
# Loaded into `CAL4_L` during the first half of the `CTRL_CMD_CONFIGURE_MOTION`
# two-step CTRL9 sequence (datasheet Table 35); byte layout per Table 34.
# (`@bitflags`-generated structs reject docstrings.)
#
# | bit | name               | meaning                                |
# |-----|--------------------|----------------------------------------|
# | 0   | AnyMotionEnX       | enable X axis for Any-Motion           |
# | 1   | AnyMotionEnY       | enable Y axis for Any-Motion           |
# | 2   | AnyMotionEnZ       | enable Z axis for Any-Motion           |
# | 3   | AnyMotionAxisLogic | 0 = OR, 1 = AND across enabled axes    |
# | 4   | NoMotionEnX        | enable X axis for No-Motion            |
# | 5   | NoMotionEnY        | enable Y axis for No-Motion            |
# | 6   | NoMotionEnZ        | enable Z axis for No-Motion            |
# | 7   | NoMotionAxisLogic  | 0 = OR, 1 = AND across enabled axes    |
@bitflags mutable struct MOTION_MODE_CTRL
    AnyMotionEnX
    AnyMotionEnY
    AnyMotionEnZ
    AnyMotionAxisLogic
    NoMotionEnX
    NoMotionEnY
    NoMotionEnZ
    NoMotionAxisLogic
end

#=============================================================================
  Register addresses
=============================================================================#

const CTRL1_ADDR         = 0x02
const CTRL2_ADDR         = 0x03
const CTRL3_ADDR         = 0x04
const CTRL5_ADDR         = 0x06
const CTRL7_ADDR         = 0x08
const CTRL8_ADDR         = 0x09
const CTRL9_ADDR         = 0x0A     # write CTRL9 command codes here

const CAL1_L_ADDR        = 0x0B
const CAL1_H_ADDR        = 0x0C
const CAL2_L_ADDR        = 0x0D
const CAL2_H_ADDR        = 0x0E
const CAL3_L_ADDR        = 0x0F
const CAL3_H_ADDR        = 0x10
const CAL4_L_ADDR        = 0x11
const CAL4_H_ADDR        = 0x12

const FIFO_WTM_TH_ADDR   = 0x13
const FIFO_CTRL_ADDR     = 0x14
const FIFO_SMPL_CNT_ADDR = 0x15
const FIFO_STATUS_ADDR   = 0x16
const FIFO_DATA_ADDR     = 0x17

const STATUSINT_ADDR     = 0x2D
const STATUS0_ADDR       = 0x2E
const STATUS1_ADDR       = 0x2F

const TIMESTAMP_LOW      = 0x30
const TIMESTAMP_MID      = 0x31
const TIMESTAMP_HIGH     = 0x32

# 16-bit two's-complement data (low byte then high byte). Not bit-packed.
const TEMP_L = 0x33; const TEMP_H = 0x34
const AX_L   = 0x35; const AX_H   = 0x36
const AY_L   = 0x37; const AY_H   = 0x38
const AZ_L   = 0x39; const AZ_H   = 0x3A
const GX_L   = 0x3B; const GX_H   = 0x3C
const GY_L   = 0x3D; const GY_H   = 0x3E
const GZ_L   = 0x3F; const GZ_H   = 0x40

const dQW_L = 0x49; const dQW_H = 0x4A;
const dQX_L = 0x4B; const dQX_H = 0x4C;
const dQY_L = 0x4D; const dQY_H = 0x4E;
const dQZ_L = 0x4F; const dQZ_H = 0x50;
const dVX_L = 0x51; const dVX_H = 0x52;
const dVY_L = 0x53; const dVY_H = 0x54;
const dVZ_L = 0x55; const dVZ_H = 0x56;

const COD_STATUS_ADDR    = 0x46
const TAP_STATUS_ADDR    = 0x59
const RESET_ADDR         = 0x60

#=============================================================================
  ICM-42688-PC (Tokmas) constants and enums
=============================================================================#

# WHO_AM_I value for the Tokmas ICM-42688-PC.
# (Note: the original TDK ICM-42688-P returns 0x47 at register 0x75 — different part.)
const ICM42688PC_WHO_AM_I_VALUE = 0x05
const ICM42688PC_REVISION_ID    = 0x7C

# SPI read flag: bit 7 = 1 for read (same convention as the TDK part)
const SPI_READ_FLAG = UInt8(0x80)

# Soft-reset magic value (written to RESET register 0x60)
const RESET_MAGIC = UInt8(0xB0)

# CTRL9 command codes (Table 29).
@enum Ctrl9Cmd::UInt8 begin
    CTRL_CMD_ACK                     = 0x00  # host ack to end the protocol
    CTRL_CMD_RST_FIFO                = 0x04
    CTRL_CMD_REQ_FIFO                = 0x05
    CTRL_CMD_ACCEL_HOST_DELTA_OFFSET = 0x09
    CTRL_CMD_GYRO_HOST_DELTA_OFFSET  = 0x0A
    CTRL_CMD_CONFIGURE_TAP           = 0x0C
    CTRL_CMD_CONFIGURE_MOTION        = 0x0E
    CTRL_CMD_COPY_USID               = 0x10
    CTRL_CMD_SET_RPU                 = 0x11
    CTRL_CMD_AHB_CLOCK_GATING        = 0x12
    CTRL_CMD_ON_DEMAND_CALIBRATION   = 0xA2
    CTRL_CMD_APPLY_GYRO_GAINS        = 0xAA
end

# Gyroscope full-scale range (CTRL3 bits 6:4 = gFS<2:0>)
@enum GyroRange::UInt8 begin
    GYRO_FS_16DPS   = 0x00  # ±16   dps -> 2048 LSB/dps
    GYRO_FS_32DPS   = 0x01  # ±32   dps -> 1024 LSB/dps
    GYRO_FS_64DPS   = 0x02  # ±64   dps -> 512  LSB/dps
    GYRO_FS_128DPS  = 0x03  # ±128  dps -> 256  LSB/dps
    GYRO_FS_256DPS  = 0x04  # ±256  dps -> 128  LSB/dps
    GYRO_FS_512DPS  = 0x05  # ±512  dps -> 64   LSB/dps
    GYRO_FS_1024DPS = 0x06  # ±1024 dps -> 32   LSB/dps
    GYRO_FS_2048DPS = 0x07  # ±2048 dps -> 16   LSB/dps
end

# Accelerometer full-scale range (CTRL2 bits 6:4 = aFS<2:0>)
@enum AccelRange::UInt8 begin
    ACCEL_FS_2G  = 0x00  # ±2  g -> 16384 LSB/g
    ACCEL_FS_4G  = 0x01  # ±4  g -> 8192  LSB/g
    ACCEL_FS_8G  = 0x02  # ±8  g -> 4096  LSB/g
    ACCEL_FS_16G = 0x03  # ±16 g -> 2048  LSB/g
end

# Gyroscope ODR (CTRL3 bits 3:0). All rates require the gyro nature-frequency clock.
@enum GyroODR::UInt8 begin
    GYRO_ODR_7174_4HZ = 0x00
    GYRO_ODR_3587_2HZ = 0x01
    GYRO_ODR_1793_6HZ = 0x02
    GYRO_ODR_896_8HZ  = 0x03
    GYRO_ODR_448_4HZ  = 0x04
    GYRO_ODR_224_2HZ  = 0x05
    GYRO_ODR_112_1HZ  = 0x06
    GYRO_ODR_56_05HZ  = 0x07
    GYRO_ODR_28_025HZ = 0x08
end

# Accelerometer ODR (CTRL2 bits 3:0). 0x00-0x08 are valid in 6DOF mode and pick from
# the gyro-derived ODR table; in accel-only the same codes pick the internal-osc table.
# 0x0C-0x0F are low-power modes (accel only, gyro must be disabled).
@enum AccelODR::UInt8 begin
    # Codes 0x00..0x02 are only valid when the gyroscope is also enabled (6DOF).
    ACCEL_ODR_7174_4HZ_6DOF = 0x00
    ACCEL_ODR_3587_2HZ_6DOF = 0x01
    ACCEL_ODR_1793_6HZ_6DOF = 0x02
    # Codes 0x03..0x08: accel-only ODR shown; in 6DOF the gyro-derived rate applies.
    ACCEL_ODR_1000HZ   = 0x03  # 6DOF: 896.8 Hz
    ACCEL_ODR_500HZ    = 0x04  # 6DOF: 448.4 Hz
    ACCEL_ODR_250HZ    = 0x05  # 6DOF: 224.2 Hz
    ACCEL_ODR_125HZ    = 0x06  # 6DOF: 112.1 Hz
    ACCEL_ODR_62_5HZ   = 0x07  # 6DOF: 56.05 Hz
    ACCEL_ODR_31_25HZ  = 0x08  # 6DOF: 28.025 Hz
    # Low-power codes: accel-only, gyro must be disabled.
    ACCEL_ODR_128HZ_LP = 0x0C
    ACCEL_ODR_21HZ_LP  = 0x0D
    ACCEL_ODR_11HZ_LP  = 0x0E
    ACCEL_ODR_3HZ_LP   = 0x0F
end

# Low-pass filter mode for accel and gyro (CTRL5 bits 2:1 and 6:5).
# Bandwidth is expressed as a percentage of the active ODR.
@enum LpfMode::UInt8 begin
    LPF_MODE_2_66  = 0x00  # 2.66 % of ODR
    LPF_MODE_3_63  = 0x01  # 3.63 % of ODR
    LPF_MODE_5_39  = 0x02  # 5.39 % of ODR
    LPF_MODE_13_37 = 0x03  # 13.37 % of ODR
end

# Sensitivity scale factors (LSB per unit), per datasheet Tables 8 and 9.
const GYRO_SCALE = Dict(
    GYRO_FS_16DPS   => 2048.0,
    GYRO_FS_32DPS   => 1024.0,
    GYRO_FS_64DPS   =>  512.0,
    GYRO_FS_128DPS  =>  256.0,
    GYRO_FS_256DPS  =>  128.0,
    GYRO_FS_512DPS  =>   64.0,
    GYRO_FS_1024DPS =>   32.0,
    GYRO_FS_2048DPS =>   16.0,
)

const ACCEL_SCALE = Dict(
    ACCEL_FS_2G  => 16384.0,
    ACCEL_FS_4G  =>  8192.0,
    ACCEL_FS_8G  =>  4096.0,
    ACCEL_FS_16G =>  2048.0,
)

# I²C / I³C slave address selection via the SA0 pin / pull (datasheet §1.5).
const I2C_ADDR_SA0_HIGH = 0x6A   # SA0 floats or pulled high
const I2C_ADDR_SA0_LOW  = 0x6B   # SA0 pulled low

#=============================================================================
  Exports
=============================================================================#

export
    # FieldFlags struct types
    CTRL1, CTRL2, CTRL3, CTRL5, CTRL7, CTRL8,
    FIFO_CTRL, FIFO_STATUS,
    STATUSINT, STATUS0, STATUS1,
    COD_STATUS, TAP_STATUS,
    RPU_CTRL, MOTION_MODE_CTRL,
    # Register addresses
    WHO_AM_I_ADDR, REVISION_ID_ADDR,
    CTRL1_ADDR, CTRL2_ADDR, CTRL3_ADDR, CTRL5_ADDR,
    CTRL7_ADDR, CTRL8_ADDR, CTRL9_ADDR,
    CAL1_L_ADDR, CAL1_H_ADDR, CAL2_L_ADDR, CAL2_H_ADDR,
    CAL3_L_ADDR, CAL3_H_ADDR, CAL4_L_ADDR, CAL4_H_ADDR,
    FIFO_WTM_TH_ADDR, FIFO_CTRL_ADDR, FIFO_SMPL_CNT_ADDR,
    FIFO_STATUS_ADDR, FIFO_DATA_ADDR,
    STATUSINT_ADDR, STATUS0_ADDR, STATUS1_ADDR,
    TIMESTAMP_LOW, TIMESTAMP_MID, TIMESTAMP_HIGH,
    TEMP_L, TEMP_H,
    AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H,
    GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H,
    COD_STATUS_ADDR, TAP_STATUS_ADDR, RESET_ADDR,
    # Constants
    RESET_MAGIC,
    ICM42688PC_WHO_AM_I_VALUE, ICM42688PC_REVISION_ID,
    SPI_READ_FLAG,
    GYRO_SCALE, ACCEL_SCALE,
    I2C_ADDR_SA0_HIGH, I2C_ADDR_SA0_LOW,
    # Enums and their members
    Ctrl9Cmd,
    CTRL_CMD_ACK, CTRL_CMD_RST_FIFO, CTRL_CMD_REQ_FIFO,
    CTRL_CMD_ACCEL_HOST_DELTA_OFFSET, CTRL_CMD_GYRO_HOST_DELTA_OFFSET,
    CTRL_CMD_CONFIGURE_TAP, CTRL_CMD_CONFIGURE_MOTION,
    CTRL_CMD_COPY_USID, CTRL_CMD_SET_RPU,
    CTRL_CMD_AHB_CLOCK_GATING,
    CTRL_CMD_ON_DEMAND_CALIBRATION, CTRL_CMD_APPLY_GYRO_GAINS,
    GyroRange,
    GYRO_FS_16DPS, GYRO_FS_32DPS, GYRO_FS_64DPS, GYRO_FS_128DPS,
    GYRO_FS_256DPS, GYRO_FS_512DPS, GYRO_FS_1024DPS, GYRO_FS_2048DPS,
    AccelRange,
    ACCEL_FS_2G, ACCEL_FS_4G, ACCEL_FS_8G, ACCEL_FS_16G,
    GyroODR,
    GYRO_ODR_7174_4HZ, GYRO_ODR_3587_2HZ, GYRO_ODR_1793_6HZ,
    GYRO_ODR_896_8HZ, GYRO_ODR_448_4HZ, GYRO_ODR_224_2HZ,
    GYRO_ODR_112_1HZ, GYRO_ODR_56_05HZ, GYRO_ODR_28_025HZ,
    AccelODR,
    ACCEL_ODR_7174_4HZ_6DOF, ACCEL_ODR_3587_2HZ_6DOF, ACCEL_ODR_1793_6HZ_6DOF,
    ACCEL_ODR_1000HZ, ACCEL_ODR_500HZ, ACCEL_ODR_250HZ, ACCEL_ODR_125HZ,
    ACCEL_ODR_62_5HZ, ACCEL_ODR_31_25HZ,
    ACCEL_ODR_128HZ_LP, ACCEL_ODR_21HZ_LP, ACCEL_ODR_11HZ_LP, ACCEL_ODR_3HZ_LP,
    LpfMode,
    LPF_MODE_2_66, LPF_MODE_3_63, LPF_MODE_5_39, LPF_MODE_13_37

export dQW_L, dQX_L, dQY_L, dQZ_L, dVX_L, dVY_L, dVZ_L, dQW_H, dQX_H, dQY_H, dQZ_H, dVX_H, dVY_H, dVZ_H

end # module Registers

using .Registers

# Temperature: register output is 16-bit two's complement at 256 LSB/°C.
# T (°C) = raw / 256
const TEMP_SCALE = 256.0

#=============================================================================
  Sensor data struct
=============================================================================#

"""
    ICM42688PCData

Raw sensor readings from the ICM-42688-PC. All values are raw 16-bit signed
integers directly from sensor registers, two's-complement.
"""
struct ICM42688PCData
    temp::Int16
    accel_x::Int16
    accel_y::Int16
    accel_z::Int16
    gyro_x::Int16
    gyro_y::Int16
    gyro_z::Int16
end

#=============================================================================
  Device struct
=============================================================================#

"""
    ICM42688PC

Wrapper for the Tokmas ICM-42688-PC IMU over SPI.

# Fields
- `device::SPI.SPIDevice`: SPI device handle.
- `gyro_range::GyroRange`:  current gyro full-scale setting (cached for scaling).
- `accel_range::AccelRange`: current accel full-scale setting.
- `ctrl::CTRL1`: the configuration of ctrl1 used to initialize the IMU.
- `tx_buf::Vector{UInt8}`: pre-allocated TX buffer.
- `rx_buf::Vector{UInt8}`: pre-allocated RX buffer.
"""
mutable struct ICM42688PC
    device::SPI.SPIDevice
    gyro_bias::NTuple{3, Float64}
    gyro_range::GyroRange
    accel_range::AccelRange
    ctrl1::CTRL1
    tx_buf::Vector{UInt8}
    rx_buf::Vector{UInt8}
end

"""
    ICM42688PC(device::SPI.SPIDevice)

Create an ICM42688PC instance with default range settings (±2048 dps gyro, ±16 g
accel). Call `initialize!` to actually configure the chip.

Buffer is sized for the largest burst we do in this driver: 1 address byte +
14 data bytes (TEMP + AX..AZ + GX..GZ = 14 bytes) = 15 bytes total.
"""
function ICM42688PC(device::SPI.SPIDevice, ctrl1::CTRL1 = CTRL1(
        false, false, false, 
        false, true, true, # enable big endian and autoincrement by default
        false))
    imu = ICM42688PC(device, (0.0, 0.0, 0.0), GYRO_FS_2048DPS, ACCEL_FS_16G, ctrl1,
                      zeros(UInt8, 15), zeros(UInt8, 15))
    write_reg(imu, CTRL1_ADDR, as_byte(ctrl1))
    return imu
end

"""
    ICM42688PC(bus::Int, cs::Int; speed_hz=1_000_000)

Open an SPI connection and wrap it. The ICM-42688-PC accepts SPI Mode 0 or 3
and auto-detects; up to 15 MHz per datasheet Table 40.
"""
function ICM42688PC(bus::Int, cs::Int; speed_hz::UInt32=UInt32(15_000_000), ctrl1::CTRL1 = CTRL1(
        false, false, false, 
        false, true, true, # enable big endian and autoincrement by default
        false))
    device = SPI.SPIDevice(bus, cs; mode=SPI.SPI_MODE_0, speed_hz=speed_hz)
    imu = ICM42688PC(device, ctrl1)
    return imu
end

function set_gyro_bias(imu::ICM42688PC, bias::NTuple{3, Float64})
    imu.gyro_bias = bias
end

#=============================================================================
  Low-level SPI register operations
=============================================================================#

"""
    write_reg(imu::ICM42688PC, reg, value::UInt8)

Write a single byte to the specified register.
SPI write frame: `[reg & 0x7F, value]`.

Note: per datasheet §15.1, burst writes to configuration registers (CTRL1..CTRL9)
are NOT supported — use single-byte writes for those.
"""
function write_reg(imu::ICM42688PC, reg::Integer, value::UInt8)
    imu.tx_buf[1] = UInt8(reg) & 0x7F  # bit 7 = 0 for write
    imu.tx_buf[2] = value
    return SPI.transfer!(imu.device, @view(imu.tx_buf[1:2]), @view(imu.rx_buf[1:2]))
end

"""
    read_reg(imu::ICM42688PC, reg) -> UInt8

Read a single byte from the specified register.
SPI read frame: `[0x80 | reg, 0x00]`, with data on the second clocked byte.
"""
function read_reg(imu::ICM42688PC, reg::Integer)
    imu.tx_buf[1] = UInt8(reg) | SPI_READ_FLAG
    imu.tx_buf[2] = 0x00
    SPI.transfer!(imu.device, @view(imu.tx_buf[1:2]), @view(imu.rx_buf[1:2]))
    return imu.rx_buf[2]
end

"""
    read_regs!(imu::ICM42688PC, reg, count::Int) -> Nothing

Burst read `count` bytes starting from `reg`. Requires `CTRL1.ADDR_AI = 1`
to be set, otherwise the same register is returned `count` times.

Data lands in `imu.rx_buf[2 : count+1]`; index 1 is the discarded address echo.
"""
function read_regs!(imu::ICM42688PC, reg::Integer, count::Int)
    @assert imu.ctrl1.ADDR_AI
    n = count + 1
    imu.tx_buf[1] = UInt8(reg) | SPI_READ_FLAG
    fill!(@view(imu.tx_buf[2:n]), 0x00)
    SPI.transfer!(imu.device, @view(imu.tx_buf[1:n]), @view(imu.rx_buf[1:n]))
    return nothing
end

#=============================================================================
  Helpers: bit fields and 16-bit parsing
=============================================================================#

# FieldFlags structs wrap a primitive type stored in their internal `:fields`
# slot, which is the same size as the original register. `as_byte` pulls that
# packed value back out as a UInt8 ready for `write_reg`.
@inline as_byte(x) = reinterpret(UInt8, getfield(x, :fields))

"""
    pack_int16(low::UInt8, high::UInt8) -> Int16

Combine an `_L`/`_H` register pair into a signed 16-bit integer
(two's complement, as the sensor outputs).
"""
@inline function pack_int16(low::UInt8, high::UInt8)
    return reinterpret(Int16, (UInt16(high) << 8) | UInt16(low))
end

"""
    parse_pair(imu::ICM42688PC, lo_byte::UInt8, hi_byte::UInt8) -> Int16

Decode a 16-bit value taking the current CTRL1.BE setting into account.
When `little_endian == true`, bytes come out of a burst read as `[L, H]`;
when `false` they come out as `[H, L]`.
"""
@inline function parse_pair(imu::ICM42688PC, b0::UInt8, b1::UInt8)
    return imu.ctrl1.BE ? pack_int16(b0, b1) : pack_int16(b1, b0)
end

#=============================================================================
  High-level operations
=============================================================================#

"""
    check_who_am_i(imu::ICM42688PC) -> Bool

Return `true` if the chip identifies as a Tokmas ICM-42688-PC (`WHO_AM_I = 0x05`).
"""
check_who_am_i(imu::ICM42688PC) =
    read_reg(imu, WHO_AM_I_ADDR) == ICM42688PC_WHO_AM_I_VALUE

"""
    soft_reset!(imu::ICM42688PC)

Trigger a software reset (writes `0xB0` to register `0x60`). After this returns,
wait at least 15 ms before issuing further configuration (datasheet §11.4).
"""
function soft_reset!(imu::ICM42688PC)
    write_reg(imu, RESET_ADDR, RESET_MAGIC)
    sleep(0.02)
    write_reg(imu, CTRL1_ADDR, as_byte(imu.ctrl1))
    return nothing
end

function set_sensor_config!(imu::ICM42688PC, accel_en::Bool, gyro_en::Bool, 
    gyro_snooze::Bool, DRDY_disable::Bool, sync_sample::Bool)
    ctrl7 = CTRL7(accel_en, gyro_en, gyro_snooze, DRDY_disable, sync_sample)
    write_reg(imu, CTRL7_ADDR, as_byte(ctrl7))
    return nothing
end

"""
    set_accel_config!(imu, range::AccelRange, odr::AccelODR; self_test::Bool=false)

Program CTRL2 with the desired accel full-scale, ODR, and self-test enable.
Caches `range` on the device struct for later scaling.
"""
function set_accel_config!(imu::ICM42688PC, range::AccelRange, odr::AccelODR;
                           self_test::Bool=false)
    ctrl2 = CTRL2(UInt8(odr), UInt8(range), self_test ? 0x01 : 0x00)
    write_reg(imu, CTRL2_ADDR, as_byte(ctrl2))
    imu.accel_range = range
    return nothing
end

"""
    set_gyro_config!(imu, range::GyroRange, odr::GyroODR; self_test::Bool=false)

Program CTRL3 with the desired gyro full-scale, ODR, and self-test enable.
"""
function set_gyro_config!(imu::ICM42688PC, range::GyroRange, odr::GyroODR;
                          self_test::Bool=false)
    ctrl3 = CTRL3(UInt8(odr), UInt8(range), self_test ? 0x01 : 0x00)
    write_reg(imu, CTRL3_ADDR, as_byte(ctrl3))
    imu.gyro_range = range
    return nothing
end

function get_statusint(imu::ICM42688PC)
    return convert(STATUSINT, read_reg(imu, STATUSINT_ADDR))
end

function wait_until_avail(imu::ICM42688PC, dt::Float64, tries::Int; state::Bool=true)
    for i=1:tries
        if get_statusint(imu).Avail == state
            return true
        end
        sleep(dt)
    end
    return false
end

function wait_until_cmddone(imu::ICM42688PC, dt::Float64, tries::Int; state::Bool=true)
    for i=1:tries
        if get_statusint(imu).CmdDone == state
            return true
        end
        sleep(dt)
    end
    return false
end

struct SelfTestResult
    dVX::UInt16
    dVY::UInt16
    dVZ::UInt16
end

function accel_self_test(imu::ICM42688PC; range::AccelRange = ACCEL_FS_2G, odr::AccelODR = ACCEL_ODR_1000HZ)
    set_sensor_config!(imu, false, false, false, false, false)
    set_accel_config!(imu, range, odr; self_test=true)
    if !wait_until_avail(imu, 0.01, 50)
        return false
    end
    set_accel_config!(imu, range, odr; self_test=false)
    if !wait_until_avail(imu, 0.01, 50; state = false)
        return false
    end
    #= doesn't work... probably autoincrement doesn't work on these registers...
    read_regs!(imu, dVX_L, 6)
    result = SelfTestResult(
        unsigned(parse_pair(imu, imu.rx_buf[2], imu.rx_buf[3])),
        unsigned(parse_pair(imu, imu.rx_buf[4], imu.rx_buf[5])),
        unsigned(parse_pair(imu, imu.rx_buf[6], imu.rx_buf[7])))
    =#
    result = SelfTestResult(
        unsigned(parse_pair(imu, read_reg(imu, dVX_L), read_reg(imu, dVX_H))),
        unsigned(parse_pair(imu, read_reg(imu, dVY_L), read_reg(imu, dVY_H))),
        unsigned(parse_pair(imu, read_reg(imu, dVZ_L), read_reg(imu, dVZ_H))))
    return result.dVX > 400 && result.dVY > 400 && result.dVZ > 400
end

function gyro_self_test(imu::ICM42688PC; range::GyroRange = GYRO_FS_16DPS, odr::GyroODR = GYRO_ODR_7174_4HZ)
    set_sensor_config!(imu, false, false, false, false, false)
    set_gyro_config!(imu, range, odr; self_test=true)
    if !wait_until_avail(imu, 0.01, 50)
        return false
    end
    set_gyro_config!(imu, range, odr; self_test=false)
    if !wait_until_avail(imu, 0.01, 50; state = false)
        return false
    end
    result = SelfTestResult(
        unsigned(parse_pair(imu, read_reg(imu, dVX_L), read_reg(imu, dVX_H))),
        unsigned(parse_pair(imu, read_reg(imu, dVY_L), read_reg(imu, dVY_H))),
        unsigned(parse_pair(imu, read_reg(imu, dVZ_L), read_reg(imu, dVZ_H))))
    return result.dVX > 4800 && result.dVY > 4800 && result.dVZ > 4800
end

function execute_ctrl9_command(imu::ICM42688PC, cmd::Ctrl9Cmd; tries=50, dt=0.01)
    write_reg(imu, CTRL9_ADDR, UInt8(cmd))
    if !wait_until_cmddone(imu, dt, tries)
        return
    end
    write_reg(imu, CTRL9_ADDR, UInt8(CTRL_CMD_ACK))
    if !wait_until_cmddone(imu, 0.01, 50; state = false)
        return
    end
end

function calibration_on_demand(imu::ICM42688PC)
    set_sensor_config!(imu, false, false, false, false, false)
    execute_ctrl9_command(imu, CTRL_CMD_ON_DEMAND_CALIBRATION; dt = 0.1)
    result = convert(COD_STATUS, read_reg(imu, COD_STATUS_ADDR))
    read_regs!(imu, dVX_L, 6)
    return result, collect(imu.rx_buf[2:7])
end

function load_calibration(imu::ICM42688PC, calibration_data::Vector{UInt8})
    set_sensor_config!(imu, false, false, false, false, false)
    write_reg.(
        (imu,), 
        [CAL1_L_ADDR, CAL1_H_ADDR, CAL2_L_ADDR, CAL2_H_ADDR, CAL3_L_ADDR, CAL3_H_ADDR],
        calibration_data)
    execute_ctrl9_command(imu, CTRL_CMD_APPLY_GYRO_GAINS)
    return nothing
end

"""
    read_all_raw(imu::ICM42688PC) -> ICM42688PCData

Read all sensor data (temp, accel, gyro) as raw values.
Reads 14 consecutive bytes starting from TEMP_DATA1 (0x1D).
Layout: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes.
"""
function read_all_raw(imu::ICM42688PC)
    read_regs!(imu, TEMP_L, 14)
    buf = imu.rx_buf  # data at indices 2..15

    return ICM42688PCData(
        parse_pair(imu, buf[2],  buf[3]),   # TEMP
        parse_pair(imu, buf[4],  buf[5]),   # ACCEL_X
        parse_pair(imu, buf[6],  buf[7]),   # ACCEL_Y
        parse_pair(imu, buf[8],  buf[9]),   # ACCEL_Z
        parse_pair(imu, buf[10], buf[11]),  # GYRO_X
        parse_pair(imu, buf[12], buf[13]),  # GYRO_Y
        parse_pair(imu, buf[14], buf[15])   # GYRO_Z
    )
end

"""
    read_all(imu::ICM42688PC) -> NamedTuple

Read all sensor data with physical units.

Returns a named tuple with:
- accel_x, accel_y, accel_z: acceleration in g
- temp: temperature in °C
- gyro_x, gyro_y, gyro_z: angular velocity in °/s
"""
function read_all(imu::ICM42688PC)
    raw = read_all_raw(imu)
    accel_scale = ACCEL_SCALE[imu.accel_range]
    gyro_scale = GYRO_SCALE[imu.gyro_range]
    buf = imu.rx_buf  # data at indices 2..15

    return (
        accel_x = raw.accel_x / accel_scale,
        accel_y = raw.accel_y / accel_scale,
        accel_z = raw.accel_z / accel_scale,
        temp = imu.ctrl1.BE ? ((buf[3] * 256) + buf[2])/256 : ((buf[2] * 256) + buf[3])/256,
        gyro_x = raw.gyro_x / gyro_scale - imu.gyro_bias[1],
        gyro_y = raw.gyro_y / gyro_scale - imu.gyro_bias[2],
        gyro_z = raw.gyro_z / gyro_scale - imu.gyro_bias[3]
    )
end

#=============================================================================
  Cleanup
=============================================================================#

"""
    close!(imu::ICM42688PC)

Close the SPI connection.
"""
function close!(imu::ICM42688PC)
    close(imu.device)
end

end # module ICM42688PC
