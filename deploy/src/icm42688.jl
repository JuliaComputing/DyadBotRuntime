include(joinpath(@__DIR__, "spi.jl"))
using .SPI

#=============================================================================
  ICM-42688-P Register Map (User Bank 0)
=============================================================================#

module ICM42688Registers
    # Device configuration
    const DEVICE_CONFIG       = 0x11
    const DRIVE_CONFIG        = 0x13
    const INT_CONFIG          = 0x14
    const FIFO_CONFIG         = 0x16

    # Sensor data (all big-endian, high byte at lower address)
    const TEMP_DATA1          = 0x1D  # Temp [15:8]
    const TEMP_DATA0          = 0x1E  # Temp [7:0]
    const ACCEL_DATA_X1       = 0x1F  # Accel X [15:8]
    const ACCEL_DATA_X0       = 0x20  # Accel X [7:0]
    const ACCEL_DATA_Y1       = 0x21
    const ACCEL_DATA_Y0       = 0x22
    const ACCEL_DATA_Z1       = 0x23
    const ACCEL_DATA_Z0       = 0x24
    const GYRO_DATA_X1        = 0x25  # Gyro X [15:8]
    const GYRO_DATA_X0        = 0x26  # Gyro X [7:0]
    const GYRO_DATA_Y1        = 0x27
    const GYRO_DATA_Y0        = 0x28
    const GYRO_DATA_Z1        = 0x29
    const GYRO_DATA_Z0        = 0x2A
    const TMST_FSYNCH         = 0x2B
    const TMST_FSYNCL         = 0x2C

    # Status and FIFO
    const INT_STATUS          = 0x2D
    const FIFO_COUNTH         = 0x2E
    const FIFO_COUNTL         = 0x2F
    const FIFO_DATA           = 0x30

    # Configuration
    const SIGNAL_PATH_RESET   = 0x4B
    const INTF_CONFIG0        = 0x4C
    const INTF_CONFIG1        = 0x4D
    const PWR_MGMT0           = 0x4E
    const GYRO_CONFIG0        = 0x4F
    const ACCEL_CONFIG0       = 0x50
    const GYRO_CONFIG1        = 0x51
    const GYRO_ACCEL_CONFIG0  = 0x52
    const ACCEL_CONFIG1       = 0x53
    const TMST_CONFIG         = 0x54
    const APEX_CONFIG0        = 0x56
    const SMD_CONFIG          = 0x57
    const FIFO_CONFIG1        = 0x5F
    const FIFO_CONFIG2        = 0x60
    const FIFO_CONFIG3        = 0x61
    const FSYNC_CONFIG        = 0x62
    const INT_CONFIG0         = 0x63
    const INT_CONFIG1         = 0x64
    const INT_SOURCE0         = 0x65
    const INT_SOURCE1         = 0x66
    const INT_SOURCE3         = 0x68
    const INT_SOURCE4         = 0x69
    const SELF_TEST_CONFIG    = 0x70
    const WHO_AM_I            = 0x75
    const REG_BANK_SEL        = 0x76
end

#=============================================================================
  ICM-42688-P Configuration Enums and Constants
=============================================================================#

# Expected WHO_AM_I value
const ICM42688_WHO_AM_I_VALUE = 0x47

# SPI read flag: bit 7 = 1 for read
const SPI_READ_FLAG = UInt8(0x80)

# Gyro full-scale range (GYRO_FS_SEL in GYRO_CONFIG0 bits 7:5)
@enum GyroRange::UInt8 begin
    GYRO_FS_2000DPS  = 0x00  # ±2000 °/s
    GYRO_FS_1000DPS  = 0x01  # ±1000 °/s
    GYRO_FS_500DPS   = 0x02  # ±500 °/s
    GYRO_FS_250DPS   = 0x03  # ±250 °/s
    GYRO_FS_125DPS   = 0x04  # ±125 °/s
    GYRO_FS_62_5DPS  = 0x05  # ±62.5 °/s
    GYRO_FS_31_25DPS = 0x06  # ±31.25 °/s
    GYRO_FS_15_625DPS = 0x07 # ±15.625 °/s
end

# Accelerometer full-scale range (ACCEL_FS_SEL in ACCEL_CONFIG0 bits 7:5)
@enum AccelRange::UInt8 begin
    ACCEL_FS_16G = 0x00  # ±16g
    ACCEL_FS_8G  = 0x01  # ±8g
    ACCEL_FS_4G  = 0x02  # ±4g
    ACCEL_FS_2G  = 0x03  # ±2g
end

# Gyro output data rate (GYRO_ODR in GYRO_CONFIG0 bits 3:0)
@enum GyroODR::UInt8 begin
    GYRO_ODR_32KHZ  = 0x01
    GYRO_ODR_16KHZ  = 0x02
    GYRO_ODR_8KHZ   = 0x03
    GYRO_ODR_4KHZ   = 0x04
    GYRO_ODR_2KHZ   = 0x05
    GYRO_ODR_1KHZ   = 0x06  # default
    GYRO_ODR_200HZ  = 0x07
    GYRO_ODR_100HZ  = 0x08
    GYRO_ODR_50HZ   = 0x09
    GYRO_ODR_25HZ   = 0x0A
    GYRO_ODR_12_5HZ = 0x0B
    GYRO_ODR_500HZ  = 0x0F
end

# Accel output data rate (ACCEL_ODR in ACCEL_CONFIG0 bits 3:0)
@enum AccelODR::UInt8 begin
    ACCEL_ODR_32KHZ    = 0x01
    ACCEL_ODR_16KHZ    = 0x02
    ACCEL_ODR_8KHZ     = 0x03
    ACCEL_ODR_4KHZ     = 0x04
    ACCEL_ODR_2KHZ     = 0x05
    ACCEL_ODR_1KHZ     = 0x06  # default
    ACCEL_ODR_200HZ    = 0x07
    ACCEL_ODR_100HZ    = 0x08
    ACCEL_ODR_50HZ     = 0x09
    ACCEL_ODR_25HZ     = 0x0A
    ACCEL_ODR_12_5HZ   = 0x0B
    ACCEL_ODR_6_25HZ   = 0x0C
    ACCEL_ODR_3_125HZ  = 0x0D
    ACCEL_ODR_1_5625HZ = 0x0E
    ACCEL_ODR_500HZ    = 0x0F
end

# Gyro mode (GYRO_MODE in PWR_MGMT0 bits 3:2)
@enum GyroMode::UInt8 begin
    GYRO_MODE_OFF     = 0x00
    GYRO_MODE_STANDBY = 0x01
    GYRO_MODE_LN      = 0x03  # Low Noise
end

# Accel mode (ACCEL_MODE in PWR_MGMT0 bits 1:0)
@enum AccelMode::UInt8 begin
    ACCEL_MODE_OFF = 0x00
    ACCEL_MODE_LP  = 0x02  # Low Power
    ACCEL_MODE_LN  = 0x03  # Low Noise
end

# Sensitivity scale factors (LSB per unit)
const GYRO_SCALE = Dict(
    GYRO_FS_2000DPS   => 16.4,
    GYRO_FS_1000DPS   => 32.8,
    GYRO_FS_500DPS    => 65.5,
    GYRO_FS_250DPS    => 131.0,
    GYRO_FS_125DPS    => 262.0,
    GYRO_FS_62_5DPS   => 524.3,
    GYRO_FS_31_25DPS  => 1048.6,
    GYRO_FS_15_625DPS => 2097.2
)

const ACCEL_SCALE = Dict(
    ACCEL_FS_16G => 2048.0,
    ACCEL_FS_8G  => 4096.0,
    ACCEL_FS_4G  => 8192.0,
    ACCEL_FS_2G  => 16384.0
)

#=============================================================================
  Sensor Data Struct
=============================================================================#

"""
    ICM42688Data

Raw sensor readings from the ICM-42688-P.
All values are raw 16-bit signed integers directly from sensor registers.
"""
struct ICM42688Data
    accel_x::Int16
    accel_y::Int16
    accel_z::Int16
    temp::Int16
    gyro_x::Int16
    gyro_y::Int16
    gyro_z::Int16
end

#=============================================================================
  ICM-42688-P Device Struct
=============================================================================#

"""
    ICM42688

Wrapper struct for ICM-42688-P IMU sensor communication via SPI.

# Fields
- `device::SPI.SPIDevice`: SPI device handle
- `gyro_range::GyroRange`: Current gyroscope full-scale range
- `accel_range::AccelRange`: Current accelerometer full-scale range
- `tx_buf::Vector{UInt8}`: Pre-allocated TX buffer for SPI transfers
- `rx_buf::Vector{UInt8}`: Pre-allocated RX buffer for SPI transfers
"""
mutable struct ICM42688
    device::SPI.SPIDevice
    gyro_range::GyroRange
    accel_range::AccelRange
    tx_buf::Vector{UInt8}
    rx_buf::Vector{UInt8}
end

"""
    ICM42688(device::SPI.SPIDevice)

Create an ICM42688 instance with default range settings (±2000dps gyro, ±16g accel).
"""
function ICM42688(device::SPI.SPIDevice)
    # 15 bytes: 1 address + 14 data (temp + accel + gyro = 14 bytes for bulk read)
    return ICM42688(device, GYRO_FS_2000DPS, ACCEL_FS_16G, zeros(UInt8, 15), zeros(UInt8, 15))
end

"""
    ICM42688(bus::Int, cs::Int; speed_hz=1_000_000)

Create an ICM42688 instance by opening an SPI connection.
ICM-42688-P supports SPI Mode 0 or Mode 3, up to 24 MHz.
"""
function ICM42688(bus::Int, cs::Int; speed_hz::UInt32=UInt32(1_000_000))
    device = SPI.SPIDevice(bus, cs; mode=SPI.SPI_MODE_3, speed_hz=speed_hz)
    return ICM42688(device)
end

#=============================================================================
  Low-level SPI Register Operations
=============================================================================#

"""
    write_reg(imu::ICM42688, reg::UInt8, value::UInt8)

Write a single byte to the specified register.
SPI write: first byte = register address (bit 7 = 0), second byte = data.
"""
function write_reg(imu::ICM42688, reg::Integer, value::UInt8)
    imu.tx_buf[1] = UInt8(reg) & 0x7F  # bit 7 = 0 for write
    imu.tx_buf[2] = value
    return SPI.transfer!(imu.device, @view(imu.tx_buf[1:2]), @view(imu.rx_buf[1:2]))
end

"""
    read_reg(imu::ICM42688, reg::UInt8) -> UInt8

Read a single byte from the specified register.
SPI read: first byte = 0x80 | register address, second byte clocks out data.
"""
function read_reg(imu::ICM42688, reg::Integer)
    imu.tx_buf[1] = UInt8(reg) | SPI_READ_FLAG
    imu.tx_buf[2] = 0x00
    SPI.transfer!(imu.device, @view(imu.tx_buf[1:2]), @view(imu.rx_buf[1:2]))
    return imu.rx_buf[2]
end

"""
    read_regs!(imu::ICM42688, reg::Integer, count::Int) -> Nothing

Burst read `count` bytes starting from a register.
Data lands in `imu.rx_buf[2:count+1]` (index 1 is the discarded address echo).
"""
function read_regs!(imu::ICM42688, reg::Integer, count::Int)
    n = count + 1  # 1 address byte + N data bytes

    imu.tx_buf[1] = UInt8(reg) | SPI_READ_FLAG
    fill!(@view(imu.tx_buf[2:n]), 0x00)

    SPI.transfer!(imu.device, @view(imu.tx_buf[1:n]), @view(imu.rx_buf[1:n]))
    return nothing
end

#=============================================================================
  Helper Functions for Parsing Sensor Data
=============================================================================#

"""
    parse_int16_be(high::UInt8, low::UInt8) -> Int16

Parse a signed 16-bit big-endian value from two bytes.
"""
function parse_int16_be(high::UInt8, low::UInt8)
    raw = (UInt16(high) << 8) | UInt16(low)
    return reinterpret(Int16, raw)
end

#=============================================================================
  Bank Selection
=============================================================================#

"""
    set_bank!(imu::ICM42688, bank::Int)

Select a register bank (0-4). Most operations use bank 0.
"""
function set_bank!(imu::ICM42688, bank::Int)
    return write_reg(imu, ICM42688Registers.REG_BANK_SEL, UInt8(bank & 0x07))
end

#=============================================================================
  Initialization and Configuration
=============================================================================#

"""
    reset!(imu::ICM42688)

Perform a software reset. Wait at least 1ms after calling this.
"""
function reset!(imu::ICM42688)
    write_reg(imu, ICM42688Registers.DEVICE_CONFIG, 0x01)  # SOFT_RESET_CONFIG = 1
    sleep(0.001)  # 1ms minimum
    return nothing
end

"""
    init!(imu::ICM42688; gyro_mode=GYRO_MODE_LN, accel_mode=ACCEL_MODE_LN)

Initialize the ICM-42688-P: verify identity, configure for SPI, and enable sensors.
Returns 0 on success.
"""
function init!(imu::ICM42688;
               gyro_mode::GyroMode=GYRO_MODE_LN,
               accel_mode::AccelMode=ACCEL_MODE_LN)
    # Ensure we're on bank 0
    set_bank!(imu, 0)

    # Verify chip identity
    id = who_am_i(imu)
    if id != ICM42688_WHO_AM_I_VALUE
        error("ICM-42688-P WHO_AM_I mismatch: expected 0x$(string(ICM42688_WHO_AM_I_VALUE, base=16, pad=2)), got 0x$(string(id, base=16, pad=2))")
    end

    # Configure INT_CONFIG1: clear INT_ASYNC_RESET (bit 4) for proper INT pin operation
    val = read_reg(imu, ICM42688Registers.INT_CONFIG1)
    write_reg(imu, ICM42688Registers.INT_CONFIG1, val & ~UInt8(0x10))

    # Set SPI slew rate for best performance (DRIVE_CONFIG)
    write_reg(imu, ICM42688Registers.DRIVE_CONFIG, 0x05)

    # Enable gyro and accel in specified modes
    pwr = (UInt8(gyro_mode) << 2) | UInt8(accel_mode)
    write_reg(imu, ICM42688Registers.PWR_MGMT0, pwr)

    # Wait 200us after turning on sensors (per datasheet)
    sleep(0.001)

    return 0
end

"""
    who_am_i(imu::ICM42688) -> UInt8

Read the WHO_AM_I register. Should return 0x47 for ICM-42688-P.
"""
function who_am_i(imu::ICM42688)
    return read_reg(imu, ICM42688Registers.WHO_AM_I)
end

#=============================================================================
  Gyro Configuration
=============================================================================#

"""
    set_gyro_range!(imu::ICM42688, range::GyroRange)

Set the gyroscope full-scale range.
"""
function set_gyro_range!(imu::ICM42688, range::GyroRange)
    val = read_reg(imu, ICM42688Registers.GYRO_CONFIG0)
    val = (val & 0x1F) | (UInt8(range) << 5)
    write_reg(imu, ICM42688Registers.GYRO_CONFIG0, val)
    imu.gyro_range = range
    return nothing
end

"""
    set_gyro_odr!(imu::ICM42688, odr::GyroODR)

Set the gyroscope output data rate.
"""
function set_gyro_odr!(imu::ICM42688, odr::GyroODR)
    val = read_reg(imu, ICM42688Registers.GYRO_CONFIG0)
    val = (val & 0xF0) | UInt8(odr)
    write_reg(imu, ICM42688Registers.GYRO_CONFIG0, val)
    return nothing
end

#=============================================================================
  Accelerometer Configuration
=============================================================================#

"""
    set_accel_range!(imu::ICM42688, range::AccelRange)

Set the accelerometer full-scale range.
"""
function set_accel_range!(imu::ICM42688, range::AccelRange)
    val = read_reg(imu, ICM42688Registers.ACCEL_CONFIG0)
    val = (val & 0x1F) | (UInt8(range) << 5)
    write_reg(imu, ICM42688Registers.ACCEL_CONFIG0, val)
    imu.accel_range = range
    return nothing
end

"""
    set_accel_odr!(imu::ICM42688, odr::AccelODR)

Set the accelerometer output data rate.
"""
function set_accel_odr!(imu::ICM42688, odr::AccelODR)
    val = read_reg(imu, ICM42688Registers.ACCEL_CONFIG0)
    val = (val & 0xF0) | UInt8(odr)
    write_reg(imu, ICM42688Registers.ACCEL_CONFIG0, val)
    return nothing
end

#=============================================================================
  Sensor Reading Functions
=============================================================================#

"""
    read_accel_raw(imu::ICM42688) -> Tuple{Int16, Int16, Int16}

Read raw accelerometer values (X, Y, Z) as signed 16-bit integers.
"""
function read_accel_raw(imu::ICM42688)
    read_regs!(imu, ICM42688Registers.ACCEL_DATA_X1, 6)
    buf = imu.rx_buf
    ax = parse_int16_be(buf[2], buf[3])
    ay = parse_int16_be(buf[4], buf[5])
    az = parse_int16_be(buf[6], buf[7])
    return (ax, ay, az)
end

"""
    read_accel(imu::ICM42688) -> Tuple{Float64, Float64, Float64}

Read accelerometer values (X, Y, Z) in g units.
"""
function read_accel(imu::ICM42688)
    (ax, ay, az) = read_accel_raw(imu)
    scale = ACCEL_SCALE[imu.accel_range]
    return (ax / scale, ay / scale, az / scale)
end

"""
    read_gyro_raw(imu::ICM42688) -> Tuple{Int16, Int16, Int16}

Read raw gyroscope values (X, Y, Z) as signed 16-bit integers.
"""
function read_gyro_raw(imu::ICM42688)
    buf = @view imu.rx_buf[1:6]
    read_regs!(imu, ICM42688Registers.GYRO_DATA_X1, buf)
    gx = parse_int16_be(buf[1], buf[2])
    gy = parse_int16_be(buf[3], buf[4])
    gz = parse_int16_be(buf[5], buf[6])
    return (gx, gy, gz)
end

"""
    read_gyro(imu::ICM42688) -> Tuple{Float64, Float64, Float64}

Read gyroscope values (X, Y, Z) in degrees per second.
"""
function read_gyro(imu::ICM42688)
    (gx, gy, gz) = read_gyro_raw(imu)
    scale = GYRO_SCALE[imu.gyro_range]
    return (gx / scale, gy / scale, gz / scale)
end

"""
    read_temp_raw(imu::ICM42688) -> Int16

Read raw temperature value as a signed 16-bit integer.
"""
function read_temp_raw(imu::ICM42688)
    buf = @view imu.rx_buf[1:2]
    read_regs!(imu, ICM42688Registers.TEMP_DATA1, buf)
    return parse_int16_be(buf[1], buf[2])
end

"""
    read_temp(imu::ICM42688) -> Float64

Read temperature in degrees Celsius.
Formula: Temperature = (TEMP_DATA / 132.48) + 25
"""
function read_temp(imu::ICM42688)
    raw = read_temp_raw(imu)
    return (raw / 132.48) + 25.0
end

"""
    read_all_raw(imu::ICM42688) -> ICM42688Data

Read all sensor data (temp, accel, gyro) as raw values.
Reads 14 consecutive bytes starting from TEMP_DATA1 (0x1D).
Layout: TEMP(2) + ACCEL(6) + GYRO(6) = 14 bytes.
"""
function read_all_raw(imu::ICM42688)
    buf = @view imu.rx_buf[1:14]
    read_regs!(imu, ICM42688Registers.TEMP_DATA1, buf)

    return ICM42688Data(
        parse_int16_be(buf[3],  buf[4]),   # ACCEL_X
        parse_int16_be(buf[5],  buf[6]),   # ACCEL_Y
        parse_int16_be(buf[7],  buf[8]),   # ACCEL_Z
        parse_int16_be(buf[1],  buf[2]),   # TEMP
        parse_int16_be(buf[9],  buf[10]),  # GYRO_X
        parse_int16_be(buf[11], buf[12]),  # GYRO_Y
        parse_int16_be(buf[13], buf[14])   # GYRO_Z
    )
end

"""
    read_all(imu::ICM42688) -> NamedTuple

Read all sensor data with physical units.

Returns a named tuple with:
- accel_x, accel_y, accel_z: acceleration in g
- temp: temperature in °C
- gyro_x, gyro_y, gyro_z: angular velocity in °/s
"""
function read_all(imu::ICM42688)
    raw = read_all_raw(imu)
    accel_scale = ACCEL_SCALE[imu.accel_range]
    gyro_scale = GYRO_SCALE[imu.gyro_range]

    return (
        accel_x = raw.accel_x / accel_scale,
        accel_y = raw.accel_y / accel_scale,
        accel_z = raw.accel_z / accel_scale,
        temp = (raw.temp / 132.48) + 25.0,
        gyro_x = raw.gyro_x / gyro_scale,
        gyro_y = raw.gyro_y / gyro_scale,
        gyro_z = raw.gyro_z / gyro_scale
    )
end

#=============================================================================
  Interrupt Status
=============================================================================#

"""
    read_int_status(imu::ICM42688) -> UInt8

Read and clear the INT_STATUS register.
Bit 3 = DATA_RDY_INT, Bit 2 = FIFO_THS_INT, Bit 1 = FIFO_FULL_INT
"""
function read_int_status(imu::ICM42688)
    return read_reg(imu, ICM42688Registers.INT_STATUS)
end

"""
    enable_data_ready_int!(imu::ICM42688; pin::Int=1)

Route the data ready interrupt to INT1 (pin=1) or INT2 (pin=2).
"""
function enable_data_ready_int!(imu::ICM42688; pin::Int=1)
    if pin == 1
        val = read_reg(imu, ICM42688Registers.INT_SOURCE0)
        write_reg(imu, ICM42688Registers.INT_SOURCE0, val | 0x08)  # UI_DRDY_INT1_EN
    else
        val = read_reg(imu, ICM42688Registers.INT_SOURCE3)
        write_reg(imu, ICM42688Registers.INT_SOURCE3, val | 0x08)  # UI_DRDY_INT2_EN
    end
    return nothing
end

"""
    configure_int_pin!(imu::ICM42688; pin::Int=1, active_high::Bool=true,
                       push_pull::Bool=true, latched::Bool=false)

Configure the INT pin behavior.
"""
function configure_int_pin!(imu::ICM42688; pin::Int=1, active_high::Bool=true,
                            push_pull::Bool=true, latched::Bool=false)
    val = read_reg(imu, ICM42688Registers.INT_CONFIG)
    if pin == 1
        val &= 0xF8  # clear bits 2:0
        active_high && (val |= 0x01)  # INT1_POLARITY
        push_pull   && (val |= 0x02)  # INT1_DRIVE_CIRCUIT
        latched     && (val |= 0x04)  # INT1_MODE
    else
        val &= 0xC7  # clear bits 5:3
        active_high && (val |= 0x08)  # INT2_POLARITY
        push_pull   && (val |= 0x10)  # INT2_DRIVE_CIRCUIT
        latched     && (val |= 0x20)  # INT2_MODE
    end
    write_reg(imu, ICM42688Registers.INT_CONFIG, val)
    return nothing
end

#=============================================================================
  FIFO Operations
=============================================================================#

"""
    flush_fifo!(imu::ICM42688)

Flush the FIFO buffer.
"""
function flush_fifo!(imu::ICM42688)
    write_reg(imu, ICM42688Registers.SIGNAL_PATH_RESET, 0x02)  # FIFO_FLUSH
    return nothing
end

"""
    read_fifo_count(imu::ICM42688) -> UInt16

Read the number of bytes currently in the FIFO.
"""
function read_fifo_count(imu::ICM42688)
    buf = @view imu.rx_buf[1:2]
    read_regs!(imu, ICM42688Registers.FIFO_COUNTH, buf)
    return (UInt16(buf[1]) << 8) | UInt16(buf[2])
end

"""
    configure_fifo!(imu::ICM42688; stream::Bool=true, accel::Bool=true,
                    gyro::Bool=true, temp::Bool=false, hires::Bool=false)

Configure FIFO mode and what data is placed into the FIFO.
"""
function configure_fifo!(imu::ICM42688; stream::Bool=true, accel::Bool=true,
                         gyro::Bool=true, temp::Bool=false, hires::Bool=false)
    # FIFO_CONFIG: set mode
    mode = stream ? 0x40 : 0x00  # 01 = Stream-to-FIFO, 00 = Bypass
    write_reg(imu, ICM42688Registers.FIFO_CONFIG, mode)

    # FIFO_CONFIG1: select data sources
    config1 = UInt8(0)
    accel && (config1 |= 0x01)  # FIFO_ACCEL_EN
    gyro  && (config1 |= 0x02)  # FIFO_GYRO_EN
    temp  && (config1 |= 0x04)  # FIFO_TEMP_EN
    hires && (config1 |= 0x10)  # FIFO_HIRES_EN
    write_reg(imu, ICM42688Registers.FIFO_CONFIG1, config1)

    return nothing
end

#=============================================================================
  Power Management
=============================================================================#

"""
    set_gyro_mode!(imu::ICM42688, mode::GyroMode)

Set the gyroscope operating mode.
"""
function set_gyro_mode!(imu::ICM42688, mode::GyroMode)
    val = read_reg(imu, ICM42688Registers.PWR_MGMT0)
    val = (val & 0xF3) | (UInt8(mode) << 2)
    write_reg(imu, ICM42688Registers.PWR_MGMT0, val)
    sleep(0.001)  # wait after mode change
    return nothing
end

"""
    set_accel_mode!(imu::ICM42688, mode::AccelMode)

Set the accelerometer operating mode.
"""
function set_accel_mode!(imu::ICM42688, mode::AccelMode)
    val = read_reg(imu, ICM42688Registers.PWR_MGMT0)
    val = (val & 0xFC) | UInt8(mode)
    write_reg(imu, ICM42688Registers.PWR_MGMT0, val)
    sleep(0.001)  # wait after mode change
    return nothing
end

"""
    set_temp_enabled!(imu::ICM42688, enabled::Bool)

Enable or disable the temperature sensor.
"""
function set_temp_enabled!(imu::ICM42688, enabled::Bool)
    val = read_reg(imu, ICM42688Registers.PWR_MGMT0)
    if enabled
        val &= ~UInt8(0x20)  # clear TEMP_DIS
    else
        val |= 0x20           # set TEMP_DIS
    end
    write_reg(imu, ICM42688Registers.PWR_MGMT0, val)
    return nothing
end

#=============================================================================
  Threaded ICM-42688-P Wrapper
=============================================================================#

"""
    ThreadedICM42688

A thread-safe wrapper around ICM42688 that runs SPI reads in a dedicated thread.
Send `true` to the request channel to trigger a read, receive ICM42688Data on the response channel.
"""
mutable struct ThreadedICM42688
    request::Channel{Bool}
    response::Channel{ICM42688Data}
    worker::Task
    running::Threads.Atomic{Bool}
end

"""
    ThreadedICM42688(imu::ICM42688)

Create a ThreadedICM42688 wrapper that spawns a worker thread for SPI reads.
"""
function ThreadedICM42688(imu::ICM42688)
    request = Channel{Bool}(1)
    response = Channel{ICM42688Data}(1)
    running = Threads.Atomic{Bool}(true)

    worker = Threads.@spawn begin
        while running[]
            try
                trigger = take!(request)
                if !trigger
                    running[] = false
                    break
                end
                data = read_all_raw(imu)
                put!(response, data)
            catch e
                isa(e, InvalidStateException) && !isopen(request) && break
                rethrow(e)
            end
        end
    end

    return ThreadedICM42688(request, response, worker, running)
end

"""
    read!(timu::ThreadedICM42688) -> ICM42688Data

Trigger a read and wait for the result.
"""
function read!(timu::ThreadedICM42688)
    put!(timu.request, true)
    return take!(timu.response)
end

"""
    stop!(timu::ThreadedICM42688)

Stop the worker thread.
"""
function stop!(timu::ThreadedICM42688)
    if timu.running[]
        put!(timu.request, false)
        wait(timu.worker)
    end
    close(timu.request)
    close(timu.response)
    return nothing
end

#=============================================================================
  Cleanup
=============================================================================#

"""
    close!(imu::ICM42688)

Close the SPI connection.
"""
function close!(imu::ICM42688)
    close(imu.device)
end
