using WiringPi

# MPU6000/MPU6050 Register Map
module MPU6000Registers
    # Self-test registers
    const SELF_TEST_X = 0x0D
    const SELF_TEST_Y = 0x0E
    const SELF_TEST_Z = 0x0F
    const SELF_TEST_A = 0x10

    # Configuration
    const SMPLRT_DIV = 0x19
    const CONFIG = 0x1A
    const GYRO_CONFIG = 0x1B
    const ACCEL_CONFIG = 0x1C

    # FIFO
    const FIFO_EN = 0x23

    # I2C Master
    const I2C_MST_CTRL = 0x24
    const I2C_SLV0_ADDR = 0x25
    const I2C_SLV0_REG = 0x26
    const I2C_SLV0_CTRL = 0x27

    # Interrupt
    const INT_PIN_CFG = 0x37
    const INT_ENABLE = 0x38
    const INT_STATUS = 0x3A

    # Accelerometer output (16-bit, high byte first)
    const ACCEL_XOUT_H = 0x3B
    const ACCEL_XOUT_L = 0x3C
    const ACCEL_YOUT_H = 0x3D
    const ACCEL_YOUT_L = 0x3E
    const ACCEL_ZOUT_H = 0x3F
    const ACCEL_ZOUT_L = 0x40

    # Temperature output (16-bit)
    const TEMP_OUT_H = 0x41
    const TEMP_OUT_L = 0x42

    # Gyroscope output (16-bit, high byte first)
    const GYRO_XOUT_H = 0x43
    const GYRO_XOUT_L = 0x44
    const GYRO_YOUT_H = 0x45
    const GYRO_YOUT_L = 0x46
    const GYRO_ZOUT_H = 0x47
    const GYRO_ZOUT_L = 0x48

    # Power management
    const PWR_MGMT_1 = 0x6B
    const PWR_MGMT_2 = 0x6C

    # FIFO count and data
    const FIFO_COUNTH = 0x72
    const FIFO_COUNTL = 0x73
    const FIFO_R_W = 0x74

    # Identity
    const WHO_AM_I = 0x75

    # Signal path reset
    const SIGNAL_PATH_RESET = 0x68
    const USER_CTRL = 0x6A
end

#=============================================================================
  MPU6000 Configuration Enums and Constants
=============================================================================#

# Gyro full-scale range options
@enum GyroRange::UInt8 begin
    GYRO_FS_250  = 0x00  # ±250 °/s
    GYRO_FS_500  = 0x08  # ±500 °/s
    GYRO_FS_1000 = 0x10  # ±1000 °/s
    GYRO_FS_2000 = 0x18  # ±2000 °/s
end

# Accelerometer full-scale range options
@enum AccelRange::UInt8 begin
    ACCEL_FS_2G  = 0x00  # ±2g
    ACCEL_FS_4G  = 0x08  # ±4g
    ACCEL_FS_8G  = 0x10  # ±8g
    ACCEL_FS_16G = 0x18  # ±16g
end

# Clock source options
@enum ClockSource::UInt8 begin
    CLOCK_INTERNAL    = 0x00  # Internal 8MHz oscillator
    CLOCK_PLL_XGYRO   = 0x01  # PLL with X axis gyroscope reference
    CLOCK_PLL_YGYRO   = 0x02  # PLL with Y axis gyroscope reference
    CLOCK_PLL_ZGYRO   = 0x03  # PLL with Z axis gyroscope reference
    CLOCK_PLL_EXT32K  = 0x04  # PLL with external 32.768kHz reference
    CLOCK_PLL_EXT19M  = 0x05  # PLL with external 19.2MHz reference
    CLOCK_STOP        = 0x07  # Stops clock, keeps timing generator in reset
end

# DLPF (Digital Low Pass Filter) bandwidth options
@enum DLPFBandwidth::UInt8 begin
    DLPF_BW_260 = 0x00  # Accel: 260Hz, Gyro: 256Hz
    DLPF_BW_184 = 0x01  # Accel: 184Hz, Gyro: 188Hz
    DLPF_BW_94  = 0x02  # Accel: 94Hz, Gyro: 98Hz
    DLPF_BW_44  = 0x03  # Accel: 44Hz, Gyro: 42Hz
    DLPF_BW_21  = 0x04  # Accel: 21Hz, Gyro: 20Hz
    DLPF_BW_10  = 0x05  # Accel: 10Hz, Gyro: 10Hz
    DLPF_BW_5   = 0x06  # Accel: 5Hz, Gyro: 5Hz
end

# Sensitivity scale factors
const GYRO_SCALE = Dict(
    GYRO_FS_250  => 131.0,   # LSB/(°/s)
    GYRO_FS_500  => 65.5,
    GYRO_FS_1000 => 32.8,
    GYRO_FS_2000 => 16.4
)

const ACCEL_SCALE = Dict(
    ACCEL_FS_2G  => 16384.0,  # LSB/g
    ACCEL_FS_4G  => 8192.0,
    ACCEL_FS_8G  => 4096.0,
    ACCEL_FS_16G => 2048.0
)

#=============================================================================
  Sensor Data Struct
=============================================================================#

"""
    MPU6000Data

Struct containing raw sensor readings from the MPU6000.
All values are raw 16-bit signed integers directly from the sensor registers.
"""
struct MPU6000Data
    accel_x::Int16
    accel_y::Int16
    accel_z::Int16
    temp::Int16
    gyro_x::Int16
    gyro_y::Int16
    gyro_z::Int16
end

#=============================================================================
  MPU6000 Device Struct
=============================================================================#

"""
    MPU6000

Wrapper struct for MPU6000/MPU6050 IMU sensor communication via I2C.

# Fields
- `handle::Int`: I2C file descriptor returned from wiringPiI2CSetup
- `gyro_range::GyroRange`: Current gyroscope full-scale range
- `accel_range::AccelRange`: Current accelerometer full-scale range
"""
mutable struct MPU6000
    handle::Int
    gyro_range::GyroRange
    accel_range::AccelRange
end

"""
    MPU6000(handle::Int)

Create an MPU6000 instance with default range settings (±250°/s gyro, ±2g accel).
"""
function MPU6000(handle::Int)
    return MPU6000(handle, GYRO_FS_250, ACCEL_FS_2G)
end

#=============================================================================
  Low-level I2C Operations
=============================================================================#

"""
    write_byte(mpu::MPU6000, reg::UInt8, value::UInt8)

Write a single byte to the specified register.
"""
function write_byte(mpu::MPU6000, reg::UInt8, value::UInt8)
    return wiringPiI2CWriteReg8(mpu.handle, Int(reg), Int(value))
end

"""
    read_byte(mpu::MPU6000, reg::UInt8) -> UInt8

Read a single byte from the specified register.
"""
function read_byte(mpu::MPU6000, reg::UInt8)
    return UInt8(wiringPiI2CReadReg8(mpu.handle, Int(reg)))
end

"""
    read_bytes(mpu::MPU6000, reg::UInt8, count::Integer) -> Vector{UInt8}

Read multiple bytes starting from the specified register.
Reads sequentially from consecutive registers.
"""
function read_bytes(mpu::MPU6000, reg::UInt8, count::Integer)
    data = Vector{UInt8}(undef, count)
    for i in 1:count
        data[i] = UInt8(wiringPiI2CReadReg8(mpu.handle, Int(reg) + i - 1))
    end
    return data
end

"""
    write_bytes(mpu::MPU6000, reg::UInt8, data::Vector{UInt8})

Write multiple bytes starting at the specified register.
"""
function write_bytes(mpu::MPU6000, reg::UInt8, data::Vector{UInt8})
    for (i, byte) in enumerate(data)
        wiringPiI2CWriteReg8(mpu.handle, Int(reg) + i - 1, Int(byte))
    end
    return 0
end

#=============================================================================
  Helper Functions for Parsing Sensor Data
=============================================================================#

"""
    parse_int16_be(data::Vector{UInt8}, offset::Integer) -> Int16

Parse a signed 16-bit big-endian value from data at the given byte offset (1-indexed).
"""
function parse_int16_be(data::Vector{UInt8}, offset::Integer)
    high = UInt16(data[offset])
    low = UInt16(data[offset + 1])
    raw = (high << 8) | low
    return reinterpret(Int16, raw)
end

#=============================================================================
  Initialization and Configuration
=============================================================================#

"""
    init!(mpu::MPU6000; clock_source::ClockSource=CLOCK_PLL_XGYRO)

Initialize the MPU6000 by waking it from sleep mode and setting the clock source.
Returns 0 on success.
"""
function init!(mpu::MPU6000; clock_source::ClockSource=CLOCK_PLL_XGYRO)
    # Wake up device (clear SLEEP bit) and set clock source
    return write_byte(mpu, MPU6000Registers.PWR_MGMT_1, UInt8(clock_source))
end

"""
    reset!(mpu::MPU6000)

Perform a full device reset. Wait at least 100ms after calling this.
"""
function reset!(mpu::MPU6000)
    return write_byte(mpu, MPU6000Registers.PWR_MGMT_1, 0x80)  # Set DEVICE_RESET bit
end

"""
    set_gyro_range!(mpu::MPU6000, range::GyroRange)

Set the gyroscope full-scale range.
"""
function set_gyro_range!(mpu::MPU6000, range::GyroRange)
    mpu.gyro_range = range
    return write_byte(mpu, MPU6000Registers.GYRO_CONFIG, UInt8(range))
end

"""
    set_accel_range!(mpu::MPU6000, range::AccelRange)

Set the accelerometer full-scale range.
"""
function set_accel_range!(mpu::MPU6000, range::AccelRange)
    mpu.accel_range = range
    return write_byte(mpu, MPU6000Registers.ACCEL_CONFIG, UInt8(range))
end

"""
    set_dlpf!(mpu::MPU6000, bandwidth::DLPFBandwidth)

Set the Digital Low Pass Filter bandwidth for both gyro and accelerometer.
"""
function set_dlpf!(mpu::MPU6000, bandwidth::DLPFBandwidth)
    return write_byte(mpu, MPU6000Registers.CONFIG, UInt8(bandwidth))
end

"""
    set_sample_rate!(mpu::MPU6000, rate_div::UInt8)

Set the sample rate divider. Sample Rate = Gyroscope Output Rate / (1 + rate_div)
where Gyroscope Output Rate = 8kHz when DLPF disabled, 1kHz when DLPF enabled.
"""
function set_sample_rate!(mpu::MPU6000, rate_div::UInt8)
    return write_byte(mpu, MPU6000Registers.SMPLRT_DIV, rate_div)
end

#=============================================================================
  Sensor Reading Functions
=============================================================================#

"""
    who_am_i(mpu::MPU6000) -> UInt8

Read the WHO_AM_I register. Should return 0x68 for MPU6000/MPU6050.
"""
function who_am_i(mpu::MPU6000)
    return read_byte(mpu, MPU6000Registers.WHO_AM_I)
end

"""
    read_accel_raw(mpu::MPU6000) -> Tuple{Int16, Int16, Int16}

Read raw accelerometer values (X, Y, Z) as signed 16-bit integers.
"""
function read_accel_raw(mpu::MPU6000)
    data = read_bytes(mpu, MPU6000Registers.ACCEL_XOUT_H, 6)
    ax = parse_int16_be(data, 1)
    ay = parse_int16_be(data, 3)
    az = parse_int16_be(data, 5)
    return (ax, ay, az)
end

"""
    read_accel(mpu::MPU6000) -> Tuple{Float64, Float64, Float64}

Read accelerometer values (X, Y, Z) in g units.
"""
function read_accel(mpu::MPU6000)
    (ax, ay, az) = read_accel_raw(mpu)
    scale = ACCEL_SCALE[mpu.accel_range]
    return (ax / scale, ay / scale, az / scale)
end

"""
    read_gyro_raw(mpu::MPU6000) -> Tuple{Int16, Int16, Int16}

Read raw gyroscope values (X, Y, Z) as signed 16-bit integers.
"""
function read_gyro_raw(mpu::MPU6000)
    data = read_bytes(mpu, MPU6000Registers.GYRO_XOUT_H, 6)
    gx = parse_int16_be(data, 1)
    gy = parse_int16_be(data, 3)
    gz = parse_int16_be(data, 5)
    return (gx, gy, gz)
end

"""
    read_gyro(mpu::MPU6000) -> Tuple{Float64, Float64, Float64}

Read gyroscope values (X, Y, Z) in degrees per second.
"""
function read_gyro(mpu::MPU6000)
    (gx, gy, gz) = read_gyro_raw(mpu)
    scale = GYRO_SCALE[mpu.gyro_range]
    return (gx / scale, gy / scale, gz / scale)
end

"""
    read_temp_raw(mpu::MPU6000) -> Int16

Read raw temperature value as a signed 16-bit integer.
"""
function read_temp_raw(mpu::MPU6000)
    data = read_bytes(mpu, MPU6000Registers.TEMP_OUT_H, 2)
    return parse_int16_be(data, 1)
end

"""
    read_temp(mpu::MPU6000) -> Float64

Read temperature in degrees Celsius.
Formula: Temperature = (TEMP_OUT / 340) + 36.53
"""
function read_temp(mpu::MPU6000)
    raw = read_temp_raw(mpu)
    return (raw / 340.0) + 36.53
end

"""
    read_all_raw(mpu::MPU6000) -> MPU6000Data

Read all sensor data (accel, temp, gyro) as raw values.
Reads 14 consecutive bytes starting from ACCEL_XOUT_H.
"""
function read_all_raw(mpu::MPU6000)
    # Read all 14 bytes: ACCEL (6) + TEMP (2) + GYRO (6)
    data = read_bytes(mpu, MPU6000Registers.ACCEL_XOUT_H, 14)

    return MPU6000Data(
        parse_int16_be(data, 1),   # ACCEL_XOUT
        parse_int16_be(data, 3),   # ACCEL_YOUT
        parse_int16_be(data, 5),   # ACCEL_ZOUT
        parse_int16_be(data, 7),   # TEMP_OUT
        parse_int16_be(data, 9),   # GYRO_XOUT
        parse_int16_be(data, 11),  # GYRO_YOUT
        parse_int16_be(data, 13)   # GYRO_ZOUT
    )
end

"""
    read_all(mpu::MPU6000) -> NamedTuple

Read all sensor data with physical units.

Returns a named tuple with:
- accel_x, accel_y, accel_z: acceleration in g
- temp: temperature in °C
- gyro_x, gyro_y, gyro_z: angular velocity in °/s
"""
function read_all(mpu::MPU6000)
    raw = read_all_raw(mpu)
    accel_scale = ACCEL_SCALE[mpu.accel_range]
    gyro_scale = GYRO_SCALE[mpu.gyro_range]

    return (
        accel_x = raw.accel_x / accel_scale,
        accel_y = raw.accel_y / accel_scale,
        accel_z = raw.accel_z / accel_scale,
        temp = (raw.temp / 340.0) + 36.53,
        gyro_x = raw.gyro_x / gyro_scale,
        gyro_y = raw.gyro_y / gyro_scale,
        gyro_z = raw.gyro_z / gyro_scale
    )
end

#=============================================================================
  Threaded MPU6000 Wrapper
=============================================================================#

"""
    ThreadedMPU6000

A thread-safe wrapper around MPU6000 that runs I2C operations in a dedicated thread.
Send `true` to the request channel to trigger a read, receive MPU6000Data on the response channel.

# Example
```julia
handle = wiringPiI2CSetup(0x68)
mpu = MPU6000(handle)
init!(mpu)

tmpu = ThreadedMPU6000(mpu)

# Trigger a read and get the result
put!(tmpu.request, true)
data = take!(tmpu.response)

stop!(tmpu)
```
"""
mutable struct ThreadedMPU6000
    request::Channel{Bool}
    response::Channel{MPU6000Data}
    worker::Task
    running::Threads.Atomic{Bool}
end

"""
    ThreadedMPU6000(mpu::MPU6000)

Create a ThreadedMPU6000 wrapper that spawns a worker thread for I2C reads.
The worker waits for a trigger on the request channel, performs a block read,
and sends the result on the response channel.
"""
function ThreadedMPU6000(mpu::MPU6000)
    request = Channel{Bool}(1)
    response = Channel{MPU6000Data}(1)
    running = Threads.Atomic{Bool}(true)

    worker = Threads.@spawn begin
        while running[]
            try
                trigger = take!(request)
                if !trigger
                    running[] = false
                    break
                end
                data = read_all_raw(mpu)
                put!(response, data)
            catch e
                isa(e, InvalidStateException) && !isopen(request) && break
                rethrow(e)
            end
        end
    end

    return ThreadedMPU6000(request, response, worker, running)
end

"""
    read!(tmpu::ThreadedMPU6000) -> MPU6000Data

Trigger a read and wait for the result.
"""
function read!(tmpu::ThreadedMPU6000)
    put!(tmpu.request, true)
    return take!(tmpu.response)
end

"""
    stop!(tmpu::ThreadedMPU6000)

Stop the worker thread.
"""
function stop!(tmpu::ThreadedMPU6000)
    if tmpu.running[]
        put!(tmpu.request, false)
        wait(tmpu.worker)
    end
    close(tmpu.request)
    close(tmpu.response)
    return nothing
end

#=============================================================================
  Interrupt Configuration
=============================================================================#

"""
    enable_data_ready_interrupt!(mpu::MPU6000)

Enable the data ready interrupt on the INT pin.
"""
function enable_data_ready_interrupt!(mpu::MPU6000)
    return write_byte(mpu, MPU6000Registers.INT_ENABLE, 0x01)
end

"""
    read_interrupt_status(mpu::MPU6000) -> UInt8

Read and clear the interrupt status register.
"""
function read_interrupt_status(mpu::MPU6000)
    return read_byte(mpu, MPU6000Registers.INT_STATUS)
end

"""
    configure_interrupt_pin!(mpu::MPU6000;
                             active_low::Bool=false,
                             open_drain::Bool=false,
                             latch::Bool=false,
                             clear_on_read::Bool=true)

Configure the INT pin behavior.
"""
function configure_interrupt_pin!(mpu::MPU6000;
                                   active_low::Bool=false,
                                   open_drain::Bool=false,
                                   latch::Bool=false,
                                   clear_on_read::Bool=true)
    config = UInt8(0)
    active_low && (config |= 0x80)
    open_drain && (config |= 0x40)
    latch && (config |= 0x20)
    clear_on_read && (config |= 0x10)
    return write_byte(mpu, MPU6000Registers.INT_PIN_CFG, config)
end

#=============================================================================
  Power Management
=============================================================================#

"""
    sleep!(mpu::MPU6000)

Put the MPU6000 into sleep mode to save power.
"""
function sleep!(mpu::MPU6000)
    current = read_byte(mpu, MPU6000Registers.PWR_MGMT_1)
    return write_byte(mpu, MPU6000Registers.PWR_MGMT_1, current | 0x40)
end

"""
    wake!(mpu::MPU6000)

Wake the MPU6000 from sleep mode.
"""
function wake!(mpu::MPU6000)
    current = read_byte(mpu, MPU6000Registers.PWR_MGMT_1)
    return write_byte(mpu, MPU6000Registers.PWR_MGMT_1, current & ~UInt8(0x40))
end
