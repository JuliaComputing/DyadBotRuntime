#=============================================================================
  I2C Wrapper using ioctl

  Provides Julia interface for I2C communication via /dev/i2c-*
  Uses ioctl for low-level control and SMBus-style operations.
=============================================================================#

module I2C

export I2CDevice
export open_device, close_device
export read_byte, write_byte, read_byte_data, write_byte_data
export read_block_data, write_block_data
export read_bytes, read_bytes!, write_bytes

#=============================================================================
  ioctl Constants
=============================================================================#

# I2C ioctl commands
const I2C_SLAVE       = 0x0703  # Set slave address
const I2C_SLAVE_FORCE = 0x0706  # Set slave address (even if already in use)
const I2C_TENBIT      = 0x0704  # 10-bit addressing
const I2C_FUNCS       = 0x0705  # Get adapter functionality
const I2C_RDWR        = 0x0707  # Combined read/write transfer
const I2C_SMBUS       = 0x0720  # SMBus transfer

# SMBus transaction types
const I2C_SMBUS_READ  = UInt8(1)
const I2C_SMBUS_WRITE = UInt8(0)

# SMBus transaction sizes
const I2C_SMBUS_BYTE        = UInt32(1)
const I2C_SMBUS_BYTE_DATA   = UInt32(2)
const I2C_SMBUS_WORD_DATA   = UInt32(3)
const I2C_SMBUS_BLOCK_DATA  = UInt32(5)
const I2C_SMBUS_I2C_BLOCK_DATA = UInt32(8)

# I2C message flags
const I2C_M_RD           = UInt16(0x0001)  # Read data
const I2C_M_TEN          = UInt16(0x0010)  # 10-bit address
const I2C_M_NOSTART      = UInt16(0x4000)  # No START condition
const I2C_M_REV_DIR_ADDR = UInt16(0x2000)  # Reverse direction
const I2C_M_IGNORE_NAK   = UInt16(0x1000)  # Ignore NAK
const I2C_M_NO_RD_ACK    = UInt16(0x0800)  # No ACK on read

const I2C_SMBUS_BLOCK_MAX = 32

#=============================================================================
  Data Structures for ioctl
=============================================================================#

# SMBus ioctl arguments (matches C struct layout with padding)
struct SMBusIoctlData
    read_write::UInt8
    command::UInt8
    _pad::UInt16
    size::UInt32
    data::Ptr{UInt8}
end

# I2C message structure for I2C_RDWR
struct I2CMsg
    addr::UInt16
    flags::UInt16
    len::UInt16
    _pad::UInt16
    buf::Ptr{UInt8}
end

# I2C RDWR ioctl data
struct I2CRdwrIoctlData
    msgs::Ptr{I2CMsg}
    nmsgs::UInt32
    _pad::UInt32
end

#=============================================================================
  I2C Device Handle
=============================================================================#

const I2C_BUFFER_SIZE = 64  # Max buffer for read_bytes/write_bytes

mutable struct I2CDevice
    fd::RawFD
    bus::Int
    address::UInt8
    path::String
    # Pre-allocated buffers (shared between SMBus and RDWR operations)
    ioctl_buf::Vector{UInt8}   # For SMBusIoctlData or I2CRdwrIoctlData
    msgs_buf::Vector{UInt8}    # For I2CMsg array (2 msgs max)
    data_buf::Vector{UInt8}    # SMBus data (34 bytes) or RDWR reg+data
end

#=============================================================================
  ioctl Wrapper
=============================================================================#

ioctl(fd::RawFD, request, arg) = ccall(:ioctl, Cint, (Cint, Culong, Clong), fd, request, arg)

#=============================================================================
  Device Operations
=============================================================================#

"""
    open_device(bus::Int, address::UInt8) -> I2CDevice

Open an I2C device at the specified bus and address.
"""
function open_device(bus::Int, address::Integer)
    path = "/dev/i2c-$bus"
    fd = ccall(:open, Cint, (Cstring, Cint), path, 0x0002)  # O_RDWR = 0x0002
    if fd < 0
        error("Failed to open I2C device: $path")
    end

    raw_fd = RawFD(fd)
    addr = UInt8(address)

    # Set slave address
    ret = ioctl(raw_fd, UInt(I2C_SLAVE), Clong(addr))
    if ret < 0
        ccall(:close, Cint, (Cint,), fd)
        error("Failed to set I2C slave address: 0x$(string(addr, base=16))")
    end

    # Pre-allocate shared buffers
    ioctl_buf = zeros(UInt8, sizeof(SMBusIoctlData))  # same size as I2CRdwrIoctlData
    msgs_buf = zeros(UInt8, 2 * sizeof(I2CMsg))
    data_buf = zeros(UInt8, 1 + I2C_BUFFER_SIZE)  # reg byte + data

    return I2CDevice(raw_fd, bus, addr, path, ioctl_buf, msgs_buf, data_buf)
end

"""
    close_device(dev::I2CDevice)

Close an I2C device.
"""
function close_device(dev::I2CDevice)
    ccall(:close, Cint, (Cint,), Base.cconvert(Cint, dev.fd))
end

#=============================================================================
  Low-level Read/Write
=============================================================================#

"""
    i2c_read(dev::I2CDevice, buf::Vector{UInt8}) -> Int

Read bytes from I2C device into buffer. Returns number of bytes read.
"""
function i2c_read(dev::I2CDevice, buf::Vector{UInt8})
    n = ccall(:read, Cssize_t, (Cint, Ptr{UInt8}, Csize_t),
              Base.cconvert(Cint, dev.fd), buf, length(buf))
    return Int(n)
end

"""
    i2c_write(dev::I2CDevice, buf::Vector{UInt8}) -> Int

Write bytes to I2C device. Returns number of bytes written.
"""
function i2c_write(dev::I2CDevice, buf::Vector{UInt8})
    n = ccall(:write, Cssize_t, (Cint, Ptr{UInt8}, Csize_t),
              Base.cconvert(Cint, dev.fd), buf, length(buf))
    return Int(n)
end

#=============================================================================
  SMBus-style Operations
=============================================================================#

"""
    smbus_access(dev::I2CDevice, read_write::UInt8, command::UInt8, size::UInt32) -> Int

Perform an SMBus transaction using pre-allocated buffers.
Data is read from / written to dev.data_buf.
"""
function smbus_access(dev::I2CDevice, read_write::UInt8, command::UInt8, size::UInt32)
    args = SMBusIoctlData(read_write, command, UInt16(0), size, pointer(dev.data_buf))
    unsafe_store!(Ptr{SMBusIoctlData}(pointer(dev.ioctl_buf)), args)
    return ioctl(dev.fd, UInt(I2C_SMBUS), pointer(dev.ioctl_buf))
end

"""
    read_byte(dev::I2CDevice) -> UInt8

Read a single byte from the device (no register address).
"""
function read_byte(dev::I2CDevice)
    ret = smbus_access(dev, I2C_SMBUS_READ, UInt8(0), I2C_SMBUS_BYTE)
    ret < 0 && error("SMBus read_byte failed")
    return dev.data_buf[1]
end

"""
    write_byte(dev::I2CDevice, value::UInt8) -> Int

Write a single byte to the device (no register address).
"""
function write_byte(dev::I2CDevice, value::UInt8)
    return smbus_access(dev, I2C_SMBUS_WRITE, value, I2C_SMBUS_BYTE)
end

"""
    read_byte_data(dev::I2CDevice, reg::UInt8) -> UInt8

Read a byte from a specific register.
"""
function read_byte_data(dev::I2CDevice, reg::UInt8)
    ret = smbus_access(dev, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA)
    ret < 0 && error("SMBus read_byte_data failed for register 0x$(string(reg, base=16))")
    return dev.data_buf[1]
end

"""
    write_byte_data(dev::I2CDevice, reg::UInt8, value::UInt8) -> Int

Write a byte to a specific register.
"""
function write_byte_data(dev::I2CDevice, reg::UInt8, value::UInt8)
    dev.data_buf[1] = value
    return smbus_access(dev, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA)
end

"""
    read_word_data(dev::I2CDevice, reg::UInt8) -> UInt16

Read a 16-bit word from a specific register.
"""
function read_word_data(dev::I2CDevice, reg::UInt8)
    ret = smbus_access(dev, I2C_SMBUS_READ, reg, I2C_SMBUS_WORD_DATA)
    ret < 0 && error("SMBus read_word_data failed for register 0x$(string(reg, base=16))")
    return UInt16(dev.data_buf[1]) | (UInt16(dev.data_buf[2]) << 8)
end

"""
    write_word_data(dev::I2CDevice, reg::UInt8, value::UInt16) -> Int

Write a 16-bit word to a specific register.
"""
function write_word_data(dev::I2CDevice, reg::UInt8, value::UInt16)
    dev.data_buf[1] = UInt8(value & 0xFF)
    dev.data_buf[2] = UInt8((value >> 8) & 0xFF)
    return smbus_access(dev, I2C_SMBUS_WRITE, reg, I2C_SMBUS_WORD_DATA)
end

#=============================================================================
  Block Read/Write using I2C_RDWR
=============================================================================#

"""
    read_bytes!(dev::I2CDevice, reg::UInt8, dest::AbstractVector{UInt8}) -> Int

Read bytes into dest buffer starting from a register. Returns number of bytes read.
"""
function read_bytes!(dev::I2CDevice, reg::UInt8, dest::AbstractVector{UInt8})
    count = length(dest)
    count > I2C_BUFFER_SIZE && error("read_bytes!: count $count exceeds buffer size $I2C_BUFFER_SIZE")

    # Store register byte
    dev.data_buf[1] = reg

    # Pack two I2CMsg structs
    msgs_ptr = Ptr{I2CMsg}(pointer(dev.msgs_buf))
    unsafe_store!(msgs_ptr, I2CMsg(dev.address, UInt16(0), UInt16(1), UInt16(0), pointer(dev.data_buf)), 1)
    unsafe_store!(msgs_ptr, I2CMsg(dev.address, I2C_M_RD, UInt16(count), UInt16(0), pointer(dest)), 2)

    # Pack rdwr ioctl data
    rdwr = I2CRdwrIoctlData(Ptr{I2CMsg}(pointer(dev.msgs_buf)), UInt32(2), UInt32(0))
    unsafe_store!(Ptr{I2CRdwrIoctlData}(pointer(dev.ioctl_buf)), rdwr)

    ret = ioctl(dev.fd, UInt(I2C_RDWR), pointer(dev.ioctl_buf))
    ret < 0 && error("I2C_RDWR read failed for register 0x$(string(reg, base=16))")

    return count
end

"""
    read_bytes(dev::I2CDevice, reg::UInt8, count::Int) -> Vector{UInt8}

Read multiple bytes starting from a register. Allocates a new vector.
For zero-allocation, use read_bytes! with a pre-allocated buffer.
"""
function read_bytes(dev::I2CDevice, reg::UInt8, count::Int)
    dest = Vector{UInt8}(undef, count)
    read_bytes!(dev, reg, dest)
    return dest
end

"""
    write_bytes(dev::I2CDevice, reg::UInt8, data::AbstractVector{UInt8}) -> Int

Write multiple bytes starting at a register.
"""
function write_bytes(dev::I2CDevice, reg::UInt8, data::AbstractVector{UInt8})
    count = length(data)
    count > I2C_BUFFER_SIZE && error("write_bytes: count $count exceeds buffer size $I2C_BUFFER_SIZE")

    # Pack register + data into data_buf
    dev.data_buf[1] = reg
    copyto!(dev.data_buf, 2, data, 1, count)

    # Pack single I2CMsg
    msgs_ptr = Ptr{I2CMsg}(pointer(dev.msgs_buf))
    unsafe_store!(msgs_ptr, I2CMsg(dev.address, UInt16(0), UInt16(count + 1), UInt16(0), pointer(dev.data_buf)), 1)

    # Pack rdwr ioctl data
    rdwr = I2CRdwrIoctlData(Ptr{I2CMsg}(pointer(dev.msgs_buf)), UInt32(1), UInt32(0))
    unsafe_store!(Ptr{I2CRdwrIoctlData}(pointer(dev.ioctl_buf)), rdwr)

    ret = ioctl(dev.fd, UInt(I2C_RDWR), pointer(dev.ioctl_buf))
    ret < 0 && error("I2C_RDWR write failed for register 0x$(string(reg, base=16))")

    return ret
end

#=============================================================================
  Convenience Aliases
=============================================================================#

const wiringPiI2CReadReg8 = read_byte_data
const wiringPiI2CWriteReg8 = write_byte_data
const wiringPiI2CReadReg16 = read_word_data
const wiringPiI2CWriteReg16 = write_word_data

end # module I2C
