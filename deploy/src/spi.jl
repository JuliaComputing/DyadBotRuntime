#=============================================================================
  SPI Wrapper using Linux chardev API

  Provides Julia interface for SPI communication via /dev/spidevB.C
  Uses ioctl for configuration and SPI_IOC_MESSAGE for full-duplex transfers.

  Follows Julia IO conventions:
    dev = SPIDevice(0, 0; mode=SPI_MODE_3, speed_hz=10_000_000)
    write(dev, UInt8[0x01, 0x02])
    data = read(dev, 4)
    close(dev)

  Or with do-block:
    SPIDevice(0, 0) do dev
        transfer!(dev, tx, rx)
    end
=============================================================================#

module SPI

export SPIDevice, transfer!, write_then_read!
export set_mode, get_mode, set_speed, get_speed, set_bits_per_word, get_bits_per_word
export SPI_CPHA, SPI_CPOL, SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3
export SPI_CS_HIGH, SPI_LSB_FIRST, SPI_NO_CS

#=============================================================================
  SPI Mode Flags
=============================================================================#

const SPI_CPHA      = UInt8(0x01)
const SPI_CPOL      = UInt8(0x02)
const SPI_MODE_0    = UInt8(0x00)                # CPOL=0, CPHA=0
const SPI_MODE_1    = SPI_CPHA                    # CPOL=0, CPHA=1
const SPI_MODE_2    = SPI_CPOL                    # CPOL=1, CPHA=0
const SPI_MODE_3    = SPI_CPHA | SPI_CPOL         # CPOL=1, CPHA=1
const SPI_CS_HIGH   = UInt8(0x04)
const SPI_LSB_FIRST = UInt8(0x08)
const SPI_NO_CS     = UInt8(0x40)

#=============================================================================
  SPI Device Handle
=============================================================================#

const SPI_BUFFER_SIZE = 256

mutable struct SPIDevice
    io::IOStream
    bus::Int
    cs::Int
    mode::UInt8
    speed_hz::UInt32
    bits_per_word::UInt8
    scratch::Vector{UInt8}   # scratch buffer for TX zeros and ioctl config
    xfer_buf::Vector{UInt8}  # for SPIIocTransfer struct(s)

    function SPIDevice(io::IOStream, bus::Int, cs::Int, mode::UInt8,
                       speed_hz::UInt32, bits_per_word::UInt8,
                       scratch::Vector{UInt8}, xfer_buf::Vector{UInt8})
        obj = new(io, bus, cs, mode, speed_hz, bits_per_word, scratch, xfer_buf)
        finalizer(close, obj)
        return obj
    end
end

Base.close(dev::SPIDevice) = isopen(dev.io) && close(dev.io)
Base.isopen(dev::SPIDevice) = isopen(dev.io)

#=============================================================================
  Error Code Handling
=============================================================================#

get_errno() = ccall(:__errno_location, Ptr{Cint}, ())[]

const ERRNO_NAMES = Dict{Cint, Tuple{String, String}}(
    1   => ("EPERM",      "Operation not permitted - check process privileges"),
    2   => ("ENOENT",     "No such file or directory - SPI device does not exist"),
    5   => ("EIO",        "I/O error - bus communication failure, check wiring"),
    9   => ("EBADF",      "Bad file descriptor - device handle is invalid or closed"),
    11  => ("EAGAIN",     "Resource temporarily unavailable - retry the operation"),
    13  => ("EACCES",     "Permission denied - add user to spi group or run as root"),
    16  => ("EBUSY",      "Device or resource busy - another process is using the SPI bus"),
    19  => ("ENODEV",     "No such device - SPI adapter not found or not loaded"),
    22  => ("EINVAL",     "Invalid argument - check mode, speed, and bit settings"),
    110 => ("ETIMEDOUT",  "Connection timed out - device not responding"),
)

function spi_error(operation::String, dev::SPIDevice)
    errno = get_errno()
    ctx = " [bus=$(dev.bus), cs=$(dev.cs)]"
    if haskey(ERRNO_NAMES, errno)
        name, desc = ERRNO_NAMES[errno]
        error("SPI $operation failed$ctx: $name (errno=$errno) - $desc")
    else
        error("SPI $operation failed$ctx: errno=$errno (unknown error code)")
    end
end

#=============================================================================
  ioctl Constants (from linux/spi/spidev.h)
=============================================================================#

const _IOC_NRSHIFT   = 0
const _IOC_TYPESHIFT  = 8
const _IOC_SIZESHIFT  = 16
const _IOC_DIRSHIFT   = 30

const _IOC_WRITE = UInt(1)
const _IOC_READ  = UInt(2)

function _IOC(dir::UInt, type::UInt8, nr::UInt8, size::UInt)
    return (dir << _IOC_DIRSHIFT) |
           (UInt(type) << _IOC_TYPESHIFT) |
           (UInt(nr) << _IOC_NRSHIFT) |
           (size << _IOC_SIZESHIFT)
end

_IOR(type::UInt8, nr::UInt8, size::UInt) = _IOC(_IOC_READ, type, nr, size)
_IOW(type::UInt8, nr::UInt8, size::UInt) = _IOC(_IOC_WRITE, type, nr, size)

const SPI_IOC_MAGIC = UInt8(0x6B)  # 'k'

const SPI_IOC_RD_MODE          = _IOR(SPI_IOC_MAGIC, 0x01, UInt(1))
const SPI_IOC_WR_MODE          = _IOW(SPI_IOC_MAGIC, 0x01, UInt(1))
const SPI_IOC_RD_BITS_PER_WORD = _IOR(SPI_IOC_MAGIC, 0x03, UInt(1))
const SPI_IOC_WR_BITS_PER_WORD = _IOW(SPI_IOC_MAGIC, 0x03, UInt(1))
const SPI_IOC_RD_MAX_SPEED_HZ  = _IOR(SPI_IOC_MAGIC, 0x04, UInt(4))
const SPI_IOC_WR_MAX_SPEED_HZ  = _IOW(SPI_IOC_MAGIC, 0x04, UInt(4))

spi_ioc_message(n::Int) = _IOW(SPI_IOC_MAGIC, 0x00, UInt(n * sizeof(SPIIocTransfer)))

#=============================================================================
  Data Structures for ioctl
=============================================================================#

# struct spi_ioc_transfer (32 bytes, matches kernel layout)
struct SPIIocTransfer
    tx_buf::UInt64
    rx_buf::UInt64
    len::UInt32
    speed_hz::UInt32
    delay_usecs::UInt16
    bits_per_word::UInt8
    cs_change::UInt8
    tx_nbits::UInt8
    rx_nbits::UInt8
    word_delay_usecs::UInt8
    pad::UInt8
end

const ZERO_XFER_TAIL = (UInt32(0), UInt16(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))

#=============================================================================
  ioctl Wrapper
=============================================================================#

ioctl(dev::SPIDevice, request, arg) = ccall(:ioctl, Cint, (Cint, Culong, Clong), fd(dev.io), request, arg)

#=============================================================================
  Constructors
=============================================================================#

"""
    SPIDevice(bus::Int, cs::Int; mode=SPI_MODE_0, speed_hz=1_000_000, bits_per_word=8)

Open an SPI device at `/dev/spidevB.C` and configure mode, speed, and word size.
"""
function SPIDevice(bus::Int, cs::Int;
                   mode::UInt8=SPI_MODE_0,
                   speed_hz::UInt32=UInt32(1_000_000),
                   bits_per_word::UInt8=UInt8(8))
    path = "/dev/spidev$bus.$cs"
    io = open(path, "r+")

    scratch = zeros(UInt8, SPI_BUFFER_SIZE)
    xfer_buf = zeros(UInt8, 2 * sizeof(SPIIocTransfer))

    dev = SPIDevice(io, bus, cs, mode, speed_hz, bits_per_word, scratch, xfer_buf)

    set_mode(dev, mode)
    set_speed(dev, speed_hz)
    set_bits_per_word(dev, bits_per_word)

    return dev
end

"""
    SPIDevice(f::Function, bus::Int, cs::Int; kwargs...)

Open an SPI device, call `f(dev)`, then close.
Useful with do-block syntax for automatic cleanup.
"""
function SPIDevice(f::Function, bus::Int, cs::Int; kwargs...)
    dev = SPIDevice(bus, cs; kwargs...)
    try
        f(dev)
    finally
        close(dev)
    end
end

#=============================================================================
  Configuration
=============================================================================#

"""
    set_mode(dev::SPIDevice, mode::UInt8)

Set the SPI mode (SPI_MODE_0 through SPI_MODE_3, optionally OR'd with flags).
"""
function set_mode(dev::SPIDevice, mode::UInt8)
    dev.scratch[1] = mode
    ret = ioctl(dev, UInt(SPI_IOC_WR_MODE), pointer(dev.scratch))
    ret < 0 && spi_error("set_mode", dev)
    dev.mode = mode
    return ret
end

"""
    get_mode(dev::SPIDevice) -> UInt8

Read the current SPI mode from the device.
"""
function get_mode(dev::SPIDevice)
    ret = ioctl(dev, UInt(SPI_IOC_RD_MODE), pointer(dev.scratch))
    ret < 0 && spi_error("get_mode", dev)
    return dev.scratch[1]
end

"""
    set_speed(dev::SPIDevice, speed_hz::UInt32)

Set the maximum SPI clock speed in Hz.
"""
function set_speed(dev::SPIDevice, speed_hz::UInt32)
    unsafe_store!(Ptr{UInt32}(pointer(dev.scratch)), speed_hz)
    ret = ioctl(dev, UInt(SPI_IOC_WR_MAX_SPEED_HZ), pointer(dev.scratch))
    ret < 0 && spi_error("set_speed", dev)
    dev.speed_hz = speed_hz
    return ret
end

"""
    get_speed(dev::SPIDevice) -> UInt32

Read the current maximum SPI clock speed from the device.
"""
function get_speed(dev::SPIDevice)
    ret = ioctl(dev, UInt(SPI_IOC_RD_MAX_SPEED_HZ), pointer(dev.scratch))
    ret < 0 && spi_error("get_speed", dev)
    return unsafe_load(Ptr{UInt32}(pointer(dev.scratch)))
end

"""
    set_bits_per_word(dev::SPIDevice, bits::UInt8)

Set the number of bits per SPI word (typically 8).
"""
function set_bits_per_word(dev::SPIDevice, bits::UInt8)
    dev.scratch[1] = bits
    ret = ioctl(dev, UInt(SPI_IOC_WR_BITS_PER_WORD), pointer(dev.scratch))
    ret < 0 && spi_error("set_bits_per_word", dev)
    dev.bits_per_word = bits
    return ret
end

"""
    get_bits_per_word(dev::SPIDevice) -> UInt8

Read the current bits-per-word setting from the device.
"""
function get_bits_per_word(dev::SPIDevice)
    ret = ioctl(dev, UInt(SPI_IOC_RD_BITS_PER_WORD), pointer(dev.scratch))
    ret < 0 && spi_error("get_bits_per_word", dev)
    return dev.scratch[1]
end

#=============================================================================
  Transfer Operations
=============================================================================#

function do_xfer(dev::SPIDevice, n::Int, operation::String="transfer")
    ret = ioctl(dev, UInt(spi_ioc_message(n)), pointer(dev.xfer_buf))
    ret < 0 && spi_error(operation, dev)
    return ret
end

"""
    transfer!(dev::SPIDevice, tx::AbstractVector{UInt8}, rx::AbstractVector{UInt8}) -> Int

Full-duplex SPI transfer. `tx` and `rx` must have the same length.
Data is clocked out from `tx` while simultaneously clocking in to `rx`.
"""
function transfer!(dev::SPIDevice, tx::AbstractVector{UInt8}, rx::AbstractVector{UInt8})
    len = length(tx)
    length(rx) != len && error("transfer!: tx and rx must have same length (tx=$len, rx=$(length(rx)))")

    xfer = SPIIocTransfer(UInt64(pointer(tx)), UInt64(pointer(rx)), UInt32(len), ZERO_XFER_TAIL...)
    unsafe_store!(Ptr{SPIIocTransfer}(pointer(dev.xfer_buf)), xfer)

    return do_xfer(dev, 1)
end

"""
    Base.write(dev::SPIDevice, data::AbstractVector{UInt8}) -> Int

Write bytes to the SPI device (half-duplex TX, received data discarded).
"""
function Base.write(dev::SPIDevice, data::AbstractVector{UInt8})
    len = length(data)
    len > SPI_BUFFER_SIZE && error("write: length $len exceeds buffer size $SPI_BUFFER_SIZE")

    xfer = SPIIocTransfer(UInt64(pointer(data)), UInt64(0), UInt32(len), ZERO_XFER_TAIL...)
    unsafe_store!(Ptr{SPIIocTransfer}(pointer(dev.xfer_buf)), xfer)

    return do_xfer(dev, 1, "write")
end

"""
    Base.read(dev::SPIDevice, n::Integer) -> Vector{UInt8}

Read `n` bytes from the SPI device (half-duplex RX, TX sends zeros).
"""
function Base.read(dev::SPIDevice, n::Integer)
    dest = Vector{UInt8}(undef, n)
    read!(dev, dest)
    return dest
end

"""
    Base.read!(dev::SPIDevice, dest::AbstractVector{UInt8}) -> typeof(dest)

Read into `dest` from the SPI device (half-duplex RX, TX sends zeros).
"""
function Base.read!(dev::SPIDevice, dest::AbstractVector{UInt8})
    len = length(dest)
    len > SPI_BUFFER_SIZE && error("read!: length $len exceeds buffer size $SPI_BUFFER_SIZE")

    fill!(@view(dev.scratch[1:len]), 0x00)

    xfer = SPIIocTransfer(UInt64(pointer(dev.scratch)), UInt64(pointer(dest)), UInt32(len), ZERO_XFER_TAIL...)
    unsafe_store!(Ptr{SPIIocTransfer}(pointer(dev.xfer_buf)), xfer)

    do_xfer(dev, 1, "read")
    return dest
end

"""
    write_then_read!(dev::SPIDevice, tx::AbstractVector{UInt8}, rx::AbstractVector{UInt8}) -> Int

Write-then-read as two chained SPI transfers with CS held low between them.
Useful for register reads: send command/address bytes in `tx`, receive response in `rx`.
"""
function write_then_read!(dev::SPIDevice, tx::AbstractVector{UInt8}, rx::AbstractVector{UInt8})
    tx_len = length(tx)
    rx_len = length(rx)
    rx_len > SPI_BUFFER_SIZE && error("write_then_read!: rx length $rx_len exceeds buffer size $SPI_BUFFER_SIZE")

    xfer_ptr = Ptr{SPIIocTransfer}(pointer(dev.xfer_buf))

    # First transfer: write, keep CS asserted
    unsafe_store!(xfer_ptr,
        SPIIocTransfer(UInt64(pointer(tx)), UInt64(0), UInt32(tx_len), ZERO_XFER_TAIL...), 1)

    # Second transfer: read (TX sends zeros)
    fill!(@view(dev.scratch[1:rx_len]), 0x00)
    unsafe_store!(xfer_ptr,
        SPIIocTransfer(UInt64(pointer(dev.scratch)), UInt64(pointer(rx)), UInt32(rx_len), ZERO_XFER_TAIL...), 2)

    return do_xfer(dev, 2, "write_then_read")
end

end # module SPI
