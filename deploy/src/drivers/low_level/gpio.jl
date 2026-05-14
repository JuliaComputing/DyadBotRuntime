#=============================================================================
  GPIO Wrapper using Linux chardev API

  Provides Julia interface for GPIO control via /dev/gpiochipN.
  Requires kernel 5.10+ for the gpio_v2 ioctl interface.
=============================================================================#

module GPIO

using FileWatching: poll_fd

export GPIOController, GPIOPin, INPUT, OUTPUT, EDGE_RISING, EDGE_FALLING, EDGE_BOTH
export open_gpio, close_gpio, request_input, request_output, request_input_edge
export set_value, get_value, release_pin
export poll_event, read_event, GPIOEvent

#=============================================================================
  ioctl Constants (from linux/gpio.h)
=============================================================================#

# ioctl direction bits
const _IOC_NONE  = UInt(0)
const _IOC_WRITE = UInt(1)
const _IOC_READ  = UInt(2)

const _IOC_NRBITS   = 8
const _IOC_TYPEBITS = 8
const _IOC_SIZEBITS = 14
const _IOC_DIRBITS  = 2

const _IOC_NRSHIFT   = 0
const _IOC_TYPESHIFT = _IOC_NRSHIFT + _IOC_NRBITS
const _IOC_SIZESHIFT = _IOC_TYPESHIFT + _IOC_TYPEBITS
const _IOC_DIRSHIFT  = _IOC_SIZESHIFT + _IOC_SIZEBITS

function _IOC(dir::UInt, type::UInt8, nr::UInt8, size::UInt)
    return (dir << _IOC_DIRSHIFT) |
           (UInt(type) << _IOC_TYPESHIFT) |
           (UInt(nr) << _IOC_NRSHIFT) |
           (size << _IOC_SIZESHIFT)
end

_IOR(type::UInt8, nr::UInt8, size::UInt) = _IOC(_IOC_READ, type, nr, size)
_IOWR(type::UInt8, nr::UInt8, size::UInt) = _IOC(_IOC_READ | _IOC_WRITE, type, nr, size)

const GPIO_IOCTL_MAGIC = UInt8(0xB4)

# Limits
const GPIO_MAX_NAME_SIZE = 32
const GPIO_V2_LINES_MAX = 64

# Line configuration flags
const GPIO_V2_LINE_FLAG_USED                 = UInt64(1 << 0)
const GPIO_V2_LINE_FLAG_ACTIVE_LOW           = UInt64(1 << 1)
const GPIO_V2_LINE_FLAG_INPUT                = UInt64(1 << 2)
const GPIO_V2_LINE_FLAG_OUTPUT               = UInt64(1 << 3)
const GPIO_V2_LINE_FLAG_EDGE_RISING          = UInt64(1 << 4)
const GPIO_V2_LINE_FLAG_EDGE_FALLING         = UInt64(1 << 5)
const GPIO_V2_LINE_FLAG_OPEN_DRAIN           = UInt64(1 << 6)
const GPIO_V2_LINE_FLAG_OPEN_SOURCE          = UInt64(1 << 7)
const GPIO_V2_LINE_FLAG_BIAS_PULL_UP         = UInt64(1 << 8)
const GPIO_V2_LINE_FLAG_BIAS_PULL_DOWN       = UInt64(1 << 9)
const GPIO_V2_LINE_FLAG_BIAS_DISABLED        = UInt64(1 << 10)
const GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME = UInt64(1 << 11)
const GPIO_V2_LINE_FLAG_EVENT_CLOCK_HTE      = UInt64(1 << 12)

# Line event IDs
const GPIO_V2_LINE_EVENT_RISING_EDGE  = UInt32(1)
const GPIO_V2_LINE_EVENT_FALLING_EDGE = UInt32(2)

#=============================================================================
  ioctl Data Structures
=============================================================================#

# struct gpiochip_info - chip metadata
struct GPIOChipInfo
    name::NTuple{GPIO_MAX_NAME_SIZE, UInt8}
    label::NTuple{GPIO_MAX_NAME_SIZE, UInt8}
    lines::UInt32
end

# struct gpio_v2_line_values
# Used for getting/setting line values - just two 64-bit masks
struct GPIOV2LineValues
    bits::UInt64   # values (bit per line)
    mask::UInt64   # which lines to get/set
end

const GPIO_V2_LINE_NUM_ATTRS_MAX = 10

# struct gpio_v2_line_attribute
struct GPIOV2LineAttribute
    id::UInt32
    padding::UInt32
    data::UInt64  # union: flags, values, debounce_period_us
end

# struct gpio_v2_line_config_attribute
struct GPIOV2LineConfigAttribute
    attr::GPIOV2LineAttribute
    mask::UInt64
end

# struct gpio_v2_line_config
struct GPIOV2LineConfig
    flags::UInt64
    num_attrs::UInt32
    padding::NTuple{5, UInt32}
    attrs::NTuple{GPIO_V2_LINE_NUM_ATTRS_MAX, GPIOV2LineConfigAttribute}
end

# struct gpio_v2_line_request
struct GPIOV2LineRequest
    offsets::NTuple{GPIO_V2_LINES_MAX, UInt32}
    consumer::NTuple{GPIO_MAX_NAME_SIZE, UInt8}
    config::GPIOV2LineConfig
    num_lines::UInt32
    event_buffer_size::UInt32
    padding::NTuple{5, UInt32}
    fd::Int32
end

# struct gpio_v2_line_event
struct GPIOV2LineEvent
    timestamp_ns::UInt64
    id::UInt32              # GPIO_V2_LINE_EVENT_RISING_EDGE or FALLING_EDGE
    offset::UInt32          # which line triggered the event
    seqno::UInt32           # sequence number in file
    line_seqno::UInt32      # sequence number for this line
    padding::NTuple{6, UInt32}
end

# ioctl numbers
const GPIO_GET_CHIPINFO_IOCTL = _IOR(GPIO_IOCTL_MAGIC, 0x01, UInt(sizeof(GPIOChipInfo)))
const GPIO_V2_GET_LINE_IOCTL = _IOWR(GPIO_IOCTL_MAGIC, 0x07, UInt(sizeof(GPIOV2LineRequest)))
const GPIO_V2_LINE_GET_VALUES_IOCTL = _IOWR(GPIO_IOCTL_MAGIC, 0x0E, UInt(sizeof(GPIOV2LineValues)))
const GPIO_V2_LINE_SET_VALUES_IOCTL = _IOWR(GPIO_IOCTL_MAGIC, 0x0F, UInt(sizeof(GPIOV2LineValues)))

#=============================================================================
  Direction Enum
=============================================================================#

@enum LineDirection begin
    INPUT
    OUTPUT
end

@enum EdgeType begin
    EDGE_RISING
    EDGE_FALLING
    EDGE_BOTH
end

"""
    GPIOEvent

Represents a GPIO edge event with timestamp and edge type.
"""
struct GPIOEvent
    timestamp_ns::UInt64
    is_rising::Bool
    pin::Int
end

#=============================================================================
  Handle Types
=============================================================================#

"""
    GPIOPin

A single GPIO pin with its own line handle.
Created by `request_input` or `request_output`.
"""
mutable struct GPIOPin
    pin::Int
    io::IOStream
    direction::LineDirection
    values::Vector{UInt8}  # 16 bytes for GPIOV2LineValues

    function GPIOPin(pin::Int, io::IOStream, direction::LineDirection, values::Vector{UInt8})
        obj = new(pin, io, direction, values)
        finalizer(close, obj)
        return obj
    end
end

Base.close(pin::GPIOPin) = isopen(pin.io) && close(pin.io)
Base.isopen(pin::GPIOPin) = isopen(pin.io)

"""
    GPIOController

Main GPIO controller. Holds the chip file descriptor.
Use `request_input` or `request_output` to get GPIOPin handles.
"""
mutable struct GPIOController
    io::IOStream
    path::String
    num_lines::UInt32
    req_buf::Vector{UInt8}  # Pre-allocated buffer for line requests

    function GPIOController(io::IOStream, path::String, num_lines::UInt32, req_buf::Vector{UInt8})
        obj = new(io, path, num_lines, req_buf)
        finalizer(close, obj)
        return obj
    end
end

Base.close(gpio::GPIOController) = isopen(gpio.io) && close(gpio.io)
Base.isopen(gpio::GPIOController) = isopen(gpio.io)

#=============================================================================
  ioctl Wrapper
=============================================================================#

ioctl(io::IOStream, request, arg::Ref{T}) where T = ccall(:ioctl, Cint, (Cint, Culong, Ref{T}), fd(io), request, arg)
ioctl(io::IOStream, request, arg) = ccall(:ioctl, Cint, (Cint, Culong, Ptr{Cvoid}), fd(io), request, arg)

#=============================================================================
  Helper Functions
=============================================================================#

"""Create a consumer label NTuple from a string."""
function make_consumer_label(s::String, ::Val{N}=Val(GPIO_MAX_NAME_SIZE)) where N
    ntuple(Val(N)) do i
        i <= ncodeunits(s) ? codeunit(s, i) : UInt8(0)
    end
end

const ZERO_PADDING_5 = ntuple(_ -> UInt32(0), Val(5))
const ZERO_ATTR = GPIOV2LineAttribute(UInt32(0), UInt32(0), UInt64(0))
const ZERO_CONFIG_ATTR = GPIOV2LineConfigAttribute(ZERO_ATTR, UInt64(0))
const ZERO_ATTRS = ntuple(_ -> ZERO_CONFIG_ATTR, Val(GPIO_V2_LINE_NUM_ATTRS_MAX))
const ZERO_OFFSETS = ntuple(_ -> UInt32(0), Val(GPIO_V2_LINES_MAX))

"""Create a line request struct."""
function make_line_request(pin::Int, consumer::String, flags::UInt64, event_buffer_size::UInt32,
                           ::Val{N}=Val(GPIO_V2_LINES_MAX)) where N
    offsets = ntuple(Val(N)) do i
        i == 1 ? UInt32(pin) : UInt32(0)
    end
    config = GPIOV2LineConfig(flags, UInt32(0), ZERO_PADDING_5, ZERO_ATTRS)
    return GPIOV2LineRequest(offsets, make_consumer_label(consumer), config,
                             UInt32(1), event_buffer_size, ZERO_PADDING_5, Int32(-1))
end

#=============================================================================
  GPIO Controller Operations
=============================================================================#

"""
    open_gpio(path::String="/dev/gpiochip0") -> GPIOController

Open a GPIO chip device.
"""
function open_gpio(path::String="/dev/gpiochip0")
    io = open(path, "r")

    info_buf = zeros(UInt8, sizeof(GPIOChipInfo))
    ret = ioctl(io, GPIO_GET_CHIPINFO_IOCTL, info_buf)
    if ret < 0
        close(io)
        error("Failed to get GPIO chip info (errno: $(Base.Libc.errno()))")
    end

    num_lines = unsafe_load(Ptr{UInt32}(pointer(info_buf, 65)))

    # Pre-allocate request buffer for line requests
    req_buf = zeros(UInt8, sizeof(GPIOV2LineRequest))

    return GPIOController(io, path, num_lines, req_buf)
end

"""
    open_gpio(f::Function, path::String="/dev/gpiochip0")

Open a GPIO chip, call `f(gpio)`, then close the chip.
Useful with do-block syntax for automatic cleanup.
"""
function open_gpio(f::Function, path::String="/dev/gpiochip0")
    gpio = open_gpio(path)
    try
        f(gpio)
    finally
        close(gpio)
    end
end

"""
    close_gpio(gpio::GPIOController)

Close the GPIO chip. Equivalent to `close(gpio)`.
"""
close_gpio(gpio::GPIOController) = close(gpio)

#=============================================================================
  Pin Request Operations
=============================================================================#

"""
    request_output(gpio::GPIOController, pin::Int, consumer::String, default_value::Int=0) -> GPIOPin

Request a pin for output with an initial value.
"""
function request_output(gpio::GPIOController, pin::Int, consumer::String, default_value::Int=0)
    if pin < 0 || pin >= gpio.num_lines
        error("Invalid GPIO pin $pin (chip has $(gpio.num_lines) lines)")
    end

    req = make_line_request(pin, consumer, GPIO_V2_LINE_FLAG_OUTPUT, UInt32(0))
    unsafe_store!(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)), req)

    ret = ioctl(gpio.io, GPIO_V2_GET_LINE_IOCTL, gpio.req_buf)
    if ret < 0
        error("Failed to request GPIO output for pin $pin (errno: $(Base.Libc.errno()))")
    end

    result = unsafe_load(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)))
    values_buf = zeros(UInt8, 16)
    values_buf[9] = 0x01  # mask = 1 (line 0)
    pin_obj = GPIOPin(pin, fdio(result.fd, true), OUTPUT, values_buf)

    if default_value != 0
        set_value(pin_obj, default_value)
    end

    return pin_obj
end

"""
    request_input(gpio::GPIOController, pin::Int, consumer::String) -> GPIOPin

Request a pin for input.
"""
function request_input(gpio::GPIOController, pin::Int, consumer::String)
    if pin < 0 || pin >= gpio.num_lines
        error("Invalid GPIO pin $pin (chip has $(gpio.num_lines) lines)")
    end

    req = make_line_request(pin, consumer, GPIO_V2_LINE_FLAG_INPUT, UInt32(0))
    unsafe_store!(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)), req)

    ret = ioctl(gpio.io, GPIO_V2_GET_LINE_IOCTL, gpio.req_buf)
    if ret < 0
        error("Failed to request GPIO input for pin $pin (errno: $(Base.Libc.errno()))")
    end

    result = unsafe_load(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)))
    values_buf = zeros(UInt8, 16)
    values_buf[9] = 0x01
    return GPIOPin(pin, fdio(result.fd, true), INPUT, values_buf)
end

"""
    request_input_edge(gpio::GPIOController, pin::Int, consumer::String, edge::EdgeType=EDGE_BOTH; buffer_size::Int=16) -> GPIOPin

Request a pin for input with edge detection.
The returned pin's fd can be polled for edge events.
"""
function request_input_edge(gpio::GPIOController, pin::Int, consumer::String, edge::EdgeType=EDGE_BOTH; buffer_size::Int=16)
    if pin < 0 || pin >= gpio.num_lines
        error("Invalid GPIO pin $pin (chip has $(gpio.num_lines) lines)")
    end

    flags = GPIO_V2_LINE_FLAG_INPUT
    if edge == EDGE_RISING
        flags |= GPIO_V2_LINE_FLAG_EDGE_RISING
    elseif edge == EDGE_FALLING
        flags |= GPIO_V2_LINE_FLAG_EDGE_FALLING
    else
        flags |= GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EDGE_FALLING
    end

    req = make_line_request(pin, consumer, flags, UInt32(buffer_size))
    unsafe_store!(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)), req)

    ret = ioctl(gpio.io, GPIO_V2_GET_LINE_IOCTL, gpio.req_buf)
    if ret < 0
        error("Failed to request GPIO input with edge detection for pin $pin (errno: $(Base.Libc.errno()))")
    end

    result = unsafe_load(Ptr{GPIOV2LineRequest}(pointer(gpio.req_buf)))
    values_buf = zeros(UInt8, 16)
    values_buf[9] = 0x01
    return GPIOPin(pin, fdio(result.fd, true), INPUT, values_buf)
end

#=============================================================================
  Edge Event Operations
=============================================================================#

"""
    poll_event(pin::GPIOPin, timeout_ms::Int=-1) -> Bool

Wait for an edge event on the pin. Returns true if an event is ready, false on timeout.
timeout_ms: -1 = block forever, 0 = non-blocking, >0 = timeout in milliseconds
"""
function poll_event(pin::GPIOPin, timeout_ms::Int=-1)
    timeout_s = timeout_ms < 0 ? Inf : timeout_ms / 1000.0
    result = poll_fd(fd(pin.io), timeout_s; readable=true, writable=false)
    return result.readable
end

"""
    read_event(pin::GPIOPin) -> GPIOEvent

Read a single edge event from the pin. Blocks if no event is available.
Call poll_event first if you want non-blocking behavior.
"""
function read_event(pin::GPIOPin)
    event = read(pin.io, GPIOV2LineEvent)
    is_rising = event.id == GPIO_V2_LINE_EVENT_RISING_EDGE
    return GPIOEvent(event.timestamp_ns, is_rising, pin.pin)
end

#=============================================================================
  Pin Value Operations
=============================================================================#

"""
    set_value(pin::GPIOPin, value::Int) -> Int

Set the value of an output pin (0 or 1).
"""
function set_value(pin::GPIOPin, value::Int)
    # Buffer layout: bits (8 bytes) + mask (8 bytes)
    # Mask is pre-initialized to 1 (line 0). Only need to set bits[0].
    pin.values[1] = value != 0 ? 0x01 : 0x00

    ret = ioctl(pin.io, GPIO_V2_LINE_SET_VALUES_IOCTL, pin.values)
    if ret < 0
        error("Failed to set GPIO value for pin $(pin.pin) (errno: $(Base.Libc.errno()))")
    end

    return 0
end

"""
    get_value(pin::GPIOPin) -> Int

Get the value of a pin (0 or 1).
"""
function get_value(pin::GPIOPin)
    # Buffer layout: bits (8 bytes) + mask (8 bytes)
    # Mask is pre-initialized to 1 (line 0). Only need to read bits[0].
    ret = ioctl(pin.io, GPIO_V2_LINE_GET_VALUES_IOCTL, pin.values)
    if ret < 0
        error("Failed to get GPIO value for pin $(pin.pin) (errno: $(Base.Libc.errno()))")
    end

    return (pin.values[1] & 0x01) != 0 ? 1 : 0
end

"""
    release_pin(pin::GPIOPin)

Release a GPIO pin handle. Equivalent to `close(pin)`.
"""
release_pin(pin::GPIOPin) = close(pin)

end # module GPIO
