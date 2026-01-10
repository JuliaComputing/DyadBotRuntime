#=============================================================================
  GPIO Wrapper using sysfs

  Provides Julia interface for GPIO control via /sys/class/gpio/
  No external library dependencies - pure file I/O.
=============================================================================#

module GPIO

export GPIOPin, INPUT, OUTPUT
export export_pin, unexport_pin, set_direction, set_value, get_value

const GPIO_SYSFS_PATH = "/sys/class/gpio"

# Line direction
@enum LineDirection begin
    INPUT
    OUTPUT
end

#=============================================================================
  GPIO Pin Handle
=============================================================================#

mutable struct GPIOPin
    pin::Int
    path::String
    exported::Bool
    direction::LineDirection
end

#=============================================================================
  Low-level sysfs Operations
=============================================================================#

"""
    sysfs_write(path::String, value::String)

Write a value to a sysfs file.
"""
function sysfs_write(path::String, value::String)
    f = open(path, "w")
    try
        write(f, value)
    finally
        close(f)
    end
end

"""
    sysfs_read(path::String) -> String

Read a value from a sysfs file.
"""
function sysfs_read(path::String)
    f = open(path, "r")
    try
        return strip(read(f, String))
    finally
        close(f)
    end
end

#=============================================================================
  Pin Export/Unexport
=============================================================================#

"""
    export_pin(pin::Int) -> GPIOPin

Export a GPIO pin for userspace control.
"""
function export_pin(pin::Int)
    pin_path = joinpath(GPIO_SYSFS_PATH, "gpio$pin")

    # Export if not already exported
    if !isdir(pin_path)
        export_file = joinpath(GPIO_SYSFS_PATH, "export")
        sysfs_write(export_file, string(pin))
        # Wait for sysfs to create the directory
        sleep(0.05)
    end

    if !isdir(pin_path)
        error("Failed to export GPIO pin $pin")
    end

    return GPIOPin(pin, pin_path, true, INPUT)
end

"""
    unexport_pin(gpio::GPIOPin)

Unexport a GPIO pin.
"""
function unexport_pin(gpio::GPIOPin)
    if gpio.exported
        unexport_file = joinpath(GPIO_SYSFS_PATH, "unexport")
        sysfs_write(unexport_file, string(gpio.pin))
        gpio.exported = false
    end
end

#=============================================================================
  Direction Control
=============================================================================#

"""
    set_direction(gpio::GPIOPin, direction::LineDirection)

Set the GPIO pin direction (INPUT or OUTPUT).
"""
function set_direction(gpio::GPIOPin, direction::LineDirection)
    direction_file = joinpath(gpio.path, "direction")
    dir_str = direction == OUTPUT ? "out" : "in"
    sysfs_write(direction_file, dir_str)
    gpio.direction = direction
end

#=============================================================================
  Value Read/Write
=============================================================================#

"""
    set_value(gpio::GPIOPin, value::Int)

Set the value of an output pin (0 or 1).
"""
function set_value(gpio::GPIOPin, value::Int)
    value_file = joinpath(gpio.path, "value")
    sysfs_write(value_file, value == 0 ? "0" : "1")
end

"""
    get_value(gpio::GPIOPin) -> Int

Get the value of a pin (0 or 1).
"""
function get_value(gpio::GPIOPin)
    value_file = joinpath(gpio.path, "value")
    val_str = sysfs_read(value_file)
    return parse(Int, val_str)
end

#=============================================================================
  Convenience Functions (compatible with previous API)
=============================================================================#

# Compatibility types
const GPIOChip = Nothing
const GPIOLine = GPIOPin

"""
    open_chip(path::String="/dev/gpiochip0") -> Nothing

Dummy function for API compatibility. sysfs doesn't use chip handles.
"""
function open_chip(path::String="/dev/gpiochip0")
    return nothing
end

"""
    close_chip(chip::Nothing)

Dummy function for API compatibility.
"""
function close_chip(chip::Nothing)
    return nothing
end

"""
    get_line(chip::Nothing, pin::Int) -> GPIOPin

Export and return a GPIO pin. Chip argument ignored (sysfs doesn't use it).
"""
function get_line(chip::Nothing, pin::Int)
    return export_pin(pin)
end

"""
    request_output(gpio::GPIOPin, consumer::String, default_value::Int=0)

Configure pin as output with initial value.
"""
function request_output(gpio::GPIOPin, consumer::String, default_value::Int=0)
    set_direction(gpio, OUTPUT)
    set_value(gpio, default_value)
    return 0
end

"""
    request_input(gpio::GPIOPin, consumer::String)

Configure pin as input.
"""
function request_input(gpio::GPIOPin, consumer::String)
    set_direction(gpio, INPUT)
    return 0
end

"""
    release_line(gpio::GPIOPin)

Unexport the GPIO pin.
"""
function release_line(gpio::GPIOPin)
    unexport_pin(gpio)
end

"""
    digitalWrite(gpio::GPIOPin, value::Bool)

Set a GPIO output pin high (true) or low (false).
"""
function digitalWrite(gpio::GPIOPin, value::Bool)
    set_value(gpio, value ? 1 : 0)
end

"""
    digitalRead(gpio::GPIOPin) -> Bool

Read a GPIO input pin state.
"""
function digitalRead(gpio::GPIOPin)
    return get_value(gpio) == 1
end

end # module GPIO
