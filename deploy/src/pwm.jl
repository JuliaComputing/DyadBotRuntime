#=============================================================================
  PWM Wrapper using sysfs

  Provides Julia interface for hardware PWM via /sys/class/pwm/

  On Raspberry Pi:
    - pwmchip0 has 2 channels
    - GPIO 18 -> pwm0 (channel 0)
    - GPIO 19 -> pwm1 (channel 1)

  Requires dtoverlay=pwm-2chan in /boot/config.txt
=============================================================================#

module PWM

export PWMChannel, PWMChip
export open_chip, export_channel, unexport_channel
export set_period_ns, set_duty_cycle_ns, set_duty_cycle_percent
export enable, disable, set_polarity

const PWM_SYSFS_PATH = "/sys/class/pwm"

# Map GPIO pins to PWM channels (Raspberry Pi specific)
const GPIO_TO_PWM_CHANNEL = Dict(
    18 => 0,  # GPIO 18 -> pwm0
    19 => 1,  # GPIO 19 -> pwm1
    12 => 0,  # GPIO 12 -> pwm0 (alternate)
    13 => 1,  # GPIO 13 -> pwm1 (alternate)
)

#=============================================================================
  PWM Types
=============================================================================#

struct PWMChip
    chip_num::Int
    path::String
end

mutable struct PWMChannel
    chip::PWMChip
    channel::Int
    path::String
    period_ns::Int
    exported::Bool
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

#=============================================================================
  Chip Operations
=============================================================================#

"""
    open_chip(chip_num::Int=0) -> PWMChip

Open a PWM chip.
"""
function open_chip(chip_num::Int=0)
    path = joinpath(PWM_SYSFS_PATH, "pwmchip$chip_num")
    if !isdir(path)
        error("PWM chip not found: $path. Ensure dtoverlay=pwm-2chan is in /boot/config.txt")
    end
    return PWMChip(chip_num, path)
end

#=============================================================================
  Channel Operations
=============================================================================#

"""
    export_channel(chip::PWMChip, channel::Int) -> PWMChannel

Export a PWM channel for use.
"""
function export_channel(chip::PWMChip, channel::Int)
    channel_path = joinpath(chip.path, "pwm$channel")

    # Export if not already exported
    if !isdir(channel_path)
        export_file = joinpath(chip.path, "export")
        sysfs_write(export_file, string(channel))
        # Wait for sysfs to create the directory
        sleep(0.1)
    end

    if !isdir(channel_path)
        error("Failed to export PWM channel $channel")
    end

    return PWMChannel(chip, channel, channel_path, 0, true)
end

"""
    export_channel_for_gpio(chip::PWMChip, gpio_pin::Int) -> PWMChannel

Export a PWM channel corresponding to the given GPIO pin.
"""
function export_channel_for_gpio(chip::PWMChip, gpio_pin::Int)
    if !haskey(GPIO_TO_PWM_CHANNEL, gpio_pin)
        error("GPIO pin $gpio_pin does not support hardware PWM")
    end
    channel = GPIO_TO_PWM_CHANNEL[gpio_pin]
    return export_channel(chip, channel)
end

"""
    unexport_channel(pwm::PWMChannel)

Unexport a PWM channel.
"""
function unexport_channel(pwm::PWMChannel)
    if pwm.exported
        unexport_file = joinpath(pwm.chip.path, "unexport")
        sysfs_write(unexport_file, string(pwm.channel))
    end
end

#=============================================================================
  Configuration
=============================================================================#

"""
    set_period_ns(pwm::PWMChannel, period_ns::Int)

Set the PWM period in nanoseconds.
"""
function set_period_ns(pwm::PWMChannel, period_ns::Int)
    period_file = joinpath(pwm.path, "period")
    sysfs_write(period_file, string(period_ns))
    pwm.period_ns = period_ns
end

"""
    set_period_hz(pwm::PWMChannel, freq_hz::Int)

Set the PWM period by frequency in Hz.
"""
function set_period_hz(pwm::PWMChannel, freq_hz::Int)
    period_ns = div(1_000_000_000, freq_hz)
    set_period_ns(pwm, period_ns)
end

"""
    set_duty_cycle_ns(pwm::PWMChannel, duty_ns::Int)

Set the duty cycle in nanoseconds (must be <= period).
"""
function set_duty_cycle_ns(pwm::PWMChannel, duty_ns::Int)
    duty_file = joinpath(pwm.path, "duty_cycle")
    sysfs_write(duty_file, string(duty_ns))
end

"""
    set_duty_cycle_percent(pwm::PWMChannel, percent::Float64)

Set the duty cycle as a percentage (0.0 to 100.0).
"""
function set_duty_cycle_percent(pwm::PWMChannel, percent::Float64)
    if pwm.period_ns == 0
        error("Period must be set before duty cycle")
    end
    duty_ns = round(Int, pwm.period_ns * percent / 100.0)
    set_duty_cycle_ns(pwm, duty_ns)
end

"""
    set_duty_cycle_ratio(pwm::PWMChannel, ratio::Float64)

Set the duty cycle as a ratio (0.0 to 1.0).
"""
function set_duty_cycle_ratio(pwm::PWMChannel, ratio::Float64)
    if pwm.period_ns == 0
        error("Period must be set before duty cycle")
    end
    duty_ns = round(Int, pwm.period_ns * clamp(ratio, 0.0, 1.0))
    set_duty_cycle_ns(pwm, duty_ns)
end

"""
    set_polarity(pwm::PWMChannel, inverted::Bool)

Set the PWM polarity. Must be called before enabling.
"""
function set_polarity(pwm::PWMChannel, inverted::Bool)
    polarity_file = joinpath(pwm.path, "polarity")
    sysfs_write(polarity_file, inverted ? "inversed" : "normal")
end

#=============================================================================
  Enable/Disable
=============================================================================#

"""
    enable(pwm::PWMChannel)

Enable the PWM output.
"""
function enable(pwm::PWMChannel)
    enable_file = joinpath(pwm.path, "enable")
    sysfs_write(enable_file, "1")
end

"""
    disable(pwm::PWMChannel)

Disable the PWM output.
"""
function disable(pwm::PWMChannel)
    enable_file = joinpath(pwm.path, "enable")
    sysfs_write(enable_file, "0")
end

#=============================================================================
  Convenience Functions
=============================================================================#

"""
    pwmWrite(pwm::PWMChannel, value::Int, max_value::Int=1024)

Set PWM duty cycle using a value from 0 to max_value (like Arduino/WiringPi style).
"""
function pwmWrite(pwm::PWMChannel, value::Int, max_value::Int=1024)
    ratio = clamp(value / max_value, 0.0, 1.0)
    set_duty_cycle_ratio(pwm, ratio)
end

end # module PWM
