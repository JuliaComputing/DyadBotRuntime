module Motor

using Libevdev
using ..GPIO
using ..PWM

struct Motor
    pwm_fwd::PWMChannel
    pwm_rev::PWMChannel
    encoder::AxisTracker
    axis_code::UInt
end

function set_drive!(m::Motor, duty::Float64)
    if duty > 0
        set_duty_cycle_ratio(m.pwm_fwd, 1-duty)
        set_duty_cycle_ratio(m.pwm_rev, 1.0)
    elseif duty < 0
        set_duty_cycle_ratio(m.pwm_fwd, 1.0)
        set_duty_cycle_ratio(m.pwm_rev, 1+duty)
    else
        set_duty_cycle_ratio(m.pwm_fwd, 0.0)
        set_duty_cycle_ratio(m.pwm_rev, 0.0)
    end
end

function enable!(m::Motor)
    set_period_hz(m.pwm_fwd, 10_000)
    set_period_hz(m.pwm_rev, 10_000)
    set_duty_cycle_ratio(m.pwm_fwd, 0.0)
    set_duty_cycle_ratio(m.pwm_rev, 0.0)
    set_polarity(m.pwm_fwd, false)
    set_polarity(m.pwm_rev, false)
    enable(m.pwm_fwd)
    enable(m.pwm_rev)
end

angle(m::Motor) = rel(m.encoder, m.axis_code)

function disable!(m::Motor)
    disable(m.pwm_fwd)
    disable(m.pwm_rev)
end

function Base.close(m::Motor)
    close(m.encoder)
    unexport_channel(m.pwm_fwd)
    unexport_channel(m.pwm_rev)
end

export Motor, set_drive!, enable!, disable!

end