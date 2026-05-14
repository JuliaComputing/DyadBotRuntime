module Motor
using ..GPIO
using ..PWM

struct Motor
    pwm_fwd::PWMChannel
    pwm_rev::PWMChannel
end

function set_drive!(m::Motor, duty::Float64)
    if duty > 0
        set_duty_cycle_ratio(m.pwm_fwd, duty)
        set_duty_cycle_ratio(m.pwm_rev, 0.0)
    elseif duty < 0
        set_duty_cycle_ratio(m.pwm_fwd, 0.0)
        set_duty_cycle_ratio(m.pwm_rev, -duty)
    else
        set_duty_cycle_ratio(m.pwm_fwd, 0.0)
        set_duty_cycle_ratio(m.pwm_rev, 0.0)
    end
end

function enable!(m::Motor)
    enable(m.pwm_fwd)
    enable(m.pwm_rev)
end

function disable!(m::Motor)
    disable(m.pwm_fwd)
    disable(m.pwm_rev)
end

function Base.close(m::Motor)
    unexport_channel(m.pin_fwd)
    unexport_channel(m.pin_rev)
end

export Motor, set_drive!, enable!, disable!

end