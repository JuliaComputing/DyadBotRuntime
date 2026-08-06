module BalansBotDrivers

using Timers
using Configurations
using Libevdev

include(joinpath(@__DIR__, "drivers", "low_level", "gpio.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "pwm.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "spi.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "i2c.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "shift_driver.jl"))

include(joinpath(@__DIR__, "drivers", "devices", "seven_seg.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "ws2812_driver.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "icm42688pc.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "motor.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "ads7142.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "dac43701.jl"))

const CM_PRESENT=23
const D1=5
const D2=24
const NOSE_RGB=18
const AIN1 = 4
const BIN1 = 17
const M1A = 10
const M2A = 9
const PWMA_LEFT = 19
const PWMB_RIGHT = 18 # both have hardware PWM
const LTRANS_OE = 11 # enables the 3.3V <-> 5V level translator
const STBY_PIN = 26

end # module BalansBotDrivers
