module DyadBalans

using Timers
using Configurations
using StatsBase
include(joinpath(@__DIR__, "drivers", "low_level", "gpio.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "pwm.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "spi.jl"))
include(joinpath(@__DIR__, "drivers", "low_level", "shift_driver.jl"))

include(joinpath(@__DIR__, "drivers", "devices", "seven_seg.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "ws2812_driver.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "icm42688pc.jl"))
include(joinpath(@__DIR__, "drivers", "devices", "motor.jl"))

using .GPIO
using .PWM
using .SPI
using .ICM42688PC

module BalanceController
include(joinpath(@__DIR__, "balance_original.jl"))
end

const AIN1 = 4
const BIN1 = 17
const M1A = 10
const M2A = 9
const PWMA_LEFT = 19
const PWMB_RIGHT = 18 # both have hardware PWM
const LTRANS_OE = 11 # enables the 3.3V <-> 5V level translator
const STBY_PIN = 26

const PWM_FREQ_HZ = 1000  # 1kHz PWM frequency
const PWM_MAX_VALUE = 1024  # PWM duty cycle range (0-1024)

# Global handles for GPIO pins and PWM channels
mutable struct HardwareContext
    gpio::GPIO.GPIOController
    pio::PIOBlock
    chain::ShiftRegisterChain
    nose::WS2812
    pins::Vector{GPIO.GPIOPin}
    imu::ICM42688PC.ICM42688PC
    motor_a::Motor
    motor_b::Motor
end

"""
    shutdown!(hw::HardwareContext)

Safely shutdown hardware: clear shift register outputs, disable motors and level translator.
Called on Ctrl-C or program exit.
"""
function shutdown!(hw::HardwareContext)
    println(Core.stdout, "Shutting down hardware...")
    hw.chain[0:23] = false
    set_color!(hw.nose, 0, 0, 0)
    close(hw.chain)
    close(hw.nose)
    close(hw.pio)
    for pin in hw.pins
        close(pin)
    end
    close(hw.gpio)
    ICM42688PC.close!(hw.imu)
    close(hw.motor_a)
    close(hw.motor_b)
    println(Core.stdout, "Hardware shutdown complete.")
end

function handle_err(ec)
    if ec == 0
        return
    end
    println(Core.stdout, "Had error ", ec)
    return
end

const CM_PRESENT=23
const D1=5
const D2=24
const NOSE_RGB=18

@option mutable struct DyadBotOptions
    gyro_bias::Union{Nothing, Vector{Float64}} = nothing
    calibration_data::Union{Nothing, Vector{UInt8}} = nothing
end

function hsv_to_rgb(h::Real, s::Real, v::Real)
    h = mod(h, 360)
    c = v * s
    x = c * (1 - abs(mod(h / 60, 2) - 1))
    m = v - c
 
    r1, g1, b1 =
        h < 60   ? (c, x, 0.0) :
        h < 120  ? (x, c, 0.0) :
        h < 180  ? (0.0, c, x) :
        h < 240  ? (0.0, x, c) :
        h < 300  ? (x, 0.0, c) :
                   (c, 0.0, x)
 
    return (r1 + m, g1 + m, b1 + m)
end
function palette_to_rgb(x::Real, y::Real)
    r = sqrt(x^2 + y^2)
    θ = atan(y, x)                  # radians, (−π, π]
    h = rad2deg(θ)                  # degrees
    s = min(r, 1.0)                 # saturation = clamped radius
    return hsv_to_rgb(h, s, 1.0)
end

function (@main)(args)::Cint
    println(Core.stdout, "Balance car starting...")
    if !isfile("options.toml")
        to_toml("options.toml", DyadBotOptions(nothing, nothing); include_defaults=true)
    end
    options = from_toml(DyadBotOptions, "options.toml")

    # Initialize GPIO controller
    gpio = GPIO.open_gpio("/dev/gpiochip0")

    # Setup GPIO output pins for motor direction
    cm_present = GPIO.request_output(gpio, CM_PRESENT, "cm_present", 0)
    d1 = GPIO.request_output(gpio, D1, "d1", 0)
    d2 = GPIO.request_output(gpio, D2, "d2", 0)
    println(Core.stdout, "GPIO configured")

    # Open shared PIO block
    pio = open_pio(0)

    # Initialize 3 chained shift registers via PIO
    chain = open_shift_registers(pio)
    chain[0:23] = false
    println(Core.stdout, "Shift registers initialized ($NUM_REGISTERS x 8-bit, $NBITS outputs)")

    # Initialize nose RGB LED (WS2812) on same PIO block
    nose = open_ws2812(pio, NOSE_RGB)
    set_color!(nose, 0, 0, 0)
    println(Core.stdout, "Nose RGB LED initialized on pin $NOSE_RGB")


    # IMU initialization
    imu = ICM42688PC.ICM42688PC(0,0; speed_hz=UInt32(15_000_000))
    ICM42688PC.soft_reset!(imu)
    good_who_am_i = ICM42688PC.check_who_am_i(imu)
    println(Core.stdout, "IMU who_am_i good? = $(good_who_am_i)")
    good_self_test = ICM42688PC.accel_self_test(imu) && ICM42688PC.gyro_self_test(imu)
    println(Core.stdout, "IMU self test good? = $(good_self_test)")
    if isnothing(options.calibration_data)
        println(Core.stdout, "No saved IMU gains. Keep the bot still and hit enter.")
        readline()
        cod_result, gains = ICM42688PC.calibration_on_demand(imu)
        println(Core.stdout, "IMU CoD success = $(!cod_result.COD_Failed); gains = $gains. Calibrating gyro bias.")
        gx = []; gy = []; gz = []
        ICM42688PC.set_gyro_config!(imu, ICM42688PC.Registers.GYRO_FS_128DPS, ICM42688PC.Registers.GYRO_ODR_896_8HZ)
        ICM42688PC.set_accel_config!(imu, ICM42688PC.Registers.ACCEL_FS_2G, ICM42688PC.Registers.ACCEL_ODR_1000HZ)
        ICM42688PC.set_sensor_config!(imu, true, true, false, false, false)
        sleep(0.1)
        for i=1:100
            m =ICM42688PC.read_all(imu)
            push!(gx, m.gyro_x); push!(gy, m.gyro_y); push!(gz, m.gyro_z)
            sleep(0.01)
        end
        ICM42688PC.set_sensor_config!(imu, false, false, false, false, false)
        offsets = (mean(gx), mean(gy), mean(gz))

        println(Core.stdout, "IMU calibration complete")
        options.calibration_data = gains
        options.gyro_bias = collect(offsets)
        ICM42688PC.set_gyro_bias(imu, offsets)
        to_toml("options.toml", options; include_defaults=true)
    else
        println(Core.stdout, "Loading saved IMU gains.")
        ICM42688PC.load_calibration(imu, options.calibration_data)
        ICM42688PC.set_gyro_bias(imu, (options.gyro_bias...,))
    end 
    ICM42688PC.set_gyro_config!(imu, ICM42688PC.Registers.GYRO_FS_128DPS, ICM42688PC.Registers.GYRO_ODR_896_8HZ)
    ICM42688PC.set_accel_config!(imu, ICM42688PC.Registers.ACCEL_FS_2G, ICM42688PC.Registers.ACCEL_ODR_1000HZ)
    ICM42688PC.set_sensor_config!(imu, true, true, false, false, false)
    println(Core.stdout, "IMU initialized!")

    # PWM/motor initialization
    chip = open_chip()
    motor_a = Motor(export_channel(chip, 0), export_channel(chip, 1))
    motor_b = Motor(export_channel(chip, 2), export_channel(chip, 3))


    hw = HardwareContext(gpio, pio, chain, nose, [cm_present, d1, d2], imu, motor_a, motor_b)

    # Enable hardware
    GPIO.set_value(cm_present, 1)
    GPIO.set_value(d1, 1)
    GPIO.set_value(d2, 0)
    d1v = 1
    d2v = 0

    # 7-segment displays on 2nd and 3rd shift registers
    #disp1 = SevenSeg(chain, 8)   # 2nd register: bits 8–15
    #disp2 = SevenSeg(chain, 16)  # 3rd register: bits 16–23

         
    digit = 0
    hue = 0

    # Control loop timing (500ms = 2Hz)
    loop_period_ns = 50_000_000
    #=
    println("reboot vdd start, disable spi gpio")
    readline()
    for i=1:100
        sleep(0.0001)
        transaction(chain) do
            chain[7]=false
        end
    end
    for i=1:100
        sleep(0.0001)
        transaction(chain) do
            chain[7]=true
        end
    end
    println("reboot vdd done, reenable spi gpio")
    readline()
    =#
    for i=1:100
        sleep(0.0001)
        transaction(chain) do
            chain[3]=false
            chain[7]=true
        end
    end
    println(Core.stdout, "Starting control loop...")
    Base.exit_on_sigint(false)
    loop_start = time_ns()

    try
        while true
            # Wait for next loop iteration
            wait_until(loop_start + loop_period_ns)
            loop_start = time_ns()

            transaction(chain) do
                chain[7]=true
            end
            state = ICM42688PC.read_all(imu)

            #=out = UInt8[0x76,0]
            res = UInt8[0,0]
            nl = transfer!(imu_spi, out, res)
            println(Core.stdout, "IMU $(out) WHO_AM_I: $(res[1]), $(res[2]) nl: $nl")
            sleep(0.01)
            out = UInt8[0xF5,0]
            res = UInt8[0,0]
            nl = transfer!(imu_spi, out, res)
            close(imu_spi)
            println(Core.stdout, "IMU $(out) WHO_AM_I: $(res[1]), $(res[2]) nl: $nl") 
            =#
            # Blink GPIOs
            d1v = 1-d1v
            d2v = 1-d2v
            GPIO.set_value(d1, d1v)
            GPIO.set_value(d2, d2v)

            # Display 1 shows current digit, display 2 shows previous
            #transaction(chain) do
            #    show_digit!(disp2, digit)
            #    show_digit!(disp1, (digit + 1) % 10)
            #end
            digit = (digit + 1) % 10
            set_color!(nose, floor.(Int, 255 .* palette_to_rgb(state.gyro_x/20, state.gyro_y/20))...)
        end
    catch e
        if e isa InterruptException
            println(Core.stdout, "\nCtrl-C received")
        else
            println(Core.stdout, "Error: ", e)
            rethrow()
        end
    finally
        shutdown!(hw)
    end
    return 0
end


@eval Base begin
    using Sockets
    function uv_readcb(handle::Ptr{Cvoid}, nread::Cssize_t, buf::Ptr{Cvoid})
        stream_unknown_type = @handle_as handle Union{TTY, BufferStream, PipeEndpoint, UDPSocket, TCPSocket}
        nrequested = ccall(:jl_uv_buf_len, Csize_t, (Ptr{Cvoid},), buf)
        function readcb_specialized(stream::LibuvStream, nread::Int, nrequested::UInt)
            lock(stream.cond)
            if nread < 0
                if nread == UV_ENOBUFS && nrequested == 0
                    # remind the client that stream.buffer is full
                    notify(stream.cond)
                elseif nread == UV_EOF # libuv called uv_stop_reading already
                    if stream.status != StatusClosing
                        stream.status = StatusEOF
                        notify(stream.cond)
                        if stream isa TTY
                            # stream can still be used by reseteof (or possibly write)
                        elseif !(stream isa PipeEndpoint) && ccall(:uv_is_writable, Cint, (Ptr{Cvoid},), stream.handle) != 0
                            # stream can still be used by write
                        else
                            # underlying stream is no longer useful: begin finalization
                            ccall(:jl_close_uv, Cvoid, (Ptr{Cvoid},), stream.handle)
                            stream.status = StatusClosing
                        end
                    end
                else
                    stream.readerror = _UVError("read", nread)
                    notify(stream.cond)
                    # This is a fatal connection error
                    ccall(:jl_close_uv, Cvoid, (Ptr{Cvoid},), stream.handle)
                    stream.status = StatusClosing
                end
            else
                notify_filled(stream.buffer, nread)
                notify(stream.cond)
            end
            unlock(stream.cond)

            # Stop background reading when
            # 1) there's nobody paying attention to the data we are reading
            # 2) we have accumulated a lot of unread data OR
            # 3) we have an alternate buffer that has reached its limit.
            if stream.status == StatusPaused ||
                (stream.status == StatusActive &&
                ((bytesavailable(stream.buffer) >= stream.throttle) ||
                (bytesavailable(stream.buffer) >= stream.buffer.maxsize)))
                # save cycles by stopping kernel notifications from arriving
                ccall(:uv_read_stop, Cint, (Ptr{Cvoid},), stream)
                stream.status = StatusOpen
            end
            nothing
        end
        readcb_specialized(stream_unknown_type, Int(nread), UInt(nrequested))
        nothing
    end
end

end # module