using Timers

include(joinpath(@__DIR__, "gpio.jl"))
include(joinpath(@__DIR__, "pwm.jl"))
include(joinpath(@__DIR__, "mpu6000.jl"))
include(joinpath(@__DIR__, "encoder.jl"))

using .GPIO
using .PWM

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
    ain1::GPIO.GPIOPin
    bin1::GPIO.GPIOPin
    stby::GPIO.GPIOPin
    ltrans_oe::GPIO.GPIOPin
    pwm_chip::PWM.PWMChip
    pwm_left::PWM.PWMChannel
    pwm_right::PWM.PWMChannel
end

"""
    shutdown!(hw::HardwareContext)

Safely shutdown hardware: disable motors and level translator.
Called on Ctrl-C or program exit.
"""
function shutdown!(hw::HardwareContext)
    println(Core.stdout, "Shutting down hardware...")
    # Stop PWM first
    PWM.pwmWrite(hw.pwm_left, 0, PWM_MAX_VALUE)
    PWM.pwmWrite(hw.pwm_right, 0, PWM_MAX_VALUE)
    # Put motor controller into standby
    GPIO.set_value(hw.stby, 0)
    # Disable level translator
    GPIO.set_value(hw.ltrans_oe, 0)
    println(Core.stdout, "Hardware shutdown complete.")
end

function handle_err(ec)
    if ec == 0
        return
    end
    println(Core.stdout, "Had error ", ec)
    return
end


# =============================================================================
# Motor Output Functions
# =============================================================================

"""
    car_stop!(hw::HardwareContext)

Stop both motors by setting PWM to 0 and appropriate direction pins.
"""
function car_stop!(hw::HardwareContext)
    GPIO.set_value(hw.ain1, 1)  # HIGH
    GPIO.set_value(hw.bin1, 0)  # LOW
    GPIO.set_value(hw.stby, 1)  # HIGH (standby off = enabled, but PWM=0)
    PWM.pwmWrite(hw.pwm_left, 0, PWM_MAX_VALUE)
    PWM.pwmWrite(hw.pwm_right, 0, PWM_MAX_VALUE)
end

"""
    apply_motor_output!(hw::HardwareContext, pwm_left, pwm_right)

Apply PWM signals to motors based on computed control values.
Handles direction pin setting based on PWM sign.
"""
function apply_motor_output!(hw::HardwareContext, pwm_left::Float32, pwm_right::Float32)
    # Left motor
    if pwm_left < 0
        GPIO.set_value(hw.ain1, 1)
        PWM.pwmWrite(hw.pwm_left, round(Int, -pwm_left), PWM_MAX_VALUE)
    else
        GPIO.set_value(hw.ain1, 0)
        PWM.pwmWrite(hw.pwm_left, round(Int, pwm_left), PWM_MAX_VALUE)
    end

    # Right motor
    if pwm_right < 0
        GPIO.set_value(hw.bin1, 1)
        PWM.pwmWrite(hw.pwm_right, round(Int, -pwm_right), PWM_MAX_VALUE)
    else
        GPIO.set_value(hw.bin1, 0)
        PWM.pwmWrite(hw.pwm_right, round(Int, pwm_right), PWM_MAX_VALUE)
    end
end

function (@main)(args)::Cint
    println(Core.stdout, "hello world!")

    # Initialize GPIO controller
    gpio = GPIO.open_gpio("/dev/gpiochip0")
    println(Core.stdout, "gpio opened")

    # Setup GPIO output pins for motor direction
    ain1 = GPIO.request_output(gpio, AIN1, "motor_ain1", 0)
    println(Core.stdout, "ain1 configured")

    bin1 = GPIO.request_output(gpio, BIN1, "motor_bin1", 0)
    println(Core.stdout, "bin1 configured")

    stby = GPIO.request_output(gpio, STBY_PIN, "motor_stby", 0)
    println(Core.stdout, "stby configured")

    ltrans_oe = GPIO.request_output(gpio, LTRANS_OE, "ltrans_oe", 0)
    println(Core.stdout, "ltrans_oe configured")

    println(Core.stdout, "gpio pins configured")

    # Setup hardware PWM via sysfs
    pwm_chip = PWM.open_chip(0)

    pwm_left = PWM.export_channel_for_gpio(pwm_chip, PWMA_LEFT)
    PWM.set_period_hz(pwm_left, PWM_FREQ_HZ)
    PWM.set_duty_cycle_ns(pwm_left, 0)
    PWM.enable(pwm_left)

    pwm_right = PWM.export_channel_for_gpio(pwm_chip, PWMB_RIGHT)
    PWM.set_period_hz(pwm_right, PWM_FREQ_HZ)
    PWM.set_duty_cycle_ns(pwm_right, 0)
    PWM.enable(pwm_right)

    println(Core.stdout, "pwm startup done")

    # Create hardware context
    hw = HardwareContext(gpio, ain1, bin1, stby, ltrans_oe, pwm_chip, pwm_left, pwm_right)

    # Initialize I2C for IMU (bus 1, address 0x68)
    imu = MPU6000(1, 0x68)
    println(Core.stdout, "i2c startup done")

    wake!(imu)
    println(Core.stdout, "imu startup done")

    timu = ThreadedMPU6000(imu)
    tenc_1a = ThreadedEncoder(Encoder(gpio, M1A))
    tenc_2a = ThreadedEncoder(Encoder(gpio, M2A))

    GPIO.set_value(ltrans_oe, 1)
    GPIO.set_value(stby, 1) # take the motor controller out of standby
    ctrl = BalanceController.BalanceController()

    ml_update_rate_ns = 50000000
    imu_read_margin = 12600000
    println(Core.stdout, "start loop!")
    start = time_ns()
    Base.exit_on_sigint(false)
    try
        while true
            wait_until(start + ml_update_rate_ns - imu_read_margin)
            put!(timu.request, true) # trigger the IMU request
            wait_until(start + ml_update_rate_ns)
            imu_data = take!(timu.response)
            println(Core.stdout, "IMU response: $(imu_data.accel_x)")
            enc_1_cnts = reset!(tenc_1a)
            enc_2_cnts = reset!(tenc_2a)
            command = BalanceController.balance_car!(ctrl, enc_1_cnts, enc_2_cnts,
                        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                        imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z)
            start = time_ns()
            if isnothing(command)
                car_stop!(hw)
                println(Core.stdout, "motors stopped!")
            else
                left, right = command
                apply_motor_output!(hw, left, right)
                println(Core.stdout, "motors run!")
            end
        end
    catch e
        rethrow()
    finally
        GPIO.set_value(ltrans_oe, 0)
        GPIO.set_value(stby, 0)
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
