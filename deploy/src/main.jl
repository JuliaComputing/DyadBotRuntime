using WiringPi
using Timers
include(joinpath(@__DIR__, "mpu6000.jl"))
include(joinpath(@__DIR__, "encoder.jl"))

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
const STBY_PIN = 0

const PWM_RANGE = 1024  # Hardware PWM duty cycle range (0-1024)

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
    car_stop!()

Stop both motors by setting PWM to 0 and appropriate direction pins.
"""
function car_stop!()
    digitalWrite(AIN1, HIGH)
    digitalWrite(BIN1, LOW)
    digitalWrite(STBY_PIN, HIGH) # standby off = enabled, but PWM=0
    pwmWrite(PWMA_LEFT, 0)
    pwmWrite(PWMB_RIGHT, 0)
end

"""
    apply_motor_output!(pwm_left, pwm_right)

Apply PWM signals to motors based on computed control values.
Handles direction pin setting based on PWM sign.
"""
function apply_motor_output!(pwm_left::Float32, pwm_right::Float32)
    # Left motor
    if pwm_left < 0
        digitalWrite(AIN1, HIGH)
        pwmWrite(PWMA_LEFT, round(Int, -pwm_left))
    else
        digitalWrite(AIN1, LOW)
        pwmWrite(PWMA_LEFT, round(Int, pwm_left))
    end

    # Right motor
    if pwm_right < 0
        digitalWrite(BIN1, HIGH)
        pwmWrite(PWMB_RIGHT, round(Int, -pwm_right))
    else
        digitalWrite(BIN1, LOW)
        pwmWrite(PWMB_RIGHT, round(Int, pwm_right))
    end
end

function (@main)(args)::Cint
    println(Core.stdout, "hello world!")

    # Initialize WiringPi with BCM GPIO numbering
    wiringPiSetupGpio()
    println(Core.stdout, "wiringPi startup done")

    # Setup GPIO pins as outputs
    pinMode(AIN1, OUTPUT)
    pinMode(BIN1, OUTPUT)
    pinMode(STBY_PIN, OUTPUT)
    pinMode(M1A, INPUT)
    pinMode(M2A, INPUT)

    # Setup hardware PWM on motor pins
    pinMode(PWMA_LEFT, PWM_OUTPUT)
    pinMode(PWMB_RIGHT, PWM_OUTPUT)
    pwmSetMode(PWM_MODE_MS)  # Mark-space mode for motor control
    pwmSetRange(PWM_RANGE)
    pwmSetClock(19)  # ~1kHz PWM frequency (19.2MHz / 19 / 1024 ≈ 1kHz)
    println(Core.stdout, "pwm startup done")

    # Initialize I2C for IMU (address 0x68)
    imu_handle = wiringPiI2CSetup(0x68)
    if imu_handle < 0
        println(Core.stdout, "Failed to initialize I2C")
        return 1
    end
    println(Core.stdout, "i2c startup done")

    imu = MPU6000(imu_handle, GyroRange.GYRO_FS_250, AccelRange.ACCEL_F_2G)
    wake!(imu)
    println(Core.stdout, "imu startup done")
    timu = ThreadedMPU6000(imu)
    tenc_1a = ThreadedEncoder(Encoder(M1A), 1_000)
    tenc_2a = ThreadedEncoder(Encoder(M2A), 1_000)

    ctrl = BalanceController()

    ml_update_rate_ns = 50000000
    imu_read_margin = 12600000
    println(Core.stdout, "start loop!")
    now = time_ns()
    while true
        wait_until(start + ml_update_rate_ns - imu_read_margin)
        put!(timu.request, true) # trigger the IMU request
        wait_until(start + ml_update_rate_ns)
        imu_data = take!(timu.response)
        enc_1_cnts = reset!(tenc_1a)
        enc_2_cnts = reset!(tenc_2a)
        command = balance_car!(ctrl, enc_1_cnts, enc_2_cnts,
                      imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                      imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z)
        now = time_ns()
        if isnothing(command)
            car_stop!()
        else
            left, right = command
            apply_motor_output!(left, right)
        end
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
