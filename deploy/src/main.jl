using PiGPIO
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
function car_stop!(p::Pi)
    write(p, AIN1, 1)     # HIGH
    write(p, BIN1, 0)     # LOW
    write(p, STBY_PIN, 1) # HIGH (standby off = enabled, but PWM=0)
    PiGPIO.set_PWM_dutycycle(p, PWMA_LEFT, 0)
    PiGPIO.set_PWM_dutycycle(p, PWMB_RIGHT, 0)
end

"""
    apply_motor_output!(pwm_left, pwm_right)

Apply PWM signals to motors based on computed control values.
Handles direction pin setting based on PWM sign.
"""
function apply_motor_output!(p::Pi, pwm_left::Float32, pwm_right::Float32)
    # Left motor
    if pwm_left < 0
        write(p, AIN1, 1)
        PiGPIO.set_PWM_dutycycle(p, PWMA_LEFT, round(Int, -pwm_left))
    else
        write(p, AIN1, 0)
        PiGPIO.set_PWM_dutycycle(p, PWMA_LEFT, round(Int, pwm_left))
    end

    # Right motor
    if pwm_right < 0
        write(p, BIN1, 1)
        PiGPIO.set_PWM_dutycycle(p, PWMB_RIGHT, round(Int, -pwm_right))
    else
        write(p, BIN1, 0)
        PiGPIO.set_PWM_dutycycle(p, PWMB_RIGHT, round(Int, pwm_right))
    end
end

function (@main)(args)::Cint
    println(Core.stdout, "hello world!")
    p = Pi()
    println(Core.stdout, "pi startup done")
    PiGPIO.set_internals(p, 8, 0)
    println(Core.stdout, "set debug level")
    handle_err(PiGPIO.hardware_PWM(p, PWMA_LEFT, 1000, 0))
    handle_err(PiGPIO.hardware_PWM(p, PWMB_RIGHT, 1000, 0)) # use 1kHz pwm for the motor drivers
    println(Core.stdout, "pwm startup done")
    imu_bus = PiGPIO.i2c_open(p, 1, 0x68)
    println(Core.stdout, "i2c startup done")
    imu = MPU6000(p, imu_bus, GyroRange.GYRO_FS_250, AccelRange.ACCEL_F_2G)
    wake!(imu)
    println(Core.stdout, "imu startup done")
    timu = ThreadedMPU6000(imu)
    tenc_1a = ThreadedEncoder(Encoder(p, M1A), 1_000)
    tenc_2a = ThreadedEncoder(Encoder(p, M2A), 1_000)

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
            car_stop!(p)
        else 
            left, right = command
            apply_motor_output!(p, left, right)
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