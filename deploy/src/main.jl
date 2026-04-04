using Timers

include(joinpath(@__DIR__, "gpio.jl"))
include(joinpath(@__DIR__, "pwm.jl"))
include(joinpath(@__DIR__, "mpu6000.jl"))
include(joinpath(@__DIR__, "encoder.jl"))
include(joinpath(@__DIR__, "shift_driver.jl"))
include(joinpath(@__DIR__, "seven_seg.jl"))

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
    chain::ShiftRegisterChain
end

"""
    shutdown!(hw::HardwareContext)

Safely shutdown hardware: clear shift register outputs, disable motors and level translator.
Called on Ctrl-C or program exit.
"""
function shutdown!(hw::HardwareContext)
    println(Core.stdout, "Shutting down hardware...")
    hw.chain[0:23] = false
    close(hw.chain)
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

function (@main)(args)::Cint
    println(Core.stdout, "Balance car starting...")

    # Initialize GPIO controller
    gpio = GPIO.open_gpio("/dev/gpiochip0")

    # Setup GPIO output pins for motor direction
    cm_present = GPIO.request_output(gpio, CM_PRESENT, "cm_present", 0)
    d1 = GPIO.request_output(gpio, D1, "d1", 0)
    d2 = GPIO.request_output(gpio, D2, "d2", 0)
    println(Core.stdout, "GPIO configured")

    # Initialize 3 chained shift registers via PIO
    chain = open_shift_registers()
    chain[0:23] = false
    println(Core.stdout, "Shift registers initialized ($NUM_REGISTERS x 8-bit, $NBITS outputs)")

    hw = HardwareContext(gpio, chain)

    # Enable hardware
    GPIO.set_value(cm_present, 1)
    GPIO.set_value(d1, 1)
    GPIO.set_value(d2, 0)
    d1v = 1
    d2v = 0

    # 7-segment displays on 2nd and 3rd shift registers
    disp1 = SevenSeg(chain, 8)   # 2nd register: bits 8–15
    disp2 = SevenSeg(chain, 16)  # 3rd register: bits 16–23

    digit = 0

    # Control loop timing (500ms = 2Hz)
    loop_period_ns = 500_000_000

    println(Core.stdout, "Starting control loop...")
    Base.exit_on_sigint(false)
    loop_start = time_ns()

    try
        while true
            # Wait for next loop iteration
            wait_until(loop_start + loop_period_ns)
            loop_start = time_ns()

            # Blink GPIOs
            d1v = 1-d1v
            d2v = 1-d2v
            GPIO.set_value(d1, d1v)
            GPIO.set_value(d2, d2v)

            # Display 1 shows current digit, display 2 shows previous
            transaction(chain) do
                show_digit!(disp2, digit)
                show_digit!(disp1, (digit + 1) % 10)
            end
            digit = (digit + 1) % 10
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
