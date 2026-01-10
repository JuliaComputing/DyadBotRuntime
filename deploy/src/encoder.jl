include(joinpath(@__DIR__, "gpio.jl"))
using .GPIO

#=============================================================================
  Encoder Struct and Basic Operations
=============================================================================#

mutable struct Encoder
    line::GPIO.GPIOLine
    count::Int
    state::Bool
end

function Encoder(chip::GPIO.GPIOChip, pin::Int)
    line = GPIO.get_line(chip, pin)
    GPIO.request_input(line, "encoder")
    initial_state = GPIO.get_value(line) == 1
    return Encoder(line, 0, initial_state)
end

function step!(e::Encoder)
    new_state = GPIO.get_value(e.line)
    if e.state && new_state == 0
        e.count += 1
        e.state = false
    elseif !e.state && new_state == 1
        e.state = true
    end
end

function reset!(e::Encoder)
    e.count = 0
end

function close!(e::Encoder)
    GPIO.release_line(e.line)
end

#=============================================================================
  Threaded Encoder Wrapper
=============================================================================#

@enum EncoderCommand::UInt8 CMD_READ CMD_RESET CMD_STOP

"""
    ThreadedEncoder

A thread-safe wrapper around Encoder that polls at a specified interval.
Send commands via the command channel, receive count via the response channel.

# Example
```julia
chip = GPIO.open_chip()
enc = Encoder(chip, pin)
tenc = ThreadedEncoder(enc, 1_000_000)  # 1ms poll interval

# Read current count
put!(tenc.command, CMD_READ)
count = take!(tenc.response)

# Reset count (returns old count)
put!(tenc.command, CMD_RESET)
old_count = take!(tenc.response)

stop!(tenc)
```
"""
mutable struct ThreadedEncoder
    command::Channel{EncoderCommand}
    response::Channel{Int}
    worker::Task
    running::Threads.Atomic{Bool}
end

"""
    ThreadedEncoder(enc::Encoder, poll_interval_ns::Int)

Create a ThreadedEncoder that polls the encoder at the specified interval (in nanoseconds).
The worker thread continuously polls the encoder and responds to commands.
"""
function ThreadedEncoder(enc::Encoder, poll_interval_ns::Int)
    command = Channel{EncoderCommand}(1)
    response = Channel{Int}(1)
    running = Threads.Atomic{Bool}(true)

    worker = Threads.@spawn begin
        next_poll = time_ns()
        while running[]
            # Poll encoder
            now = time_ns()
            if now >= next_poll
                step!(enc)
                next_poll = now + poll_interval_ns
            end

            # Check for commands (non-blocking)
            if isready(command)
                cmd = take!(command)
                if cmd == CMD_READ
                    put!(response, enc.count)
                elseif cmd == CMD_RESET
                    old_count = enc.count
                    reset!(enc)
                    put!(response, old_count)
                elseif cmd == CMD_STOP
                    running[] = false
                    break
                end
            end
        end
    end

    return ThreadedEncoder(command, response, worker, running)
end

"""
    read(tenc::ThreadedEncoder) -> Int

Read the current encoder count.
"""
function Base.read(tenc::ThreadedEncoder)
    put!(tenc.command, CMD_READ)
    return take!(tenc.response)
end

"""
    reset!(tenc::ThreadedEncoder) -> Int

Reset the encoder count and return the old value.
"""
function reset!(tenc::ThreadedEncoder)
    put!(tenc.command, CMD_RESET)
    return take!(tenc.response)
end

"""
    stop!(tenc::ThreadedEncoder)

Stop the worker thread.
"""
function stop!(tenc::ThreadedEncoder)
    if tenc.running[]
        put!(tenc.command, CMD_STOP)
        wait(tenc.worker)
    end
    close(tenc.command)
    close(tenc.response)
    return nothing
end
