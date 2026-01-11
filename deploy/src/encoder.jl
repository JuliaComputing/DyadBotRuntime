# Uses GPIO module (must be included before this file)
using .GPIO

#=============================================================================
  Encoder Struct and Basic Operations
=============================================================================#

mutable struct Encoder
    pin::GPIO.GPIOPin
    count::Int
end

"""
    Encoder(gpio::GPIOController, pin_num::Int; edge::EdgeType=EDGE_FALLING)

Create an encoder that counts edges on the specified pin.
Uses interrupt-based edge detection (no polling).
"""
function Encoder(gpio::GPIO.GPIOController, pin_num::Int; edge::GPIO.EdgeType=GPIO.EDGE_FALLING)
    pin = GPIO.request_input_edge(gpio, pin_num, "encoder", edge)
    return Encoder(pin, 0)
end

"""
    wait_edge!(e::Encoder, timeout_ms::Int=-1) -> Union{GPIO.GPIOEvent, Nothing}

Wait for an edge event. Returns the event or nothing on timeout.
Increments the encoder count on each edge.
"""
function wait_edge!(e::Encoder, timeout_ms::Int=-1)
    if GPIO.poll_event(e.pin, timeout_ms)
        event = GPIO.read_event(e.pin)
        e.count += 1
        return event
    end
    return nothing
end

"""
    drain_events(pin::GPIO.GPIOPin) -> Int

Read all pending events from a pin without blocking. Returns number of events drained.
"""
function drain_events(pin::GPIO.GPIOPin)
    n = 0
    while GPIO.poll_event(pin, 0)
        GPIO.read_event(pin)
        n += 1
    end
    return n
end

"""
    drain_events!(e::Encoder) -> Int

Read all pending events without blocking. Returns number of events drained.
"""
function drain_events!(e::Encoder)
    n = drain_events(e.pin)
    e.count += n
    return n
end

function reset!(e::Encoder)
    e.count = 0
end

function close!(e::Encoder)
    close(e.pin)
end

#=============================================================================
  Threaded Encoder Wrapper
=============================================================================#

@enum EncoderCommand::UInt8 CMD_READ CMD_RESET CMD_STOP

"""
    ThreadedEncoder

A thread-safe wrapper around Encoder using two dedicated threads:
- Event thread: continuously reads edge events and atomically updates count
- Command thread: handles read/reset commands with immediate response

# Example
```julia
chip = GPIO.open_gpio()
enc = Encoder(chip, pin)
tenc = ThreadedEncoder(enc)

# Read current count (immediate response)
count = read(tenc)

# Reset count (returns old count)
old_count = reset!(tenc)

stop!(tenc)
```
"""
mutable struct ThreadedEncoder
    count::Threads.Atomic{Int}
    command::Channel{EncoderCommand}
    response::Channel{Int}
    event_worker::Task
    command_worker::Task
    running::Threads.Atomic{Bool}
end

"""
    ThreadedEncoder(enc::Encoder)

Create a ThreadedEncoder with separate event and command threads.
"""
function ThreadedEncoder(enc::Encoder)
    count = Threads.Atomic{Int}(enc.count)
    command = Channel{EncoderCommand}(1)
    response = Channel{Int}(1)
    running = Threads.Atomic{Bool}(true)

    # Event thread: continuously drain all pending events
    event_worker = Threads.@spawn begin
        while running[]
            # Wait for at least one event (with timeout to check running flag)
            if GPIO.poll_event(enc.pin, 100)
                n = drain_events(enc.pin)
                Threads.atomic_add!(count, n)
            end
        end
    end

    # Command thread: handle commands with immediate response
    command_worker = Threads.@spawn begin
        while running[]
            cmd = try
                take!(command)
            catch e
                e isa InvalidStateException && break
                rethrow()
            end

            if cmd == CMD_READ
                put!(response, count[])
            elseif cmd == CMD_RESET
                old_count = Threads.atomic_xchg!(count, 0)
                put!(response, old_count)
            elseif cmd == CMD_STOP
                running[] = false
                break
            end
        end
    end

    return ThreadedEncoder(count, command, response, event_worker, command_worker, running)
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

Stop both worker threads.
"""
function stop!(tenc::ThreadedEncoder)
    if tenc.running[]
        tenc.running[] = false
        close(tenc.command)  # Unblocks command worker
        wait(tenc.event_worker)
        wait(tenc.command_worker)
    end
    close(tenc.response)
    return nothing
end
