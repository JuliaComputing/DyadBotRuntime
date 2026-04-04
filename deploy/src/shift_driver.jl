using PIOLib

# --- Pin assignments ---
const SER_PIN  = 26  # Serial data  (SER/DS)
const CLK_PIN  = 19  # Shift clock  (SRCLK/SHCP)
const RCLK_PIN = 6   # Latch/output clock (RCLK/STCP)

# 3 chained 74HC595 shift registers = 24 bits per transfer
const NUM_REGISTERS = 3
const NBITS = NUM_REGISTERS * 8  # 24

# Clock divider: 125 MHz / 62.5 = 2 MHz PIO clock → 1 MHz shift rate
const CLKDIV = 62.5f0

"""
    ShiftRegisterChain

Driver for 3 daisy-chained 74HC595 shift registers controlled via PIO.
Each register provides 8 output pins (Q0-Q7), for 24 outputs total.
Maintains a shadow register so pin state can be read back and modified
individually without re-specifying the full output word.

Data layout in a 24-bit word:
- Bits  0–7:  Register 3 (end of chain, closest to SER_PIN)
- Bits  8–15: Register 2 (middle)
- Bits 16–23: Register 1 (start of chain, farthest from SER_PIN)

# Indexing (0-based pin numbers)
    chain[5]              # read pin 5 → Bool
    chain[5] = true       # set pin 5, flush immediately
    chain[[0,3,15]] = true                  # set multiple pins to same value
    chain[[0,3,15]] = [true, false, true]   # set multiple pins to different values
    chain[0:7] = 0xFF                       # set range from byte
    chain[0:7] = false                      # clear range

# Transactions (single flush for multiple operations)
    transaction(chain) do
        chain[0] = sensor_val
        chain[1] = chain[0]
    end
"""
mutable struct ShiftRegisterChain
    pio::PIOBlock
    sm::StateMachine
    state::UInt32
    hold::Bool
end

"""
    open_shift_registers(pio_idx=0) -> ShiftRegisterChain

Initialize the PIO and state machine for driving the shift register chain.
Caller is responsible for calling `close` when done.
"""
function open_shift_registers(pio_idx::Integer=0)
    prog, config = shift_register_program(
        ser_pin=SER_PIN, clk_pin=CLK_PIN, rclk_pin=RCLK_PIN,
        nbits=NBITS, clkdiv=CLKDIV,
    )

    pio = open_pio(pio_idx)
    try
        for pin in (SER_PIN, CLK_PIN, RCLK_PIN)
            pio_pin_init!(pio, pin)
        end

        load_program!(pio, prog, config)

        sm = claim_sm(pio)
        try
            pin_mask = UInt32(1) << SER_PIN | UInt32(1) << CLK_PIN | UInt32(1) << RCLK_PIN
            set_pindirs!(sm, pin_mask, pin_mask)
            init!(sm, 0, config)
            setup_shift_register!(sm, NBITS)
            enable!(sm)
        catch
            unclaim!(sm)
            rethrow()
        end

        return ShiftRegisterChain(pio, sm, UInt32(0), false)
    catch
        close(pio)
        rethrow()
    end
end

# --- Flush ---

function flush!(chain::ShiftRegisterChain)
    shift_out!(chain.sm, chain.state & 0x00FFFFFF)
end

# --- Bulk writes (update shadow + flush) ---

function write_outputs!(chain::ShiftRegisterChain, data::UInt32)
    chain.state = data & 0x00FFFFFF
    chain.hold || flush!(chain)
end

function write_registers!(chain::ShiftRegisterChain, reg1::UInt8, reg2::UInt8, reg3::UInt8)
    chain.state = (UInt32(reg1) << 16) | (UInt32(reg2) << 8) | UInt32(reg3)
    chain.hold || flush!(chain)
end

# --- Transaction ---

function transaction(f, chain::ShiftRegisterChain)
    was_held = chain.hold
    chain.hold = true
    try
        f()
    finally
        chain.hold = was_held
        was_held || flush!(chain)
    end
end

# --- Single pin: chain[i] ---

function _check_pin(i::Integer)
    0 <= i <= 23 || throw(BoundsError("pin index must be 0-23, got $i"))
end

function Base.getindex(chain::ShiftRegisterChain, i::Integer)
    _check_pin(i)
    (chain.state >> i) & 1 == 1
end

function Base.setindex!(chain::ShiftRegisterChain, val::Bool, i::Integer)
    _check_pin(i)
    if val
        chain.state |= UInt32(1) << i
    else
        chain.state &= ~(UInt32(1) << i)
    end
    chain.hold || flush!(chain)
    val
end

# --- Multiple pins: chain[[0, 3, 15]] = true / [true, false, true] ---

function Base.getindex(chain::ShiftRegisterChain, idxs::AbstractVector{<:Integer})
    [chain[i] for i in idxs]
end

function Base.setindex!(chain::ShiftRegisterChain, val::Bool, idxs::AbstractVector{<:Integer})
    for i in idxs
        _check_pin(i)
        if val
            chain.state |= UInt32(1) << i
        else
            chain.state &= ~(UInt32(1) << i)
        end
    end
    chain.hold || flush!(chain)
    val
end

function Base.setindex!(chain::ShiftRegisterChain, vals::AbstractVector{Bool}, idxs::AbstractVector{<:Integer})
    length(vals) == length(idxs) || throw(DimensionMismatch("$(length(vals)) values for $(length(idxs)) pins"))
    for (i, v) in zip(idxs, vals)
        _check_pin(i)
        if v
            chain.state |= UInt32(1) << i
        else
            chain.state &= ~(UInt32(1) << i)
        end
    end
    chain.hold || flush!(chain)
    vals
end

# --- Range: chain[0:7] = 0xFF / true / false ---

function Base.getindex(chain::ShiftRegisterChain, r::UnitRange{<:Integer})
    [chain[i] for i in r]
end

function Base.setindex!(chain::ShiftRegisterChain, val::Bool, r::UnitRange{<:Integer})
    for i in r
        _check_pin(i)
        if val
            chain.state |= UInt32(1) << i
        else
            chain.state &= ~(UInt32(1) << i)
        end
    end
    chain.hold || flush!(chain)
    val
end

function Base.setindex!(chain::ShiftRegisterChain, byte::Integer, r::UnitRange{<:Integer})
    nbits = length(r)
    1 <= nbits <= 24 || throw(ArgumentError("range too wide"))
    for (bit, i) in enumerate(r)
        _check_pin(i)
        if (byte >> (bit - 1)) & 1 == 1
            chain.state |= UInt32(1) << i
        else
            chain.state &= ~(UInt32(1) << i)
        end
    end
    chain.hold || flush!(chain)
    byte
end

function Base.setindex!(chain::ShiftRegisterChain, vals::AbstractVector{Bool}, r::UnitRange{<:Integer})
    length(vals) == length(r) || throw(DimensionMismatch("$(length(vals)) values for $(length(r)) pins"))
    for (v, i) in zip(vals, r)
        _check_pin(i)
        if v
            chain.state |= UInt32(1) << i
        else
            chain.state &= ~(UInt32(1) << i)
        end
    end
    chain.hold || flush!(chain)
    vals
end

# --- Cleanup ---

function Base.close(chain::ShiftRegisterChain)
    disable!(chain.sm)
    unclaim!(chain.sm)
    close(chain.pio)
end
