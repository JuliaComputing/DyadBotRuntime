using PIOLib

function ws2812_program(pio::PIOBlock; pin::Integer, sys_clk::Real=200_000_000.0)
    T1 = 2
    T2 = 5
    T3 = 3
    cycles_per_bit = T1 + T2 + T3
    div = clkdiv(800_000.0 * cycles_per_bit; sys_clk)

    prog = build_program([
        WrapTarget(),
        Label(:bitloop),
        Out(RegX(), 1; sideset=0, delay=T3-1),
        Jmp{:not_x}(:do_zero; sideset=1, delay=T1-1),
        Jmp{:always}(:bitloop; sideset=1, delay=T2-1),
        Label(:do_zero),
        Nop(; sideset=0, delay=T2-1),
        Wrap(),
    ]; sideset_bits=1)

    config = SMConfig(pio;
        sideset_pin_base=pin,
        sideset=(1, false, false),
        out_shift=(false, true, 24),
        clkdiv=div,
        wrap=(prog.wrap_target, prog.wrap),
        fifo_join=PIOLib.LibPIO.PIO_FIFO_JOIN_TX,
    )

    prog, config
end

ws2812_put!(sm::StateMachine, grb::UInt32) = put!(sm, grb << 8)

function ws2812_rgb!(sm::StateMachine, r::Integer, g::Integer, b::Integer)
    grb = (UInt32(g) << 16) | (UInt32(r) << 8) | UInt32(b)
    ws2812_put!(sm, grb)
end

struct WS2812
    pio::PIOBlock
    sm::StateMachine
end

"""
    open_ws2812(pio::PIOBlock, pin::Integer) -> WS2812

Initialize a WS2812 LED on `pin` using an SM from the given PIO block.
The PIO block must already be open (shared with other drivers).
"""
function open_ws2812(pio::PIOBlock, pin::Integer)
    prog, config = ws2812_program(pio; pin=pin)

    pio_pin_init!(pio, pin)
    offset = load_program!(pio, prog, config)

    sm = claim_sm(pio)
    try
        set_consecutive_pindirs!(sm, pin, 1, true)
        PIOLib.init!(sm, offset, config)
        enable!(sm)
    catch
        unclaim!(sm)
        rethrow()
    end

    WS2812(pio, sm)
end

set_color!(led::WS2812, r::Integer, g::Integer, b::Integer) = ws2812_rgb!(led.sm, r, g, b)

function Base.close(led::WS2812)
    disable!(led.sm)
    unclaim!(led.sm)
end
