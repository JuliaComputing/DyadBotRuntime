"""
    SevenSeg

Driver for a common-anode 7-segment + DP display (e.g. SLS0391FB4A1GD)
connected through a ShiftRegisterChain.

The display uses 8 consecutive shift register outputs starting at `start`:
    start+0 = A  (top)
    start+1 = B  (upper right)
    start+2 = C  (lower right)
    start+3 = D  (bottom)
    start+4 = E  (lower left)
    start+5 = F  (upper left)
    start+6 = G  (middle)
    start+7 = DP (decimal point)

Common anode is wired to VCC — segments light when the shift register
output is LOW, so all patterns are inverted before writing.
"""
struct SevenSeg
    chain::ShiftRegisterChain
    start::Int  # 0-based index of segment A in the chain
end

#        A  B  C  D  E  F  G
# bit    0  1  2  3  4  5  6
const SEG_A  = 0x01
const SEG_B  = 0x02
const SEG_C  = 0x04
const SEG_D  = 0x08
const SEG_E  = 0x10
const SEG_F  = 0x20
const SEG_G  = 0x40
const SEG_DP = 0x80

# Digit encodings: which segments are ON (active-high logic, inverted at write time)
const DIGIT_PATTERNS = UInt8[
    #                         GFEDCBA
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,         # 0
    SEG_B|SEG_C,                                    # 1
    SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,                 # 2
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,                 # 3
    SEG_B|SEG_C|SEG_F|SEG_G,                        # 4
    SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,                 # 5
    SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,           # 6
    SEG_A|SEG_B|SEG_C,                              # 7
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,    # 8
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G,           # 9
    SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G,           # A (10)
    SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,                 # b (11)
    SEG_A|SEG_D|SEG_E|SEG_F,                        # C (12)
    SEG_B|SEG_C|SEG_D|SEG_E|SEG_G,                 # d (13)
    SEG_A|SEG_D|SEG_E|SEG_F|SEG_G,                 # E (14)
    SEG_A|SEG_E|SEG_F|SEG_G,                        # F (15)
]

"""
    show_digit!(seg, digit::Integer; dp=false)

Display a hex digit (0x0–0xF). Set `dp=true` to light the decimal point.
"""
function show_digit!(seg::SevenSeg, digit::Integer; dp::Bool=false)
    0 <= digit <= 15 || error("digit must be 0-15, got $digit")
    pattern = DIGIT_PATTERNS[digit + 1]
    if dp
        pattern |= SEG_DP
    end
    _write_pattern!(seg, pattern)
end

"""
    show_raw!(seg, pattern::UInt8)

Write a raw segment pattern (active-high: bit 0=A … bit 7=DP).
The driver inverts for common anode automatically.
"""
function show_raw!(seg::SevenSeg, pattern::UInt8)
    _write_pattern!(seg, pattern)
end

"""
    clear!(seg)

Turn off all segments (including DP).
"""
clear!(seg::SevenSeg) = _write_pattern!(seg, 0x00)

"""
    set_dp!(seg, on::Bool)

Set only the decimal point without changing the digit segments.
"""
function set_dp!(seg::SevenSeg, on::Bool)
    seg.chain[seg.start + 7] = !on  # inverted: LOW = lit
end

function _write_pattern!(seg::SevenSeg, pattern::UInt8)
    # Common anode: invert — LOW lights the segment
    inverted = xor(pattern, 0xFF)
    seg.chain[seg.start:seg.start+7] = inverted
end
