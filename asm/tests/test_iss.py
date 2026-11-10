"""ISA semantics, assembler and generated-package checks.  Run:  cd asm && python3 -m pytest -q"""

import pytest

from sharada import gen_pkg, isa
from sharada.asm import AsmError, assemble
from sharada.iss import Machine, alu


def run(src, host_in=()):
    m = Machine(assemble(src))
    m.host_in = list(host_in)
    m.spawn(0, 0)
    m.run()
    return m


def test_pkg_is_fresh():
    assert gen_pkg.OUT.read_text() == gen_pkg.render(), "run python3 -m sharada.gen_pkg"


@pytest.mark.parametrize("op,a,b,want", [
    ("ADD", 0xFFFF, 1, 0), ("SUB", 0, 1, 0xFFFF), ("AND", 0xF0F0, 0xFF00, 0xF000),
    ("OR", 0xF000, 0x000F, 0xF00F), ("XOR", 0xFFFF, 0x0F0F, 0xF0F0),
    ("SHL", 1, 15, 0x8000), ("SHL", 1, 16, 0), ("SHR", 0x8000, 15, 1), ("SHR", 0x8000, 31, 0),
    ("PAR", 0b1011, 0xFF, 1), ("PAR", 0b1001, 0xFF, 0), ("PAR", 0b1011, 0b0011, 0),
    ("SLTU", 1, 2, 1), ("SLTU", 2, 2, 0), ("SLTU", 0xFFFF, 0, 0),
])
def test_alu(op, a, b, want):
    assert alu(op, a, b) == want


def test_r0_is_zero_and_loop():
    m = run("""
        mov  r0, 55          ; discarded
        mov  r1, 10
        mov  r2, 0
    loop:
        add  r2, r2, r1
        djnz r1, loop
        iow  CH_HOST, r2     ; 55
        iow  CH_HOST, r0     ; 0
        halt
    """)
    assert m.host_out == [55, 0]


def test_branches_and_calls():
    m = run("""
        mov  r1, 3
        mov  r2, 5
        bltu r1, r2, less
        halt
    less:
        jal  r7, sub
        iow  CH_HOST, r3
        bbs  r3, 2, set
        halt
    set:
        bbc  r3, 0, done      ; bit0 of 4 is clear
        halt
    done:
        iow  CH_HOST, r2
        halt
    sub:
        mov  r3, 4
        jr   r7
    """)
    assert m.host_out == [4, 5]


def test_host_echo_with_parity():
    m = run("""
    top:
        ior  r1, CH_HOST
        par  r2, r1, 0xFF
        shl  r2, r2, 8
        or   r2, r2, r1
        iow  CH_HOST, r2
        jmp  top
    """, host_in=[0x01, 0x03, 0x7F])
    assert m.host_out == [0x101, 0x003, 0x17F]


def test_two_threads_channel():
    words = assemble("""
    producer:
        mov  r1, 4
    p:  iow  CH_A, r1
        djnz r1, p
        halt
    consumer:
        mov  r2, 4
    c:  ior  r1, CH_A
        iow  CH_HOST, r1
        djnz r2, c
        halt
    """)
    m = Machine(words)
    m.spawn(0, 0)
    m.spawn(1, 4)
    m.run()
    assert m.host_out == [4, 3, 2, 1]


def test_encoding_fields():
    w = assemble("add r1, r2, 0x1234")[0]
    d = isa.decode(w)
    assert (d["op"], d["rd"], d["rs"], d["i"], d["imm"]) == ("ADD", 1, 2, 1, 0x1234)
    w = assemble("wait LANE2, E_RXF | E_MISMATCH")[0]
    assert isa.decode(w)["imm"] == (2 << 8) | 0b1100


def test_errors():
    with pytest.raises(AsmError, match="undefined"):
        assemble("jmp nowhere")
    with pytest.raises(AsmError, match="out of range"):
        assemble("ior r1, 300")
    with pytest.raises(AsmError, match="unknown"):
        assemble("frob r1")
