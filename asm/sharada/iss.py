"""
Architectural golden model (ISS) for Sharada.

This models *what* each instruction does, not *when*. Timing (the barrel
schedule, lane bit clocks) belongs to the RTL. The two meet in lock-step
co-simulation: every time the RTL commits an instruction for thread t, the
ISS steps thread t once and the two results must agree. Values that come
from the outside world (IO reads, WAIT outcomes) are taken from the RTL's
commit record, the same trick RISC-V co-simulators use for MMIO.

Used three ways:
  * unit tests of the ISA semantics (asm/tests/test_iss.py)
  * stand-alone firmware runs with a simple IO model (Machine.run)
  * the lock-step checker in test/cosim.py
"""

from dataclasses import dataclass, field

from . import isa

MASK = (1 << isa.XLEN) - 1
PCMASK = isa.IMEM_WORDS - 1


@dataclass
class Thread:
    tid: int
    pc: int = 0
    regs: list = field(default_factory=lambda: [0] * isa.NREGS)
    running: bool = False
    sleep: int = 0

    def r(self, n):
        return 0 if n == 0 else self.regs[n]


@dataclass
class Commit:
    """What one retired instruction did. Compared field by field in co-sim."""
    tid: int
    pc: int
    word: int
    next_pc: int
    rd: int = 0             # 0 means no register write
    wdata: int = 0


class IllegalInstruction(Exception):
    pass


def alu(op, a, b):
    if op == "ADD":
        return (a + b) & MASK
    if op == "SUB":
        return (a - b) & MASK
    if op == "AND":
        return a & b
    if op == "OR":
        return a | b
    if op == "XOR":
        return a ^ b
    if op == "SHL":
        sh = b & 0x1F
        return (a << sh) & MASK if sh < 16 else 0
    if op == "SHR":
        sh = b & 0x1F
        return a >> sh if sh < 16 else 0
    if op == "PAR":
        return bin(a & b).count("1") & 1
    if op == "SLTU":
        return int(a < b)
    raise IllegalInstruction(op)


def step(t: Thread, word: int, io_read=None, io_write=None) -> Commit:
    """
    Execute one instruction for thread t.

    io_read(addr) -> int and io_write(addr, value) are the IO model. In
    co-simulation io_read returns the value the RTL observed. WAIT is
    treated as already satisfied: the RTL only commits a WAIT once its
    condition holds, and a stand-alone run models blocking in Machine.
    """
    d = isa.decode(word)
    op, rd, rs, rt, i, imm = d["op"], d["rd"], d["rs"], d["rt"], d["i"], d["imm"]
    a = t.r(rs)
    b = imm if i else t.r(rt)
    target = imm & PCMASK
    c = Commit(tid=t.tid, pc=t.pc, word=word, next_pc=(t.pc + 1) & PCMASK)

    def write(reg, val):
        if reg != 0:
            t.regs[reg] = val & MASK
            c.rd, c.wdata = reg, val & MASK

    if op == "NOP":
        pass
    elif op in isa.ALU_OPS:
        write(rd, alu(op, a, b))
    elif op == "JMP":
        c.next_pc = target
    elif op in isa.REG_BRANCHES:
        bv = t.r(rt)
        taken = {"BEQ": a == bv, "BNE": a != bv, "BLTU": a < bv, "BGEU": a >= bv}[op]
        if taken:
            c.next_pc = target
    elif op in isa.BIT_BRANCHES:
        bit = (a >> ((imm >> 12) & 0xF)) & 1
        if bit == (1 if op == "BBS" else 0):
            c.next_pc = target
    elif op == "DJNZ":
        v = (a - 1) & MASK
        write(rd, v)
        if v != 0:
            c.next_pc = target
    elif op == "JAL":
        write(rd, (t.pc + 1) & PCMASK)
        c.next_pc = target
    elif op == "JR":
        c.next_pc = a & PCMASK
    elif op == "IOR":
        write(rd, io_read(imm & 0xFF) if io_read else 0)
    elif op == "IOW":
        if io_write:
            io_write(imm & 0xFF, a)
    elif op == "WAIT":
        pass
    elif op == "DLY":
        t.sleep = b
    elif op == "HALT":
        t.running = False
        c.next_pc = t.pc
    else:
        raise IllegalInstruction(f"{word:08x} at pc {t.pc}")

    t.pc = c.next_pc
    return c


class WouldBlock(Exception):
    pass


class Machine:
    """
    Stand-alone model for running firmware logic without RTL: four threads,
    round-robin, with the host and inter-thread channels and the packet
    buffer (without its ECC). Lanes and GPIO are not modelled; firmware that
    needs them is tested on the RTL.
    """

    def __init__(self, words: dict):
        self.imem = [0] * isa.IMEM_WORDS
        for a, w in words.items():
            self.imem[a] = w
        self.threads = [Thread(t) for t in range(isa.NTHREADS)]
        self.host_in, self.host_out = [], []
        self.chan = {isa.IO["CH_A"]: [], isa.IO["CH_B"]: []}
        self.pbuf = [0] * isa.PBUF_WORDS
        self.rp = self.wp = 0

    def spawn(self, tid, pc):
        t = self.threads[tid]
        t.pc, t.running, t.sleep, t.regs = pc, True, 0, [0] * isa.NREGS

    def _io_read(self, t, addr):
        if addr == isa.IO["PB_DATA"]:
            v, self.rp = self.pbuf[self.rp], (self.rp + 1) % isa.PBUF_WORDS
            return v
        if addr in (isa.IO["PB_RP"], isa.IO["PB_WP"]):
            return self.rp if addr == isa.IO["PB_RP"] else self.wp
        q = self.host_in if addr == isa.IO["CH_HOST"] else self.chan.get(addr)
        if q is None:
            return t.tid if addr == isa.IO["TID"] else 0
        if not q:
            raise WouldBlock
        return q.pop(0)

    def _io_write(self, addr, v):
        if addr == isa.IO["CH_HOST"]:
            self.host_out.append(v)
        elif addr in self.chan:
            if len(self.chan[addr]) >= isa.CHAN_DEPTH:
                raise WouldBlock
            self.chan[addr].append(v)
        elif addr == isa.IO["SPAWN"]:
            self.spawn((v >> 9) & 3, v & PCMASK)
        elif addr == isa.IO["PB_DATA"]:
            self.pbuf[self.wp], self.wp = v, (self.wp + 1) % isa.PBUF_WORDS
        elif addr == isa.IO["PB_RP"]:
            self.rp = v % isa.PBUF_WORDS
        elif addr == isa.IO["PB_WP"]:
            self.wp = v % isa.PBUF_WORDS

    def tick(self):
        """One round: each running thread gets one issue slot."""
        progressed = False
        for t in self.threads:
            if not t.running:
                continue
            if t.sleep:
                t.sleep -= 1
                progressed = True
                continue
            saved = (t.pc, list(t.regs), t.running, t.sleep)
            try:
                step(t, self.imem[t.pc], lambda a, t=t: self._io_read(t, a), self._io_write)
            except WouldBlock:
                t.pc, t.regs, t.running, t.sleep = saved
                continue
            progressed = True
        return progressed

    def run(self, max_rounds=100_000):
        for _ in range(max_rounds):
            if not any(t.running for t in self.threads):
                return
            if not self.tick():
                return   # every live thread is blocked
        raise RuntimeError("ISS did not finish")
