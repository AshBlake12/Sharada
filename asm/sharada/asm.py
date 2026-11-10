"""
Two-pass assembler for the Sharada ISA.

    python3 -m sharada.asm programs/uart_echo.s -o uart_echo.hex

Syntax (one statement per line, `;` or `#` starts a comment):

    label:
    .equ  NAME, expr
    .org  expr
    .word expr
    add   rd, rs, rt|expr        ; every ALU op: rd = rs op B
    mov   rd, rt|expr            ; pseudo: or rd, r0, B
    beq   rs, rt, label          ; bne bltu bgeu
    bbs   rs, bit, label         ; bbc
    djnz  rd, label
    jmp   label
    jal   rd, label
    jr    rs
    ior   rd, ioaddr
    iow   ioaddr, rs
    wait  lane, mask             ; event wait
    waitp pin, level             ; pin wait
    dly   rt|expr
    halt / nop

Expressions are Python-style integer arithmetic over numbers, labels,
.equ names and the ISA symbols (L0_DATA, C_TXEN, E_RXF, P_UIO3, ...).
"""

import argparse
import ast
import operator
import re
import sys

from . import isa

_BINOPS = {
    ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul,
    ast.FloorDiv: operator.floordiv, ast.Mod: operator.mod,
    ast.LShift: operator.lshift, ast.RShift: operator.rshift,
    ast.BitOr: operator.or_, ast.BitAnd: operator.and_, ast.BitXor: operator.xor,
}
_UNOPS = {ast.USub: operator.neg, ast.Invert: operator.invert, ast.UAdd: operator.pos}

REG_RE = re.compile(r"^r([0-7])$", re.I)


class AsmError(Exception):
    pass


def _eval(node, env):
    if isinstance(node, ast.Expression):
        return _eval(node.body, env)
    if isinstance(node, ast.Constant) and isinstance(node.value, int):
        return node.value
    if isinstance(node, ast.Name):
        if node.id not in env:
            raise AsmError(f"undefined symbol '{node.id}'")
        return env[node.id]
    if isinstance(node, ast.BinOp) and type(node.op) in _BINOPS:
        return _BINOPS[type(node.op)](_eval(node.left, env), _eval(node.right, env))
    if isinstance(node, ast.UnaryOp) and type(node.op) in _UNOPS:
        return _UNOPS[type(node.op)](_eval(node.operand, env))
    raise AsmError(f"unsupported expression: {ast.dump(node)}")


def evaluate(text: str, env: dict) -> int:
    try:
        tree = ast.parse(text.strip(), mode="eval")
    except SyntaxError as e:
        raise AsmError(f"bad expression '{text}'") from e
    return _eval(tree, env)


def split_operands(s: str) -> list:
    """Split on commas that are not inside parentheses."""
    out, depth, cur = [], 0, ""
    for ch in s:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def is_reg(tok: str) -> bool:
    return bool(REG_RE.match(tok))


def reg(tok: str) -> int:
    m = REG_RE.match(tok)
    if not m:
        raise AsmError(f"expected register, got '{tok}'")
    return int(m.group(1))


class Assembler:
    def __init__(self):
        self.env = isa.symbols()
        self.words = {}       # addr -> word
        self.lines = {}       # addr -> source line (for listings / traces)

    # ---- pass helpers ---------------------------------------------------
    @staticmethod
    def _clean(line: str) -> str:
        return re.split(r"[;#]", line, maxsplit=1)[0].strip()

    def _statements(self, text: str):
        for lineno, raw in enumerate(text.splitlines(), 1):
            line = self._clean(raw)
            while True:
                m = re.match(r"^([A-Za-z_]\w*)\s*:\s*(.*)$", line)
                if not m:
                    break
                yield lineno, raw, ("label", m.group(1))
                line = m.group(2)
            if line:
                parts = line.split(None, 1)
                mnem = parts[0].lower()
                ops = split_operands(parts[1]) if len(parts) > 1 else []
                yield lineno, raw, (mnem, ops)

    def assemble(self, text: str) -> dict:
        stmts = list(self._statements(text))
        # Pass 1: addresses and .equ
        pc = 0
        for lineno, raw, (kind, arg) in stmts:
            try:
                if kind == "label":
                    if arg in self.env:
                        raise AsmError(f"symbol '{arg}' redefined")
                    self.env[arg] = pc
                elif kind == ".equ":
                    self.env[arg[0]] = evaluate(arg[1], self.env)
                elif kind == ".org":
                    pc = evaluate(arg[0], self.env)
                else:
                    pc += 1
            except AsmError as e:
                raise AsmError(f"line {lineno}: {e}") from None
        # Pass 2: encode
        pc = 0
        for lineno, raw, (kind, arg) in stmts:
            try:
                if kind in ("label", ".equ"):
                    continue
                if kind == ".org":
                    pc = evaluate(arg[0], self.env)
                    continue
                if pc >= isa.IMEM_WORDS:
                    raise AsmError("program exceeds instruction memory")
                if pc in self.words:
                    raise AsmError(f"address {pc} written twice")
                self.words[pc] = self._encode(kind, arg, pc)
                self.lines[pc] = raw.strip()
                pc += 1
            except AsmError as e:
                raise AsmError(f"line {lineno}: {e}") from None
        return self.words

    # ---- encoding -------------------------------------------------------
    def _imm(self, tok, lo=-(1 << 15), hi=(1 << 16) - 1, what="immediate"):
        v = evaluate(tok, self.env)
        if not lo <= v <= hi:
            raise AsmError(f"{what} {v} out of range [{lo}, {hi}]")
        return v & 0xFFFF

    def _target(self, tok):
        return self._imm(tok, 0, isa.IMEM_WORDS - 1, "branch target")

    def _b(self, tok):
        """Second operand: register or immediate. Returns (rt, i, imm)."""
        if is_reg(tok):
            return reg(tok), 0, 0
        return 0, 1, self._imm(tok)

    def _encode(self, m, ops, pc):
        def need(n):
            if len(ops) != n:
                raise AsmError(f"'{m}' takes {n} operands, got {len(ops)}")

        E = isa.encode
        if m == ".word":
            need(1)
            return evaluate(ops[0], self.env) & 0xFFFFFFFF
        if m in ("nop", "halt"):
            need(0)
            return E(m.upper())
        if m == "mov":
            need(2)
            rt, i, imm = self._b(ops[1])
            return E("OR", rd=reg(ops[0]), rs=0, rt=rt, i=i, imm=imm)
        if m.upper() in isa.ALU_OPS:
            need(3)
            rt, i, imm = self._b(ops[2])
            return E(m.upper(), rd=reg(ops[0]), rs=reg(ops[1]), rt=rt, i=i, imm=imm)
        if m.upper() in isa.REG_BRANCHES:
            need(3)
            return E(m.upper(), rs=reg(ops[0]), rt=reg(ops[1]), imm=self._target(ops[2]))
        if m.upper() in isa.BIT_BRANCHES:
            need(3)
            bit = self._imm(ops[1], 0, 15, "bit index")
            return E(m.upper(), rs=reg(ops[0]), imm=(bit << 12) | self._target(ops[2]))
        if m == "djnz":
            need(2)
            r = reg(ops[0])
            return E("DJNZ", rd=r, rs=r, imm=self._target(ops[1]))
        if m == "jmp":
            need(1)
            return E("JMP", imm=self._target(ops[0]))
        if m == "jal":
            need(2)
            return E("JAL", rd=reg(ops[0]), imm=self._target(ops[1]))
        if m == "jr":
            need(1)
            return E("JR", rs=reg(ops[0]))
        if m == "ior":
            need(2)
            return E("IOR", rd=reg(ops[0]), imm=self._imm(ops[1], 0, 255, "io address"))
        if m == "iow":
            need(2)
            return E("IOW", rs=reg(ops[1]), imm=self._imm(ops[0], 0, 255, "io address"))
        if m == "wait":
            need(2)
            lane = self._imm(ops[0], 0, isa.NLANES - 1, "lane")
            mask = self._imm(ops[1], 1, 255, "event mask")
            return E("WAIT", i=1, imm=(lane << 8) | (mask & 0xFF)
                     | ((mask >> 8) << isa.WAIT_EV_HI_SHIFT))
        if m == "waitp":
            need(2)
            pin = self._imm(ops[0], 0, 23, "pin")
            lvl = self._imm(ops[1], 0, 1, "level")
            return E("WAIT", i=1, imm=isa.WAIT_PIN_FLAG | (lvl << 5) | pin)
        if m == "dly":
            need(1)
            rt, i, imm = self._b(ops[0])
            return E("DLY", rt=rt, i=i, imm=imm)
        raise AsmError(f"unknown mnemonic '{m}'")


def assemble(text: str) -> dict:
    return Assembler().assemble(text)


def assemble_file(path: str) -> dict:
    with open(path) as f:
        return assemble(f.read())


def to_hex(words: dict) -> str:
    n = max(words) + 1 if words else 0
    return "".join(f"{words.get(a, 0):08x}\n" for a in range(n))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    ap.add_argument("src")
    ap.add_argument("-o", "--out")
    ap.add_argument("-l", "--listing", action="store_true")
    a = ap.parse_args(argv)
    asm = Assembler()
    try:
        words = asm.assemble(open(a.src).read())
    except AsmError as e:
        sys.exit(f"{a.src}: {e}")
    if a.listing:
        for addr in sorted(words):
            print(f"{addr:03x}: {words[addr]:08x}   {asm.lines[addr]}")
    if a.out:
        open(a.out, "w").write(to_hex(words))


if __name__ == "__main__":
    main()
