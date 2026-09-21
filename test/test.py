# SPDX-License-Identifier: Apache-2.0
# UART TX testbench — cocotb 2.x
#
# Two tests:
#   test_uart_bytes   decodes frames bit by bit and checks the payload
#   test_frame_period checks each frame is exactly 10 bit-times (8N1)
#
# Note: cocotb 2.x API. Many examples online are cocotb 1.x and use
# `units=` instead of `unit=` and `.value.integer` instead of int(...).

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

DIVISOR = 434          # must match localparam in src/project.sv
CLK_PERIOD_NS = 20     # 50 MHz, so one bit = 434 cycles = 8.68 us


def tx(dut) -> int:
    return int(dut.uo_out.value[0])


async def reset(dut):
    dut.ena.value = 1
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1


async def wait_for_start_bit(dut, timeout_cycles: int):
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if tx(dut) == 0:
            return
    raise AssertionError(f"no start bit within {timeout_cycles} cycles")


async def read_frame(dut) -> int:
    await wait_for_start_bit(dut, DIVISOR * 12)
    await ClockCycles(dut.clk, DIVISOR // 2)          # move to bit centre
    assert tx(dut) == 0, "start bit not low at its centre"

    byte = 0
    for i in range(8):                                 # LSB first
        await ClockCycles(dut.clk, DIVISOR)
        byte |= tx(dut) << i

    await ClockCycles(dut.clk, DIVISOR)
    assert tx(dut) == 1, "stop bit not high"
    return byte


@cocotb.test()
async def test_uart_bytes(dut):
    """Send several bytes and decode each frame."""
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())

    for value in (0xA5, 0x00, 0xFF, 0x3C, 0x81):
        dut.ui_in.value = value
        await reset(dut)
        got = await read_frame(dut)
        dut._log.info(f"sent 0x{value:02X}  received 0x{got:02X}")
        assert got == value, f"sent 0x{value:02X}, received 0x{got:02X}"


@cocotb.test()
async def test_frame_period(dut):
    """Start-bit to start-bit must be exactly 10 bit-times.

    Uses 0xFF so the line is high for all 8 data bits plus the stop bit:
    the only low period in each frame is the start bit, which makes the
    measurement unambiguous. A doubled stop bit shows up as 11 x DIVISOR.
    """
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, unit="ns").start())
    dut.ui_in.value = 0xFF
    await reset(dut)

    await wait_for_start_bit(dut, DIVISOR * 12)
    cycles = 0
    while tx(dut) == 0:
        await RisingEdge(dut.clk)
        cycles += 1
    while tx(dut) == 1:
        await RisingEdge(dut.clk)
        cycles += 1

    expected = 10 * DIVISOR
    dut._log.info(f"frame period {cycles} cycles, expected {expected}")
    assert cycles == expected, (
        f"frame is {cycles / DIVISOR:.2f} bit-times, expected 10 (8N1)"
    )