/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * FIFO in a single-port SRAM, with the same interface as sh_fifo: `dout` is
 * the oldest word, valid in the same cycle as `!empty`, and a pop takes it.
 *
 * A single-port SRAM reads one cycle late and does one access per cycle, so
 * the oldest word lives in a head register:
 *
 *   push   into the head if the FIFO is empty, otherwise into the SRAM
 *   pop    empties the head; the SRAM's next word is read to refill it, and
 *          lands one cycle later
 *   clash  a push to the SRAM and a refill want the port in the same cycle;
 *          the push wins and the refill waits a cycle
 *
 * So after a pop the FIFO can read as empty for a cycle or two while the
 * next word is on its way. That only ever happens because of the pop
 * itself: the head fills, and `full` clears, by themselves, but they never
 * fall by themselves. The core decides readiness a cycle early from
 * registered state and relies on exactly that (docs/theory.md §15).
 *
 * Capacity is DEPTH words in total: head, a word in flight, and the SRAM.
 * Synthesis uses the IHP RM_IHPSG13_1P_64x16 macro (236.8 x 64.36 um,
 * ~15,200 um^2, 32 of its 64 words used); a 32 x 16 flop FIFO with its
 * read mux is ~43,000 um^2.
 *
 * The control state (pointers, hv, rd_q) is in self-correcting TMR: in the
 * fault-injection campaign (verify/fault) a flip there put a wrong word out
 * in nearly every trial. The head and the SRAM hold data and stay single.
 */
`default_nettype none

module sh_fifo_sram #(
    parameter int W     = 16,
    parameter int DEPTH = 32    // power of two, at most 64
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         push,
    input  logic [W-1:0] din,
    input  logic         pop,
    output logic [W-1:0] dout,
    output logic         empty,
    output logic         full,
    output logic         hit       // a TMR copy was outvoted this cycle
);

  localparam int AW = $clog2(DEPTH);

  logic [W-1:0] head;
  logic         hv;              // head holds a word
  logic         rd_q;            // a refill read was issued last cycle
  logic [AW:0]  wptr, rptr;      // SRAM entries: wptr - rptr
  logic [AW:0]  n_sram;
  logic [AW+1:0] total;

  assign n_sram = wptr - rptr;
  assign total  = (AW+2)'(hv) + (AW+2)'(rd_q) + (AW+2)'(n_sram);
  assign empty  = !hv;
  assign full   = total == (AW+2)'(DEPTH);
  assign dout   = head;

  logic do_push, do_pop;
  assign do_push = push && !full;
  assign do_pop  = pop && hv;

  // Where a pushed word goes, and whether the port is free for a refill.
  // Straight into the head only if nothing older exists anywhere.
  logic to_head, to_sram, refill;
  assign to_head = do_push && n_sram == '0 && !rd_q && (!hv || do_pop);
  assign to_sram = do_push && !to_head;
  assign refill  = !to_sram && n_sram != '0 && !rd_q && (!hv || do_pop);

  logic [W-1:0] q;

`ifdef SH_USE_MACRO

  RM_IHPSG13_1P_64x16_c2 u_sram (
      .A_CLK  (clk),
      .A_MEN  (to_sram || refill),
      .A_WEN  (to_sram),
      .A_REN  (refill),
      .A_ADDR (6'(to_sram ? wptr[AW-1:0] : rptr[AW-1:0])),
      .A_DIN  (din),
      .A_DLY  (1'b1),
      .A_DOUT (q)
  );

`else

  logic [W-1:0] mem [DEPTH];
  always_ff @(posedge clk) begin
    if (to_sram)     mem[wptr[AW-1:0]] <= din;
    else if (refill) q <= mem[rptr[AW-1:0]];
  end

`endif

  // The head fills when a refill lands (it was empty) or a push goes
  // straight in, and empties on a pop.
  logic [AW:0] wptr_n, rptr_n;
  logic        hv_n;
  // Ifs, not ?:, so X controls in simulation hold state as the old
  // always_ff did, instead of turning it to X. Later ifs take priority.
  always_comb begin
    wptr_n = wptr;
    rptr_n = rptr;
    hv_n   = hv;
    if (to_sram) wptr_n = wptr + 1'b1;
    if (refill)  rptr_n = rptr + 1'b1;
    if (do_pop)  hv_n   = 1'b0;
    if (to_head) hv_n   = 1'b1;
    if (rd_q)    hv_n   = 1'b1;
  end

  sh_tmr #(.W(2*AW+4)) u_ctl (
      .clk, .rst_n, .en(1'b1),
      .d({hv_n, refill, wptr_n, rptr_n}), .q({hv, rd_q, wptr, rptr}), .hit);

  always_ff @(posedge clk)
    if (!rst_n)       head <= '0;
    else if (rd_q)    head <= q;
    else if (to_head) head <= din;

`ifdef FORMAL
  // The head is only ever refilled while it is empty.
  always_comb if (rst_n && rd_q) assert (!hv);
  always_comb if (rst_n) assert (total <= (AW+2)'(DEPTH));
`endif

endmodule

`ifdef SH_USE_MACRO
/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
(* blackbox *)
module RM_IHPSG13_1P_64x16_c2 (
    input  logic        A_CLK, A_MEN, A_WEN, A_REN,
    input  logic [5:0]  A_ADDR,
    input  logic [15:0] A_DIN,
    input  logic        A_DLY,
    output logic [15:0] A_DOUT
);
endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
`endif
