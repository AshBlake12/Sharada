/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Self-correcting TMR register. Three copies, a bitwise majority vote on the
 * output, and the vote fed back into all three every cycle the register is
 * not written. An upset in one copy is outvoted at once and overwritten at
 * the next clock, so upsets cannot pile up into two bad copies (a plain TMR
 * register without feedback only survives until the second hit on the same
 * bit). `hit` is high for the cycle in which a copy disagrees.
 *
 * Used for narrow control state (thread control, FIFO pointers). Below
 * about 20 bits it is smaller than the self-correcting ECC register
 * (sh_ecr.sv), which holds the wide words. Defining SH_NO_TMR builds one
 * copy instead, for area comparisons.
 *
 * Each copy is its own process marked keep. The copies have identical
 * inputs, so without that Yosys merges them into one flop (opt_merge) and
 * the voter into a wire; keep on the wires is not enough. Check the flop
 * count in the netlist after any change here.
 */
`default_nettype none

module sh_tmr #(
    parameter int          W   = 1,
    parameter logic [W-1:0] RST = '0
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         en,
    input  logic [W-1:0] d,
    output logic [W-1:0] q,
    output logic         hit
`ifdef SH_FV_TMR
  , input  logic [W-1:0] inj0, inj1, inj2   // proof only: upsets, XORed into the copies
`endif
);

`ifdef SH_NO_TMR

  always_ff @(posedge clk)
    if (!rst_n)  q <= RST;
    else if (en) q <= d;
  assign hit = 1'b0;

`else

  logic [W-1:0] r0, r1, r2, n;
  assign q   = (r0 & r1) | (r1 & r2) | (r0 & r2);
  assign hit = (r0 != r1) || (r1 != r2);
  assign n   = en ? d : q;

`ifdef SH_FV_TMR
  always_ff @(posedge clk) r0 <= rst_n ? n ^ inj0 : RST;
  always_ff @(posedge clk) r1 <= rst_n ? n ^ inj1 : RST;
  always_ff @(posedge clk) r2 <= rst_n ? n ^ inj2 : RST;
  // Induction invariant: at most one copy differs from the vote.
  always_comb if (rst_n) assert (32'(r0 != q) + 32'(r1 != q) + 32'(r2 != q) <= 1);
`else
  (* keep *) always_ff @(posedge clk) r0 <= rst_n ? n : RST;
  (* keep *) always_ff @(posedge clk) r1 <= rst_n ? n : RST;
  (* keep *) always_ff @(posedge clk) r2 <= rst_n ? n : RST;
`endif

`endif

endmodule
