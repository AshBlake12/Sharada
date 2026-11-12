/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Two-flop synchroniser for asynchronous pin inputs. The first flop may go
 * metastable; the second gives it a full clock period to resolve. Every pin
 * read anywhere in the design goes through exactly one of these.
 */
`default_nettype none

module sh_sync #(
    parameter int W = 1
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic [W-1:0] d,
    output logic [W-1:0] q
);

  logic [W-1:0] meta;

  // What metastability does to logic: on the first edge after the input
  // changes, a bit that was changing may resolve to its old value. The
  // first flop then holds the input as it was just before that change, and
  // catches up on the next edge. Real silicon does this; a plain simulator
  // never does, which is how CDC bugs pass simulation. SH_META_SIM picks the
  // old value for each such bit with probability 1/4; under FORMAL the
  // solver picks, every time, for every bit.
  logic [W-1:0] late, d_before, d_late;
  logic         fresh;            // input changed since the last edge
`ifdef FORMAL
  (* anyseq *) logic [W-1:0] late_any;
  logic [W-1:0] d_last, d_before_q;
  logic         clk_last, fresh_q;
  assign late = late_any;
  // Combinational view of "the value before the latest change" so that a
  // change in this solver step is already visible at an edge in this step.
  assign d_before = (d != d_last) ? d_last : d_before_q;
  assign fresh    = rst_n && ((d != d_last) || fresh_q);
  initial fresh_q = 1'b0;
  // Changes made while this side is in reset settled long before it leaves.
  always_ff @($global_clock) begin
    d_last     <= d;
    clk_last   <= clk;
    d_before_q <= d_before;
    if (!rst_n || (clk && !clk_last)) fresh_q <= (d != d_last) && rst_n && !(clk && !clk_last);
    else if (d != d_last)              fresh_q <= 1'b1;
  end
`elsif SH_META_SIM
  logic [W-1:0] d_last = '0;
  initial begin d_before = '0; fresh = 1'b0; end
  always @(d) begin
    d_before = d_last;
    d_last   = d;
    fresh    = 1'b1;
  end
  always @(posedge clk) #0 fresh = 1'b0;
  always_ff @(posedge clk) late <= W'($urandom) & W'($urandom);
`else
  assign late     = '0;
  assign d_before = '0;
  assign fresh    = 1'b0;
`endif
  assign d_late = fresh ? d_before : d;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      meta <= '0;
      q    <= '0;
    end else begin
      meta <= (d_late & late) | (d & ~late);
      q    <= meta;
    end
  end

endmodule
