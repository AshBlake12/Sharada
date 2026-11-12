/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Small synchronous FIFO. Flop-based: at these depths a macro would cost
 * more in routing blockage than it saves. Push when full and pop when empty
 * are ignored; callers check `full` / `empty` first.
 *
 * The pointers are in self-correcting TMR: in the fault-injection campaign
 * (verify/fault) a flip in either one put a wrong word out in every trial.
 * The words themselves are data and stay single; the protocol checks at the
 * far end are what catch those.
 */
`default_nettype none

module sh_fifo #(
    parameter int W     = 16,
    parameter int DEPTH = 4    // power of two
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

  logic [W-1:0] mem [DEPTH];
  logic [AW:0]  wptr, rptr;   // one extra bit tells full from empty
  logic [AW:0]  wptr_n, rptr_n;

  assign empty = (wptr == rptr);
  assign full  = (wptr[AW-1:0] == rptr[AW-1:0]) && (wptr[AW] != rptr[AW]);
  assign dout  = mem[rptr[AW-1:0]];

  // Written as ifs, not ?:, so an X push in simulation leaves the pointer
  // alone (as the old always_ff did) instead of turning it to X.
  always_comb begin
    wptr_n = wptr;
    rptr_n = rptr;
    if (push && !full) wptr_n = wptr + 1'b1;
    if (pop && !empty) rptr_n = rptr + 1'b1;
  end

  sh_tmr #(.W(2*AW+2)) u_ptr (
      .clk, .rst_n, .en(1'b1), .d({wptr_n, rptr_n}), .q({wptr, rptr}), .hit);

  always_ff @(posedge clk)
    if (rst_n && push && !full) mem[wptr[AW-1:0]] <= din;

`ifdef FORMAL
  // Occupancy never exceeds the depth (the induction invariant for
  // verify/monotone).
  always_comb if (rst_n) assert ((AW+1)'(wptr - rptr) <= (AW+1)'(DEPTH));
`endif

endmodule
