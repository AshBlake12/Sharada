/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Asynchronous FIFO between two unrelated clocks (Cummings, SNUG 2002).
 *
 * Each side keeps a binary pointer one bit wider than the address, and a
 * Gray-coded copy of it. Only the Gray pointer crosses, through sh_sync.
 * Gray code changes one bit per increment, so a sample taken while the
 * pointer moves is either the old value or the new one, never a mix. The
 * synchronised pointer is therefore an old but real value, and both flags
 * err on the safe side: the writer may see "full" a little too long, the
 * reader "empty" a little too long, and neither ever sees a word that is
 * not there. Memory is written in the write domain and read in the read
 * domain; a word is only read after its write has crossed via the pointer.
 */
`default_nettype none

module sh_afifo #(
    parameter int W  = 8,
    parameter int AW = 2      // depth 2**AW
) (
    input  logic         wclk,
    input  logic         wrst_n,
    input  logic         push,
    input  logic [W-1:0] din,
    output logic         full,

    input  logic         rclk,
    input  logic         rrst_n,
    input  logic         pop,
    output logic [W-1:0] dout,
    output logic         empty
);

  function automatic logic [AW:0] bin2gray(input logic [AW:0] b);
    bin2gray = b ^ (b >> 1);
  endfunction

  logic [W-1:0] mem [1 << AW];

  // ---------------- write domain ----------------
  logic [AW:0] wbin, wgray, rgray_w;
  logic [AW:0] wbin_next, wgray_next;

  assign wbin_next  = wbin + (AW+1)'(push && !full);
  assign wgray_next = bin2gray(wbin_next);

  always_ff @(posedge wclk) begin
    if (!wrst_n) begin
      wbin  <= '0;
      wgray <= '0;
      full  <= 1'b0;
    end else begin
      wbin  <= wbin_next;
      wgray <= wgray_next;
      // Full: the next write pointer is one lap ahead of the synced read
      // pointer. In Gray code that is "top two bits inverted, rest equal".
      full  <= wgray_next == {~rgray_w[AW:AW-1], rgray_w[AW-2:0]};
    end
  end

  always_ff @(posedge wclk)
    if (push && !full) mem[wbin[AW-1:0]] <= din;

  sh_sync #(.W(AW+1)) u_r2w (.clk(wclk), .rst_n(wrst_n), .d(rgray), .q(rgray_w));

  // ---------------- read domain ----------------
  logic [AW:0] rbin, rgray, wgray_r;
  logic [AW:0] rbin_next, rgray_next;

  assign rbin_next  = rbin + (AW+1)'(pop && !empty);
  assign rgray_next = bin2gray(rbin_next);

  always_ff @(posedge rclk) begin
    if (!rrst_n) begin
      rbin  <= '0;
      rgray <= '0;
      empty <= 1'b1;
    end else begin
      rbin  <= rbin_next;
      rgray <= rgray_next;
      empty <= rgray_next == wgray_r;
    end
  end

  assign dout = mem[rbin[AW-1:0]];

  sh_sync #(.W(AW+1)) u_w2r (.clk(rclk), .rst_n(rrst_n), .d(wgray), .q(wgray_r));

`ifdef FORMAL
  // Two-clock proof (verify/cdc/afifo.sby). Needs both resets low until both
  // sides have been reset (verify/cdc/fv_afifo.sv assumes that).
  function automatic logic [AW:0] g2b(input logic [AW:0] g);
    for (int i = AW; i >= 0; i--) g2b[i] = (i == AW) ? g[AW] : g2b[i+1] ^ g[i];
  endfunction

  logic [AW:0] occ, wseen_r, rseen_w;
  assign occ     = wbin - rbin;
  assign wseen_r = g2b(wgray_r);    // reader's view of the write pointer
  assign rseen_w = g2b(rgray_w);    // writer's view of the read pointer

  always_comb if (wrst_n && rrst_n) begin
    assert (occ <= (AW+1)'(1 << AW));                                  // no overflow
    assert ((AW+1)'(wseen_r - rbin) <= occ);                           // rbin <= view <= wbin
    assert ((AW+1)'(rbin - rseen_w) <= (AW+1)'(1 << AW) - occ);        // writer never sees space that isn't there
    assert (wgray == bin2gray(wbin));
    assert (rgray == bin2gray(rbin));
  end

  // Data: one arbitrary pointer value, written once, read back unchanged.
  (* anyconst *) logic [AW:0] f_ptr;
  logic         f_written;
  logic [W-1:0] f_data;
  initial f_written = 1'b0;
  always_ff @(posedge wclk)
    if (wrst_n && push && !full && wbin == f_ptr && !f_written) begin
      f_written <= 1'b1;
      f_data    <= din;
    end
  always_ff @(posedge rclk)
    if (wrst_n && rrst_n && pop && !empty && rbin == f_ptr && f_written) assert (dout == f_data);
`endif

endmodule
