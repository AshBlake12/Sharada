/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Packet buffer: 1024 x 16 (2 KB, a full 1518-byte Ethernet frame with room
 * to spare), on the IO bus. The host port moves a word per ~250 clocks and
 * 10BASE-T takes one per 96, so a frame is staged here before it goes out,
 * and a received frame lands here before firmware or the host looks at it.
 *
 * Two pointers, both auto-incrementing and both readable and writable:
 *
 *   PB_DATA  write: store at WP, WP++.  read: the word at RP, RP++.
 *   PB_RP    the read pointer.  Writing it moves the read side.
 *   PB_WP    the write pointer.
 *   PB_ECC   read: {ue seen, words repaired (sat. 31), last error address}.
 *            write: clear the log.
 *
 * Pointers wrap at 1024, so the buffer is also a ring: one thread can
 * receive into it while another sends out of it.
 *
 * The SRAM reads a cycle late, and the core decides readiness from
 * registered state (theory §15), so the word at RP is prefetched into a
 * register: a read of PB_DATA is ready when that register is full, takes it,
 * and the next word is fetched behind it. The register only empties because
 * of an access in E (a PB_DATA read or a PB_RP write), which the core
 * already corrects for, so readiness never falls on its own. A write to the
 * address RP points at updates the register as well, so a word written and
 * then read back is never stale.
 *
 * Writes are registered and committed the next cycle, so nothing from the
 * E stage reaches the SRAM pins in the same cycle. The port goes: pending
 * write, then a corrected word's write-back, then the prefetch.
 *
 * Every word is stored with 7 SECDED check bits. The (39,32) code in
 * common/sh_secded.sv, with data bits 16..31 fixed at zero, is a shortened
 * (23,16) code with the same guarantees: a single upset is corrected on the
 * way into the prefetch register and written back, a double upset is
 * flagged in the log. Firmware clears the log at the start of a frame and
 * reads it at the end, so a damaged frame is dropped, not sent.
 *
 * The pointers and the control flags are in a self-correcting ECC register
 * (common/sh_ecr.sv): in the
 * fault campaign (verify/fault) the FIFO pointers were the largest single
 * source of wrong words on the pins, and a flipped pointer here would send
 * or overwrite the wrong part of a frame without any ECC noticing. Data
 * registers stay single; ECC covers what they feed.
 *
 * Synthesis uses RM_IHPSG13_1P_1024x16 (236.8 x 336.46 um) and
 * RM_IHPSG13_1P_1024x8 for the check bits (146.88 x 336.46 um).
 */
`default_nettype none

module sh_pbuf #(
    parameter int AW = 10
) (
    input  logic          clk,
    input  logic          rst_n,

    // Accesses from the core's E stage, each already valid && ready
    input  logic          rd,        // PB_DATA read
    input  logic          wr,        // PB_DATA write
    input  logic          rp_wr,
    input  logic          wp_wr,
    input  logic          log_clr,
    input  logic [15:0]   wdata,

    output logic [15:0]   rdata,     // the word at RP
    output logic          rd_ok,     // rdata is valid: a PB_DATA read is ready
    output logic [AW-1:0] rp,
    output logic [AW-1:0] wp,
    output logic [15:0]   log,       // {ue, repaired[4:0], addr[9:0]}
    output logic          hit        // an upset was corrected this cycle
);

  // ------------------------------------------------------------------------
  // State. Control (ECC register): pointers, pending-write address, write-back
  // address, and the flags. Data: the words those flags qualify.
  // ------------------------------------------------------------------------
  logic          rd_v, rq, wq_v, fx_v;
  logic [AW-1:0] wq_a, fx_a;
  logic [15:0]   rd_word, wq_d, fx_d;

  // ------------------------------------------------------------------------
  // Port: pending write, then write-back, then prefetch. All from registers.
  // ------------------------------------------------------------------------
  logic          p_fx, p_rd, en, we;
  logic [AW-1:0] addr;
  logic [15:0]   din;
  assign p_fx = fx_v && !wq_v;
  assign p_rd = !wq_v && !fx_v && !rd_v && !rq;
  assign en   = wq_v || p_fx || p_rd;
  assign we   = wq_v || p_fx;
  assign addr = wq_v ? wq_a : p_fx ? fx_a : rp;
  assign din  = wq_v ? wq_d : fx_d;

  logic [6:0] cin;
  assign cin = sh_secded::check({16'h0, din});

  logic [15:0] dout;
  logic [7:0]  cout;

`ifdef SH_USE_MACRO

  RM_IHPSG13_1P_1024x16_c2_bm_bist u_sram (
      .A_CLK       (clk),
      .A_MEN       (en),
      .A_WEN       (we),
      .A_REN       (en && !we),
      .A_ADDR      (addr),
      .A_DIN       (din),
      .A_DLY       (1'b1),
      .A_DOUT      (dout),
      .A_BM        ({16{1'b1}}),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR ('0),
      .A_BIST_DIN  ('0),
      .A_BIST_BM   ('0)
  );

  RM_IHPSG13_1P_1024x8_c2_bm_bist u_sram_ecc (
      .A_CLK       (clk),
      .A_MEN       (en),
      .A_WEN       (we),
      .A_REN       (en && !we),
      .A_ADDR      (addr),
      .A_DIN       ({1'b0, cin}),
      .A_DLY       (1'b1),
      .A_DOUT      (cout),
      .A_BM        ({8{1'b1}}),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR ('0),
      .A_BIST_DIN  ('0),
      .A_BIST_BM   ('0)
  );

`else

  // Two arrays, like the two macros, so a testbench can flip a data bit or
  // a check bit on its own.
  logic [15:0] mem_d [1 << AW];
  logic [7:0]  mem_c [1 << AW];

  always_ff @(posedge clk) begin
    if (en) begin
      if (we) begin
        mem_d[addr] <= din;
        mem_c[addr] <= {1'b0, cin};
      end else begin
        dout <= mem_d[addr];
        cout <= mem_c[addr];
      end
    end
  end

`endif

  logic [31:0] q32;
  logic        ce, ue;
  sh_secded_dec u_dec (.d({16'h0, dout}), .c(cout[6:0]), .q(q32), .ce, .ue);

  // ------------------------------------------------------------------------
  // Next state
  // ------------------------------------------------------------------------
  // A write to the address RP points at: the prefetch register takes the
  // new word, and a read of that address still in flight is dropped.
  logic byp, land;
  assign byp  = wr && wp == rp;
  assign land = rq && !byp && !rp_wr;

  logic          rd_v_n, rq_n, wq_v_n, fx_v_n;
  logic [AW-1:0] rp_n, wp_n, wq_a_n, fx_a_n;
  // Ifs, not ?:, so X controls in simulation hold state instead of turning
  // it to X (as in sh_fifo_sram). Later ifs take priority.
  always_comb begin
    rp_n   = rp;
    wp_n   = wp;
    wq_a_n = wq_a;
    fx_a_n = fx_a;
    rd_v_n = rd_v;
    fx_v_n = fx_v;
    rq_n   = p_rd && !byp && !rp_wr;
    wq_v_n = 1'b0;
    if (p_fx) fx_v_n = 1'b0;
    if (land) begin
      rd_v_n = 1'b1;
      if (ce) begin
        fx_v_n = 1'b1;
        fx_a_n = rp;
      end
    end
    if (rd) begin
      rd_v_n = 1'b0;
      rp_n   = rp + 1'b1;
    end
    if (wr) begin
      wq_v_n = 1'b1;
      wq_a_n = wp;
      wp_n   = wp + 1'b1;
      if (fx_v && wp == fx_a) fx_v_n = 1'b0;   // the write-back is stale now
    end
    if (byp) rd_v_n = 1'b1;
    if (wp_wr) wp_n = wdata[AW-1:0];
    if (rp_wr) begin
      rp_n   = wdata[AW-1:0];
      rd_v_n = 1'b0;
    end
  end

  sh_ecr #(.W(4*AW+4)) u_ctl (
      .clk, .rst_n, .en(1'b1),
      .d({rd_v_n, rq_n, wq_v_n, fx_v_n, rp_n, wp_n, wq_a_n, fx_a_n}),
      .q({rd_v,   rq,   wq_v,   fx_v,   rp,   wp,   wq_a,   fx_a}),
      .hit);

  logic [4:0]    log_n;
  logic          log_ue;
  logic [AW-1:0] log_addr;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      rd_word <= '0; wq_d <= '0; fx_d <= '0;
      log_n <= '0; log_ue <= 1'b0; log_addr <= '0;
    end else begin
      if (land)        rd_word <= q32[15:0];
      if (byp)         rd_word <= wdata;
      if (wr)          wq_d    <= wdata;
      if (land && ce)  fx_d    <= q32[15:0];

      log_n    <= (log_clr ? 5'd0 : log_n) + 5'(land && ce && (log_clr || log_n != 5'h1F));
      log_ue   <= (log_ue && !log_clr) || (land && ue);
      if (land && (ce || ue)) log_addr <= rp;
    end
  end

  assign rdata = rd_word;
  assign rd_ok = rd_v;
  assign log   = {log_ue, log_n, log_addr};

`ifdef FORMAL
  always_comb if (rst_n && rq) assert (!rd_v);
`endif

endmodule

`ifdef SH_USE_MACRO
/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
(* blackbox *)
module RM_IHPSG13_1P_1024x16_c2_bm_bist (
    input  logic        A_CLK, A_MEN, A_WEN, A_REN,
    input  logic [9:0]  A_ADDR,
    input  logic [15:0] A_DIN,
    input  logic        A_DLY,
    output logic [15:0] A_DOUT,
    input  logic [15:0] A_BM,
    input  logic        A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
    input  logic [9:0]  A_BIST_ADDR,
    input  logic [15:0] A_BIST_DIN,
    input  logic [15:0] A_BIST_BM
);
endmodule

(* blackbox *)
module RM_IHPSG13_1P_1024x8_c2_bm_bist (
    input  logic        A_CLK, A_MEN, A_WEN, A_REN,
    input  logic [9:0]  A_ADDR,
    input  logic [7:0]  A_DIN,
    input  logic        A_DLY,
    output logic [7:0]  A_DOUT,
    input  logic [7:0]  A_BM,
    input  logic        A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
    input  logic [9:0]  A_BIST_ADDR,
    input  logic [7:0]  A_BIST_DIN,
    input  logic [7:0]  A_BIST_BM
);
endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
`endif
