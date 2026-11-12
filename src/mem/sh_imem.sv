/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Instruction memory: 512 x 32, single port, synchronous read (data appears
 * the cycle after the address). That one-cycle latency is why the core has
 * a separate fetch stage.
 *
 * Every word is stored with 7 SECDED check bits (common/sh_secded.sv), in a
 * second, 8-bit-wide SRAM beside the first.
 *
 * Correct by replay. The core decodes the word straight from the SRAM, as it
 * would without ECC, and the decoder runs beside it. If the word has an error
 * the decoder raises f_err in the same cycle and the core cancels that one
 * instruction in D: it never commits, so its pc does not move, and its thread
 * fetches the same address at its next slot, 4 cycles later. Meanwhile the
 * corrected word (or HALT, if two or more bits are bad) sits in a one-entry
 * fix buffer, and that refetch is served from it. Correcting in the fetch
 * path instead put 7 ns of decoder between the SRAM and decode and missed
 * setup by 4.8 ns at the slow corner. An upset now costs its thread one slot,
 * once; without one, timing is exactly as before.
 *
 * The fix buffer writes the corrected word back in the next cycle, ahead of
 * core fetches (`busy` holds the core off for that cycle), so a bad word is
 * repaired at once and two bad words fetched in turn cannot livelock the
 * buffer. An uncorrectable word is not written back.
 *
 * A scrubber covers the words nobody fetches: it walks the words the host
 * has written since reset, one read in each cycle the port is otherwise
 * idle, and writes back every word it had to correct. It only takes idle
 * cycles, so it never moves an instruction's timing.
 *
 * Port priority: host write, fix write-back, core fetch, scrubber. A host
 * write to a buffered address drops the buffer, and any host write cancels a
 * scrub write-back that is waiting, so stale data never overwrites new.
 *
 * Synthesis uses the IHP RM_IHPSG13_1P_512x32 macro (416.64 x 191.34 um,
 * ~79,700 um^2) and RM_IHPSG13_1P_512x8 for the check bits (236.8 x 110.38
 * um, ~26,100 um^2). The same storage in flops would be most of the die.
 * Simulation uses the behavioural model below unless SH_USE_MACRO is defined
 * (the LibreLane config defines it).
 */
`default_nettype none

module sh_imem #(
    parameter int AW = 9
) (
    input  logic          clk,
    input  logic          rst_n,

    // Core fetch
    input  logic          f_en,
    input  logic [AW-1:0] f_addr,
    output logic [31:0]   f_word,     // the cycle after f_en
    output logic          f_err,      // with f_word: cancel it, the refetch gets the fix
    output logic          busy,       // the port is taken this cycle: don't fetch

    // Host write
    input  logic          w_en,
    input  logic [AW-1:0] w_addr,
    input  logic [31:0]   w_data,

    // Error log, cleared by the host
    input  logic          log_clr,
    output logic [7:0]    log_fixed,  // words rewritten with a correction (saturates)
    output logic          log_ue,     // an uncorrectable word was seen
    output logic [AW-1:0] log_addr    // address of the latest error found
);

  // ------------------------------------------------------------------------
  // Fix buffer and scrubber state
  // ------------------------------------------------------------------------
  logic          fx_valid, fx_wb;  // buffer holds a word; it still has to be written back
  logic [AW-1:0] fx_addr;
  logic [31:0]   fx_word;

  typedef enum logic [1:0] {SC_READ, SC_CHECK, SC_WRITE} sc_t;
  sc_t           sc;
  logic [AW-1:0] sc_addr;      // next word to scrub
  logic [AW:0]   sc_limit;     // words written since reset: 0 .. limit-1
  logic [31:0]   sc_fix;       // corrected word waiting to be written back

  // ------------------------------------------------------------------------
  // Port arbitration
  // ------------------------------------------------------------------------
  logic fx_wr, idle, sc_rd, sc_wr;
  assign fx_wr = fx_wb && !w_en;
  assign busy  = fx_wb;
  assign idle  = !w_en && !fx_wb && !f_en;
  assign sc_rd = idle && sc == SC_READ && sc_limit != '0;
  assign sc_wr = idle && sc == SC_WRITE;

  logic          en, we;
  logic [AW-1:0] addr;
  logic [31:0]   din;
  assign en   = w_en || fx_wr || f_en || sc_rd || sc_wr;
  assign we   = w_en || fx_wr || sc_wr;
  assign addr = w_en ? w_addr : fx_wr ? fx_addr : f_en ? f_addr : sc_addr;
  assign din  = w_en ? w_data : fx_wr ? fx_word : sc_fix;

  logic [6:0]  cin;
  assign cin = sh_secded::check(din);

  // ------------------------------------------------------------------------
  // Storage
  // ------------------------------------------------------------------------
  logic [31:0] dout;
  logic [7:0]  cout;

`ifdef SH_USE_MACRO

  RM_IHPSG13_1P_512x32_c2_bm_bist u_sram (
      .A_CLK       (clk),
      .A_MEN       (en),
      .A_WEN       (we),
      .A_REN       (en && !we),
      .A_ADDR      (addr),
      .A_DIN       (din),
      .A_DLY       (1'b1),
      .A_DOUT      (dout),
      .A_BM        ({32{1'b1}}),
      .A_BIST_CLK  (1'b0),
      .A_BIST_EN   (1'b0),
      .A_BIST_MEN  (1'b0),
      .A_BIST_WEN  (1'b0),
      .A_BIST_REN  (1'b0),
      .A_BIST_ADDR ('0),
      .A_BIST_DIN  ('0),
      .A_BIST_BM   ('0)
  );

  RM_IHPSG13_1P_512x8_c3_bm_bist u_sram_ecc (
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
  logic [31:0] mem_d [1 << AW];
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

  // ------------------------------------------------------------------------
  // Fetch result. The hit on the fix buffer is decided in F and registered,
  // so the only thing between the SRAM and the core's decode is one mux.
  // ------------------------------------------------------------------------
  logic          f_q, f_hit_q, sc_q;
  logic [AW-1:0] f_addr_q;

  logic [31:0] q;
  logic        ce, ue;
  sh_secded_dec u_dec (.d(dout), .c(cout[6:0]), .q, .ce, .ue);

  assign f_word = f_hit_q ? fx_word : dout;
  assign f_err  = f_q && !f_hit_q && (ce || ue);

  // ------------------------------------------------------------------------
  // Fix buffer, scrubber, error log
  // ------------------------------------------------------------------------
  logic sc_fixed;
  assign sc_fixed = sc_wr;

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      fx_valid <= 1'b0; fx_wb <= 1'b0; fx_addr <= '0; fx_word <= '0;
      sc <= SC_READ; sc_addr <= '0; sc_limit <= '0; sc_fix <= '0;
      f_q <= 1'b0; f_hit_q <= 1'b0; sc_q <= 1'b0; f_addr_q <= '0;
      log_fixed <= '0; log_ue <= 1'b0; log_addr <= '0;
    end else begin
      f_q      <= f_en && !w_en && !fx_wb;
      f_hit_q  <= fx_valid && f_addr == fx_addr;
      sc_q     <= sc_rd;
      f_addr_q <= f_addr;

      if (w_en && {1'b0, w_addr} >= sc_limit) sc_limit <= {1'b0, w_addr} + 1'b1;

      // Fix buffer: take the corrected word (HALT if it cannot be corrected),
      // write it back next cycle, keep serving it until the host rewrites it.
      if (fx_wr) fx_wb <= 1'b0;
      if (f_err) begin
        fx_valid <= 1'b1;
        fx_wb    <= !ue;
        fx_addr  <= f_addr_q;
        fx_word  <= ue ? {sh_pkg::OP_HALT, 26'd0} : q;
      end
      if (w_en && fx_valid && w_addr == fx_addr) begin
        fx_valid <= 1'b0;
        fx_wb    <= 1'b0;
      end

      // Scrubber
      unique case (sc)
        SC_READ:  if (sc_rd) sc <= SC_CHECK;
        SC_CHECK: begin
          if (w_en) sc <= SC_READ;               // the host may have rewritten this word
          else if (ce) begin
            sc     <= SC_WRITE;
            sc_fix <= q;
          end else begin
            sc      <= SC_READ;
            sc_addr <= ({1'b0, sc_addr} + 1'b1 >= sc_limit) ? '0 : sc_addr + 1'b1;
          end
        end
        SC_WRITE: begin
          if (w_en) sc <= SC_READ;               // the host rewrote memory: read again
          else if (sc_wr) begin
            sc      <= SC_READ;
            sc_addr <= ({1'b0, sc_addr} + 1'b1 >= sc_limit) ? '0 : sc_addr + 1'b1;
          end
        end
        default: sc <= SC_READ;
      endcase

      log_fixed <= (log_clr ? 8'd0 : log_fixed)
                   + 8'((fx_wr || sc_fixed) && (log_clr || log_fixed != 8'hFF));
      log_ue    <= (log_ue && !log_clr) || ((f_err || sc_q) && ue);
      if (f_err || (sc_q && (ce || ue))) log_addr <= f_err ? f_addr_q : sc_addr;
    end
  end

endmodule

`ifdef SH_USE_MACRO
// Port lists of the IHP macros, so lint and synthesis see black boxes. The
// layout, timing and power views come from the PDK via MACROS in config.json.
/* verilator lint_off UNDRIVEN */
/* verilator lint_off UNUSEDSIGNAL */
(* blackbox *)
module RM_IHPSG13_1P_512x32_c2_bm_bist (
    input  logic        A_CLK, A_MEN, A_WEN, A_REN,
    input  logic [8:0]  A_ADDR,
    input  logic [31:0] A_DIN,
    input  logic        A_DLY,
    output logic [31:0] A_DOUT,
    input  logic [31:0] A_BM,
    input  logic        A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
    input  logic [8:0]  A_BIST_ADDR,
    input  logic [31:0] A_BIST_DIN,
    input  logic [31:0] A_BIST_BM
);
endmodule

(* blackbox *)
module RM_IHPSG13_1P_512x8_c3_bm_bist (
    input  logic        A_CLK, A_MEN, A_WEN, A_REN,
    input  logic [8:0]  A_ADDR,
    input  logic [7:0]  A_DIN,
    input  logic        A_DLY,
    output logic [7:0]  A_DOUT,
    input  logic [7:0]  A_BM,
    input  logic        A_BIST_CLK, A_BIST_EN, A_BIST_MEN, A_BIST_WEN, A_BIST_REN,
    input  logic [8:0]  A_BIST_ADDR,
    input  logic [7:0]  A_BIST_DIN,
    input  logic [7:0]  A_BIST_BM
);
endmodule
/* verilator lint_on UNUSEDSIGNAL */
/* verilator lint_on UNDRIVEN */
`endif
