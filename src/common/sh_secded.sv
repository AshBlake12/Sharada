/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * SECDED Hamming (39,32): corrects any single-bit error in a 32-bit word and
 * its 7 check bits, and detects any double-bit error.
 *
 * Codeword positions 1..38: check bits c[k] at the powers of two (1, 2, 4,
 * 8, 16, 32), data bits in order at the other 32 positions. c[k] is the XOR
 * of every position with bit k set, so after a single flip the syndrome
 * (recomputed XOR stored, 6 bits) is the flipped position. c[6] is the XOR
 * of everything else, so a single flip always makes the overall parity odd
 * and a double flip leaves it even with a non-zero syndrome.
 *
 * Reference model: asm/sharada/secded.py. Proof that every single flip is
 * corrected and every double flip detected: verify/secded/.
 */
`default_nettype none

package sh_secded;

  // Codeword position of data bit j. Written in the plain Verilog subset
  // (no return, no ++, loop variables declared up front) because Yosys
  // evaluates it as a constant function.
  function automatic integer pos(input integer j);
    integer p, n;
    begin
      n   = 0;
      pos = 0;
      for (p = 1; p <= 38; p = p + 1) begin
        if ((p & (p - 1)) != 0) begin
          if (n == j) pos = p;
          n = n + 1;
        end
      end
    end
  endfunction

  // Data bits that feed check bit k.
  function automatic logic [31:0] mask(input integer k);
    integer j;
    begin
      mask = 32'd0;
      for (j = 0; j < 32; j = j + 1)
        if (((pos(j) >> k) & 1) != 0) mask[j] = 1'b1;
    end
  endfunction

  // Elaboration-time constants, so neither simulation nor synthesis runs
  // the loops above per evaluation.
  localparam logic [31:0] M0 = mask(0), M1 = mask(1), M2 = mask(2),
                          M3 = mask(3), M4 = mask(4), M5 = mask(5);

  function automatic logic [6:0] check(input logic [31:0] d);
    logic [5:0] c;
    begin
      c = {^(d & M5), ^(d & M4), ^(d & M3), ^(d & M2), ^(d & M1), ^(d & M0)};
      check = {^{d, c}, c};
    end
  endfunction

endpackage

// Decoder: q is d corrected; ce = one bit was wrong (data or check), ue = more.
module sh_secded_dec (
    input  logic [31:0] d,
    input  logic [6:0]  c,
    output logic [31:0] q,
    output logic        ce,
    output logic        ue
);

  logic [6:0] r;
  logic [5:0] s;
  logic       odd;
  assign r   = sh_secded::check(d);
  assign s   = r[5:0] ^ c[5:0];
  assign odd = ^{d, c};

  for (genvar j = 0; j < 32; j++) begin : g_fix
    localparam logic [5:0] P = 6'(sh_secded::pos(j));
    assign q[j] = d[j] ^ (odd && s == P);
  end

  assign ce = odd && s <= 6'd38;
  assign ue = (!odd && s != '0) || (odd && s > 6'd38);

endmodule
