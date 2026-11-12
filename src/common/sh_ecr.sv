/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Self-correcting ECC register: a drop-in for sh_tmr at about half the flops.
 *
 * The value is stored as an extended Hamming (SECDED) codeword: W data bits,
 * R check bits (the fewest with 2^R >= W + R + 1) and one overall parity
 * bit. Every clock the stored word is decoded, corrected, and written back
 * re-encoded, the same feedback that makes sh_tmr self-correcting. So an
 * upset is corrected on the output at once and gone from the flops at the
 * next clock, and upsets cannot pile up.
 *
 *            flops   W=8   W=16   W=24   W=44
 *   sh_tmr    3W      24     48     72    132
 *   sh_ecr  W+R+1     13     22     30     51
 *
 * What it survives, per clock:
 *   one flipped bit anywhere in the word: masked, repaired next clock (`hit`).
 *   two flipped bits: detected, never decoded to a wrong value. The output
 *     falls back to SAFE and the register is rewritten to SAFE, so the state
 *     that decides whether a pin is driven lets go of the pin. (TMR with two
 *     copies of a bit hit outputs the wrong value and keeps it.)
 *   three or more: no promise; the syndrome can alias to a single error.
 *
 * TMR survives any number of flips confined to one copy, which ECC does not:
 * one strike that upsets two adjacent flops of the same word is a double
 * error here. Detected and safe, but not masked.
 *
 * The decoder sits between the flops and q (a 4- to 6-level XOR tree and a
 * compare), on paths with several ns of slack at 60 MHz.
 *
 * Codeword layout, cw = {p, c[R-1:0], data[W-1:0]}: positions 1..W+R of the
 * Hamming code hold the check bits at the powers of two and the data bits
 * in order elsewhere; p is the parity over all of them. `asm/sharada/
 * secded.py` uses the same numbering for the (39,32) instruction code.
 *
 * q_raw is the stored data bits with no decoder in front, for logic that
 * reads the register every cycle on a tight path. It equals q except in
 * the cycle an upset lands: the write-back repairs the flops at the next
 * clock, so a wrong value on q_raw lasts exactly one cycle. Anything that
 * decides whether or how a pin is driven must use q; q_raw is for state
 * whose one-cycle error the far end's checks catch (bit timing, CRC
 * configuration). The decoder costs about 4 ns at the slow corner, which a
 * lane's bit timer cannot spare at 60 MHz.
 *
 * Defining SH_ECR_AS_TMR builds sh_tmr in its place, for A/B comparisons.
 */
`default_nettype none

module sh_ecr #(
    parameter int           W    = 1,
    parameter logic [W-1:0] RST  = '0,
    parameter logic [W-1:0] SAFE = RST
) (
    input  logic         clk,
    input  logic         rst_n,
    input  logic         en,
    input  logic [W-1:0] d,
    output logic [W-1:0] q,
    output logic [W-1:0] q_raw,   // the stored data bits, no decoder: see below
    output logic         hit
`ifdef SH_FV_ECR
  , input  logic [63:0]  inj     // proof only: upsets on the flops' outputs this cycle
  , output logic [W-1:0] held    // proof only: the data the flops hold
`endif
);

  // Fewest check bits for W data bits.
  function automatic int rbits(int w);
    rbits = 7;
    for (int r = 7; r >= 2; r--)
      if ((1 << r) >= w + r + 1) rbits = r;
  endfunction

  localparam int R = rbits(W);
  localparam int N = W + R + 1;

  // Hamming position of each data bit, 8 bits apiece: the positions that
  // are not powers of two, in order (3, 5, 6, 7, 9, ...).
  function automatic logic [8*W-1:0] dpos_table();
    int i;
    i = 0;
    dpos_table = '0;
    for (int p = 3; p < 256; p++)
      if ((p & (p - 1)) != 0 && i < W) begin
        dpos_table[8*i +: 8] = 8'(p);
        i++;
      end
  endfunction

  // The data bits each check bit covers: bit k of each data bit's position.
  function automatic logic [R*W-1:0] cover_table();
    logic [8*W-1:0] dp;
    dp = dpos_table();
    for (int k = 0; k < R; k++)
      for (int i = 0; i < W; i++) cover_table[W*k + i] = dp[8*i + k];
  endfunction

  // Tables, not function calls in the logic: computed once at elaboration,
  // so simulators do not rerun the search loops on every evaluation.
  localparam logic [8*W-1:0] DPOS  = dpos_table();
  localparam logic [R*W-1:0] COVER = cover_table();

  function automatic logic [R-1:0] check(logic [W-1:0] x);
    for (int k = 0; k < R; k++) check[k] = ^(x & COVER[W*k +: W]);
  endfunction

  function automatic logic [N-1:0] encode(logic [W-1:0] x);
    logic [R-1:0] c;
    c      = check(x);
    encode = {^{x, c}, c, x};
  endfunction

`ifdef SH_ECR_AS_TMR

  sh_tmr #(.W(W), .RST(RST)) u_tmr (.clk, .rst_n, .en, .d, .q, .hit);
  assign q_raw = q;

`else

  logic [N-1:0] cw;       // the stored word as read, upsets included
  logic [W-1:0] cd;
  logic [R-1:0] cc;
  assign {cc, cd} = cw[N-2:0];     // the parity bit only enters through ^cw

  // Decode. The syndrome names the flipped position; overall parity says
  // whether an odd number of bits flipped.
  logic [R-1:0] s;
  logic         odd, ue;
  logic [W-1:0] fix;
  assign s   = cc ^ check(cd);
  assign odd = ^cw;
  assign ue  = odd ? (32'(s) > W + R) : (s != '0);
  always_comb
    for (int i = 0; i < W; i++) fix[i] = odd && 8'(s) == DPOS[8*i +: 8];

  assign q     = ue ? SAFE : cd ^ fix;
  assign q_raw = cd;
  assign hit = odd || ue;

  logic [N-1:0] n;
  assign n = encode(en ? d : q);

`ifdef SH_FV_ECR
  // An upset shows on a flop's output until the next clock overwrites it,
  // which is what the write-back does to a real one.
  logic [N-1:0] cw_r;
  always_ff @(posedge clk) cw_r <= rst_n ? n : encode(RST);
  assign cw   = cw_r ^ inj[N-1:0];
  assign held = cw_r[W-1:0];
  // Induction invariant: the flops always hold a codeword.
  always_comb if (rst_n) assert (cw_r == encode(cw_r[W-1:0]));
`else
  always_ff @(posedge clk) cw <= rst_n ? n : encode(RST);
`endif

`endif

endmodule
