/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Four-thread barrel core.
 *
 * A slot counter hands the fetch stage to thread 0, 1, 2, 3, 0, ... one per
 * clock. With four stages (F, M, D, E) and four threads, a thread never has
 * two instructions in flight: its next fetch is four cycles after its last,
 * and its previous instruction left E the cycle before. So there are no
 * data hazards, no forwarding, no branch prediction and no flushes, and
 * every thread runs at exactly clk/4. That determinism is the point: a
 * firmware loop's timing is a property of the program alone.
 *
 *   F  pc[slot] -> imem address.  Sleeping threads burn their slot here.
 *   M  the SRAM reads; its word (and ECC verdict) is registered.
 *   D  decode, register-file read, ready/WAIT/lock-step decisions.
 *   E  ALU, branch, IO, WAIT; commit or replay.
 *
 * M exists for timing. The SRAM's clock-to-output is 6.3 ns at the slow
 * corner, and with the word feeding decode directly, SRAM -> register-file
 * read mux -> lock-step compare was the critical path (14.9 ns of logic
 * before wires, at a 16.7 ns period). Four stages is also the most a
 * four-thread barrel takes without hazards: a thread's pc is written at
 * the end of E, exactly as its next fetch begins, and its register-file
 * write (one cycle after commit) lands the cycle before its next D.
 *
 * Blocking is replay: an IO access that is not ready, or a WAIT whose
 * condition is false, simply does not commit. The pc is unchanged, so the
 * same instruction is fetched again in the thread's next slot.
 *
 * Safe-side retiming (docs/theory.md section 15). Replay makes a false "not
 * ready" cost one round and nothing else, so ready and WAIT are decided in
 * D from registered state and carried into E in a flop, instead of being
 * computed in E from the IO block. The D-stage answer is cleared when the
 * instruction in E touches the same device that cycle; nothing outside the
 * core ever makes a device less ready, so the result is never wrongly
 * "ready". Barrel slot slack also lets the register-file write land one
 * cycle after commit: the thread's next D comes three cycles after its E.
 *
 * Lock-step pairs (radiation tolerance). With lockstep[p] set, thread p is
 * the master and thread p+2 its shadow: spawned and stopped together, they
 * run the same program on separate registers and pcs, the shadow exactly
 * two cycles behind. This is redundant multithreading (Reinhardt and
 * Mukherjee, ISCA 2000) with the barrel's fixed schedule replacing the
 * slack queues an out-of-order SMT core needs. The sphere of replication
 * is the core: inputs enter once, outputs leave once, after comparison.
 *
 *   master  performs IO reads and WAITs, records a signature of what it
 *           executed, and performs no IO writes or thread control.
 *   shadow  replays the instruction, takes IO read data from the master's
 *           signature, compares, and only on a match performs the write.
 *
 * The shadow compares the instruction's inputs (pc, word, operands), not
 * its results: that keeps the ALU out of the commit decision, which was the
 * critical path. Under a single upset, results can only differ if inputs
 * do, or if a transient hits the ALU itself; either way the divergence
 * surfaces as a different pc or operand at a later compare. Everything that
 * can reach a pin (IOW data and address, SPAWN/STOP operands) is an input.
 * So a single upset in core state (register file, pc, pipeline) is caught
 * before it reaches a pin. On a mismatch the pair halts and `fault` is
 * raised for the host to restart it. Not covered: the instruction memory
 * (both threads read the same word) and the lanes.
 *
 * The compare that gates the shadow's IO is made in D, from D's operands,
 * and carried into E in a flop: in E it sat between the heavily loaded
 * operand registers and every lane's write enable. The master's signature
 * cannot change in between: the master left E the cycle before the shadow
 * entered D, and the instruction in E meanwhile belongs to the other pair.
 * The same compare is repeated in E off the commit path; an upset that hits
 * the shadow's E-stage registers in that one cycle still raises `fault`,
 * a cycle after the write.
 */
`default_nettype none

module sh_core (
    input  logic                     clk,
    input  logic                     rst_n,

    // Instruction memory (shared port; the host may steal a cycle)
    output logic                     imem_en,
    output logic [sh_pkg::IMEM_AW-1:0]       imem_addr,
    input  logic [31:0]              imem_rdata,
    input  logic                     imem_busy,   // host owns the port this cycle
    input  logic                     imem_err,    // the word in M has an ECC error: cancel it

    // Thread control from the host port
    input  logic                     host_spawn,
    input  logic [1:0]               host_spawn_tid,
    input  logic [sh_pkg::IMEM_AW-1:0]       host_spawn_pc,
    input  logic [sh_pkg::NTHREADS-1:0]      host_stop,
    input  logic                     host_ls_wr,  // write lock-step config, clear faults
    input  logic [1:0]               host_ls,
    output logic [sh_pkg::NTHREADS-1:0]      running,
    output logic [1:0]               fault,
    output logic                     tmr_hit,

    // IO bus (combinational handshake in E)
    output logic                     io_valid,
    output logic                     io_we,
    output logic [7:0]               io_addr,
    output logic [sh_pkg::XLEN-1:0]          io_wdata,
    output logic [1:0]               io_tid,
    input  logic                     io_ready,   // soundness check only
    input  logic [sh_pkg::XLEN-1:0]          io_rdata,
    output logic [7:0]               peek_addr,  // ready query for the instruction in D
    output logic                     peek_we,
    input  logic                     peek_ready,

    // WAIT sources
    input  logic [sh_pkg::NLANES*sh_pkg::NEVENTS-1:0] lane_events,
    input  logic [23:0]              pins,
    output logic                     ev_clr_valid,
    output logic [1:0]               ev_clr_lane,
    output logic [sh_pkg::NEVENTS-1:0] ev_clr_mask,

    // Commit trace, for lock-step co-simulation and debug. Unused in silicon.
    output logic                     tr_valid,
    output logic [1:0]               tr_tid,
    output logic [sh_pkg::IMEM_AW-1:0]       tr_pc,
    output logic [31:0]              tr_word,
    output logic [sh_pkg::IMEM_AW-1:0]       tr_next_pc,
    output logic                     tr_rd_we,
    output logic [2:0]               tr_rd,
    output logic [sh_pkg::XLEN-1:0]          tr_wdata
);

  // ------------------------------------------------------------------------
  // Architectural state
  // ------------------------------------------------------------------------
  logic [sh_pkg::IMEM_AW-1:0] pc    [sh_pkg::NTHREADS];
  logic [sh_pkg::XLEN-1:0]    sleep [sh_pkg::NTHREADS];
  logic [sh_pkg::XLEN-1:0]    rf    [sh_pkg::NTHREADS * sh_pkg::NREGS];   // index {tid, reg}; reg 0 unused
  logic [1:0]         lockstep;

  // Spawning or stopping either thread of a pair spawns or stops both: a
  // master without its shadow (or the reverse) would wait in E forever.
  function automatic logic [sh_pkg::NTHREADS-1:0] mirror(input logic [sh_pkg::NTHREADS-1:0] m,
                                                 input logic [1:0] ls);
    mirror = m | {m[1:0] & ls, m[3:2] & ls};
  endfunction

  // ------------------------------------------------------------------------
  // Spawn / stop. Applied at the end of the cycle; they also kill any
  // instruction of that thread already in M, D or E, so the in-flight
  // instruction cannot overwrite the new pc.
  // ------------------------------------------------------------------------
  logic                    sw_spawn;          // from IOW SPAWN in E
  logic [1:0]              sw_spawn_tid;
  logic [sh_pkg::IMEM_AW-1:0]      sw_spawn_pc;
  logic [sh_pkg::NTHREADS-1:0]     sw_stop;
  logic [sh_pkg::NTHREADS-1:0]     host_spawn_m, sw_spawn_m, host_stop_m, sw_stop_m;
  logic [sh_pkg::NTHREADS-1:0]     kill_host, kill;

  assign host_spawn_m = mirror(host_spawn ? sh_pkg::NTHREADS'(1) << host_spawn_tid : '0, lockstep);
  assign sw_spawn_m   = mirror(sw_spawn ? sh_pkg::NTHREADS'(1) << sw_spawn_tid : '0, lockstep);
  assign host_stop_m  = mirror(host_stop, lockstep);
  assign sw_stop_m    = mirror(sw_stop, lockstep);

  // E only needs killing by the host: a SPAWN/STOP in E *is* the instruction
  // in E, and its own thread-control write lands after its commit anyway.
  assign kill_host = host_stop_m | host_spawn_m;
  assign kill      = kill_host | sw_stop_m | sw_spawn_m;

  // ------------------------------------------------------------------------
  // F
  // ------------------------------------------------------------------------
  logic [1:0] slot;
  logic       f_issue;

  assign f_issue   = running[slot] && (sleep[slot] == '0) && !imem_busy && !kill[slot];
  assign imem_en   = f_issue;
  assign imem_addr = pc[slot];

  // ------------------------------------------------------------------------
  // M: the SRAM is reading for the instruction fetched last cycle.
  // ------------------------------------------------------------------------
  logic               m_valid;
  logic [1:0]         m_tid;
  logic [sh_pkg::IMEM_AW-1:0] m_pc;

  logic               d_valid;
  logic [1:0]         d_tid;
  logic [sh_pkg::IMEM_AW-1:0] d_pc;

  // ------------------------------------------------------------------------
  // D
  // ------------------------------------------------------------------------
  logic [31:0] d_word;                 // registered at the end of M
  logic [5:0]  d_op;
  logic [2:0]  d_rd, d_rs, d_rt;
  logic        d_i;
  logic [15:0] d_imm;
  logic [sh_pkg::XLEN-1:0] d_a, d_rtv;

  assign d_op   = d_word[31:26];
  assign d_rd   = d_word[25:23];
  assign d_rs   = d_word[22:20];
  assign d_rt   = d_word[19:17];
  assign d_i    = d_word[16];
  assign d_imm  = d_word[15:0];
  assign d_a    = (d_rs == 3'd0) ? '0 : rf[{d_tid, d_rs}];
  assign d_rtv  = (d_rt == 3'd0) ? '0 : rf[{d_tid, d_rt}];

  // Signature: what the master executed, waiting for its shadow.
  logic               sig_valid [2];
  logic [sh_pkg::IMEM_AW-1:0] sig_pc    [2];
  logic [31:0]        sig_word  [2];
  logic [sh_pkg::XLEN-1:0]    sig_a     [2];
  logic [sh_pkg::XLEN-1:0]    sig_b     [2];
  logic [sh_pkg::XLEN-1:0]    sig_val   [2];   // IO read data, replayed to the shadow's IOR

  // Lock-step compare for a shadow in D (see the header): the signature of
  // its master, which has already left E.
  logic d_sig_ok;
  assign d_sig_ok = sig_pc[d_tid[0]] == d_pc && sig_word[d_tid[0]] == d_word
                    && sig_a[d_tid[0]] == d_a && sig_b[d_tid[0]] == (d_i ? d_imm : d_rtv);

  assign peek_addr = d_imm[7:0];
  assign peek_we   = (d_op == sh_pkg::OP_IOW);

  // Device of an IO address: lanes 0..3, the three channels, then the
  // packet buffer's read side (a PB_RP write empties its prefetch register
  // as a PB_DATA read does). Other addresses are always ready.
  function automatic logic [3:0] io_dev(input logic [7:0] a);
    if (a >= sh_pkg::IO_LANE_BASE && a < sh_pkg::IO_LANE_BASE + 8'(sh_pkg::NLANES << sh_pkg::LANE_SHIFT))
      io_dev = {2'b10, 2'((a - sh_pkg::IO_LANE_BASE) >> sh_pkg::LANE_SHIFT)};
    else if (a == sh_pkg::IO_CH_HOST) io_dev = 4'b1100;
    else if (a == sh_pkg::IO_CH_A)    io_dev = 4'b1101;
    else if (a == sh_pkg::IO_CH_B)    io_dev = 4'b1110;
    else if (a == sh_pkg::IO_PB_DATA || a == sh_pkg::IO_PB_RP) io_dev = 4'b1111;
    else                              io_dev = 4'b0000;
  endfunction

  // What the instruction in E consumes this cycle (only the core consumes).
  logic [3:0] e_dev_used;
  logic [3:0] d_dev, d_wait_dev;
  logic       d_ready, d_wait;
  assign d_dev      = io_dev(d_imm[7:0]);
  assign d_wait_dev = {2'b10, d_imm[9:8]};
  // A WAIT clear can lower a lane's tx_free too (MISMATCH ends an ABORT halt).
  assign d_ready = peek_ready && !(d_dev[3] && e_dev_used == d_dev)
                   && !(ev_clr_valid && d_dev == {2'b10, ev_clr_lane});
  assign d_wait  = d_imm[15] ? ((d_imm[4:0] < 5'd24 ? pins[d_imm[4:0]] : 1'b1) == d_imm[5])
                             : (|(lane_events[d_imm[9:8]*sh_pkg::NEVENTS +: sh_pkg::NEVENTS] & {d_imm[sh_pkg::WAIT_EV_HI +: 2], d_imm[7:0]}) && e_dev_used != d_wait_dev
                                && !(ev_clr_valid && ev_clr_lane == d_imm[9:8]));

  logic               e_valid;
  logic [1:0]         e_tid;
  logic [sh_pkg::IMEM_AW-1:0] e_pc;
  logic [31:0]        e_word;
  logic [5:0]         e_op;
  logic [2:0]         e_rd;
  logic [15:0]        e_imm;
  logic [sh_pkg::XLEN-1:0]    e_a, e_b, e_rtv;
  logic               e_ready, e_wait;   // decided in D
  logic               e_sig_ok;          // lock-step compare, made in D

  // ------------------------------------------------------------------------
  // E
  // ------------------------------------------------------------------------
  logic [sh_pkg::IMEM_AW-1:0] e_target, e_seq;
  assign e_target = e_imm[sh_pkg::IMEM_AW-1:0];
  assign e_seq    = e_pc + 1'b1;

  // Lock-step role of the thread in E
  logic       pair;
  logic       is_master, is_shadow;
  assign pair      = e_tid[0];
  assign is_master = lockstep[pair] && !e_tid[1];
  assign is_shadow = lockstep[pair] &&  e_tid[1];

  // ALU
  logic [sh_pkg::XLEN-1:0] alu;
  always_comb begin
    unique case (e_op)
      sh_pkg::OP_ADD:  alu = e_a + e_b;
      sh_pkg::OP_SUB:  alu = e_a - e_b;
      sh_pkg::OP_AND:  alu = e_a & e_b;
      sh_pkg::OP_OR:   alu = e_a | e_b;
      sh_pkg::OP_XOR:  alu = e_a ^ e_b;
      sh_pkg::OP_SHL:  alu = e_b[4] ? '0 : (e_a << e_b[3:0]);
      sh_pkg::OP_SHR:  alu = e_b[4] ? '0 : (e_a >> e_b[3:0]);
      sh_pkg::OP_PAR:  alu = {{(sh_pkg::XLEN-1){1'b0}}, ^(e_a & e_b)};
      sh_pkg::OP_SLTU: alu = {{(sh_pkg::XLEN-1){1'b0}}, (e_a < e_b)};
      default: alu = '0;
    endcase
  end

  logic [sh_pkg::XLEN-1:0] djnz_val;
  assign djnz_val = e_a - 1'b1;

  logic bit_sel;
  assign bit_sel = e_a[e_imm[15:12]];

  // Next pc and register write. core_val is the result of everything except IOR.
  logic [sh_pkg::IMEM_AW-1:0] next_pc;
  logic               rd_we;
  logic [sh_pkg::XLEN-1:0]    core_val, rd_val;
  always_comb begin
    next_pc  = e_seq;
    rd_we    = 1'b0;
    core_val = alu;
    unique case (e_op)
      sh_pkg::OP_ADD, sh_pkg::OP_SUB, sh_pkg::OP_AND, sh_pkg::OP_OR, sh_pkg::OP_XOR,
      sh_pkg::OP_SHL, sh_pkg::OP_SHR, sh_pkg::OP_PAR, sh_pkg::OP_SLTU: rd_we = 1'b1;
      sh_pkg::OP_JMP:  next_pc = e_target;
      sh_pkg::OP_BEQ:  if (e_a == e_rtv) next_pc = e_target;
      sh_pkg::OP_BNE:  if (e_a != e_rtv) next_pc = e_target;
      sh_pkg::OP_BLTU: if (e_a <  e_rtv) next_pc = e_target;
      sh_pkg::OP_BGEU: if (e_a >= e_rtv) next_pc = e_target;
      sh_pkg::OP_BBS:  if (bit_sel)      next_pc = e_target;
      sh_pkg::OP_BBC:  if (!bit_sel)     next_pc = e_target;
      sh_pkg::OP_DJNZ: begin
        rd_we    = 1'b1;
        core_val = djnz_val;
        if (djnz_val != '0) next_pc = e_target;
      end
      sh_pkg::OP_JAL: begin
        rd_we    = 1'b1;
        core_val = {{(sh_pkg::XLEN-sh_pkg::IMEM_AW){1'b0}}, e_seq};
        next_pc  = e_target;
      end
      sh_pkg::OP_JR:   next_pc = e_a[sh_pkg::IMEM_AW-1:0];
      sh_pkg::OP_IOR:  rd_we = 1'b1;
      sh_pkg::OP_HALT: next_pc = e_pc;
      default: ;
    endcase
  end

  logic is_ior, is_iow, is_ctl;
  assign is_ior = (e_op == sh_pkg::OP_IOR);
  assign is_iow = (e_op == sh_pkg::OP_IOW);
  assign is_ctl = is_iow && (e_imm[7:0] == sh_pkg::IO_SPAWN || e_imm[7:0] == sh_pkg::IO_STOP);

  assign rd_val = !is_ior ? core_val : (is_shadow ? sig_val[pair] : io_rdata);

  // Shadow compare repeated in E. It only raises a fault (late_fault, a
  // cycle after the write); the commit decision uses e_sig_ok from D.
  logic sig_match;
  assign sig_match = sig_pc[pair] == e_pc && sig_word[pair] == e_word
                     && sig_a[pair] == e_a && sig_b[pair] == e_b;

  logic ls_wait, ls_fault;
  assign ls_wait  = (is_master && sig_valid[pair]) || (is_shadow && !sig_valid[pair]);
  // A waiting shadow and its master must be at the same pc: the master is
  // replaying the instruction the shadow is about to execute. Checking that
  // every round bounds detection latency even when an upset sends the
  // master to an instruction that blocks forever.
  assign ls_fault = e_valid && !kill_host[e_tid] && is_shadow
                    && (sig_valid[pair] ? !e_sig_ok : (pc[{1'b0, pair}] != e_pc));

  logic late_fault;
  assign late_fault = e_valid && !kill_host[e_tid] && is_shadow && sig_valid[pair] && !sig_match;

  // IO. Masters read, shadows write, unpaired threads do both. SPAWN and
  // STOP are thread control, so the core handles them itself.
  logic io_go, io_access;
  assign io_go     = e_valid && !kill_host[e_tid] && !ls_wait && !ls_fault;
  assign io_access = io_go && !is_ctl && ((is_ior && !is_shadow) || (is_iow && !is_master));
  assign io_valid  = io_access && e_ready;
  assign e_dev_used = io_valid ? io_dev(e_imm[7:0]) : 4'b0000;
  assign io_we    = is_iow;
  assign io_addr  = e_imm[7:0];
  assign io_wdata = e_a;
  assign io_tid   = e_tid;

  // WAIT (the shadow trusts the master's WAIT)
  logic       wait_pin;
  logic       wait_met;
  assign wait_pin = e_imm[15];
  assign wait_met = is_shadow || e_wait;

  logic blocked, commit;
  assign blocked = ls_wait || ls_fault || (io_access && !e_ready) || (e_op == sh_pkg::OP_WAIT && !wait_met);
  assign commit  = e_valid && !kill_host[e_tid] && !blocked;

  assign ev_clr_valid = commit && (e_op == sh_pkg::OP_WAIT) && !wait_pin && !is_shadow;
  assign ev_clr_lane  = e_imm[9:8];
  assign ev_clr_mask  = {e_imm[sh_pkg::WAIT_EV_HI +: 2], e_imm[7:0]} & sh_pkg::EV_STICKY;

  assign sw_spawn     = commit && is_ctl && !is_master && (e_imm[7:0] == sh_pkg::IO_SPAWN);
  assign sw_spawn_tid = e_a[10:9];
  assign sw_spawn_pc  = e_a[sh_pkg::IMEM_AW-1:0];
  assign sw_stop      = (commit && is_ctl && !is_master && (e_imm[7:0] == sh_pkg::IO_STOP)) ? e_a[sh_pkg::NTHREADS-1:0] : '0;

  // Trace
  assign tr_valid   = commit;
  assign tr_tid     = e_tid;
  assign tr_pc      = e_pc;
  assign tr_word    = e_word;
  assign tr_next_pc = next_pc;
  assign tr_rd_we   = rd_we && (e_rd != 3'd0);
  assign tr_rd      = e_rd;
  assign tr_wdata   = rd_val;

  // Register-file write-back, one cycle after commit (slot slack).
  logic                        wb_valid;
  logic [4:0]                  wb_idx;
  logic [sh_pkg::XLEN-1:0]     wb_val;

`ifndef SYNTHESIS
  // Soundness of the D-stage ready decision: the IO block must agree.
  always_ff @(posedge clk)
    if (rst_n && io_valid && !io_ready) $error("sh_core: issued an IO access the IO block was not ready for");
`endif

  // ------------------------------------------------------------------------
  // Thread control: which threads run, which pairs are in lock step, which
  // pairs have faulted. An upset here would start a stopped thread at a
  // stale pc or silently drop lock-step, so it is held in self-correcting
  // TMR (8 bits: narrower than an ECC register pays for, see sh_ecr.sv).
  // Next state in one place, in the order that decides priority: halt
  // and divergence, then stop and spawn, then the host's lock-step write.
  // ------------------------------------------------------------------------
  logic [sh_pkg::NTHREADS-1:0] run_n;
  logic [1:0]                  ls_n, fault_n;
  always_comb begin
    run_n   = running;
    ls_n    = lockstep;
    fault_n = fault;
    if (commit && e_op == sh_pkg::OP_HALT) run_n[e_tid] = 1'b0;
    if (ls_fault || late_fault) begin
      fault_n[pair]       = 1'b1;
      run_n[{1'b0, pair}] = 1'b0;
      run_n[{1'b1, pair}] = 1'b0;
    end
    for (int t = 0; t < sh_pkg::NTHREADS; t++) begin
      if (sw_stop_m[t] || host_stop_m[t])   run_n[t] = 1'b0;
      if (sw_spawn_m[t] || host_spawn_m[t]) run_n[t] = 1'b1;
    end
    if (host_ls_wr) begin
      ls_n    = host_ls;
      fault_n = '0;
    end
  end

  sh_tmr #(.W(sh_pkg::NTHREADS + 4)) u_ctl (
      .clk, .rst_n, .en(1'b1),
      .d({fault_n, ls_n, run_n}), .q({fault, lockstep, running}), .hit(tmr_hit));

  // ------------------------------------------------------------------------
  // Sequential
  // ------------------------------------------------------------------------
  always_ff @(posedge clk) begin
    if (!rst_n) begin
      slot     <= '0;
      m_valid  <= 1'b0;
      d_valid  <= 1'b0;
      e_valid  <= 1'b0;
      for (int t = 0; t < sh_pkg::NTHREADS; t++) begin
        pc[t]    <= '0;
        sleep[t] <= '0;
      end
      for (int p = 0; p < 2; p++) begin
        sig_valid[p] <= 1'b0;
        sig_pc[p]    <= '0;
        sig_word[p]  <= '0;
        sig_a[p]     <= '0;
        sig_b[p]     <= '0;
        sig_val[p]   <= '0;
      end
      // Flops power up random. Clearing the register file costs ~2,000 um^2
      // and makes every run reproducible, which co-simulation depends on.
      for (int r = 0; r < sh_pkg::NTHREADS * sh_pkg::NREGS; r++) rf[r] <= '0;
      wb_valid <= 1'b0;
    end else begin
      slot <= slot + 1'b1;

      wb_valid <= commit && rd_we && e_rd != 3'd0;
      wb_idx   <= {e_tid, e_rd};
      wb_val   <= rd_val;
      if (wb_valid) rf[wb_idx] <= wb_val;
      // A spawned thread starts with its registers at zero, as after reset.
      // For a lock-step pair this is what makes a restart sound: a stop that
      // lands between the master's commit and its shadow's (a 2-cycle window
      // every round), or a fault, leaves their registers different, and the
      // first compare after the restart would fault again. Clearing wins
      // over a write-back still landing for the old program.
      for (int t = 0; t < sh_pkg::NTHREADS; t++)
        if (sw_spawn_m[t] || host_spawn_m[t])
          for (int r = 1; r < sh_pkg::NREGS; r++) rf[t * sh_pkg::NREGS + r] <= '0;

      // F -> M
      m_valid <= f_issue;
      m_tid   <= slot;
      m_pc    <= pc[slot];
      if (running[slot] && sleep[slot] != '0) sleep[slot] <= sleep[slot] - 1'b1;

      // M -> D
      // An ECC error cancels the instruction here, before anything commits;
      // its pc does not move, so the thread refetches it (from the fix buffer).
      d_valid <= m_valid && !kill[m_tid] && !imem_err;
      d_tid   <= m_tid;
      d_pc    <= m_pc;
      d_word  <= imem_rdata;

      // D -> E
      e_valid <= d_valid && !kill[d_tid];
      e_tid   <= d_tid;
      e_pc    <= d_pc;
      e_word  <= d_word;
      e_op    <= d_op;
      e_rd    <= d_rd;
      e_imm   <= d_imm;
      e_a     <= d_a;
      e_rtv   <= d_rtv;
      e_b     <= d_i ? d_imm : d_rtv;
      e_ready <= d_ready;
      e_wait  <= d_wait;
      e_sig_ok <= d_sig_ok;

      // E: commit
      if (commit) begin
        pc[e_tid] <= next_pc;
        if (e_op == sh_pkg::OP_DLY)  sleep[e_tid] <= e_b;
        if (is_master) begin
          sig_valid[pair] <= 1'b1;
          sig_pc[pair]    <= e_pc;
          sig_word[pair]  <= e_word;
          sig_a[pair]     <= e_a;
          sig_b[pair]     <= e_b;
          sig_val[pair]   <= io_rdata;
        end
        if (is_shadow) sig_valid[pair] <= 1'b0;
      end

      // Divergence: the pair halts (thread control above), flag the host.
      if (ls_fault || late_fault) sig_valid[pair] <= 1'b0;

      // Thread control last, so it wins over the commit above.
      for (int t = 0; t < sh_pkg::NTHREADS; t++) begin
        if (sw_spawn_m[t]) begin
          pc[t]    <= sw_spawn_pc;
          sleep[t] <= '0;
        end
        if (host_spawn_m[t]) begin
          pc[t]    <= host_spawn_pc;
          sleep[t] <= '0;
        end
      end
      for (int p = 0; p < 2; p++) begin
        if (host_spawn_m[p] || sw_spawn_m[p] || host_stop_m[p] || sw_stop_m[p]) sig_valid[p] <= 1'b0;
      end
    end
  end

endmodule
