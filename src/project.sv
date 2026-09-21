/*
 * Copyright (c) 2026 Shashank
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 pin out:
 *   ui_in[7:0]  data byte 
 *   uo_out[0]   UART TX, idles high
 *   uo_out[1]   busy flag
 */
`default_nettype none

module tt_um_example (
    input  logic [7:0] ui_in,
    output logic [7:0] uo_out,
    input  logic [7:0] uio_in,
    output logic [7:0] uio_out,
    output logic [7:0] uio_oe,
    input  logic       ena,
    input  logic       clk,
    input  logic       rst_n
);

  // 50 MHz / 115200 = 434.
  localparam int unsigned DIVISOR = 434;

  typedef enum logic {IDLE, SEND} state_t;

  state_t      state;
  logic [15:0] baud_cnt;
  logic [3:0]  bit_idx;     // counts data bits 0..7, then 8 = stop bit
  logic [9:0]  shifter;     // {stop, data[7:0], start}
  logic        tx;
  logic        baud_tick;

  assign baud_tick = (baud_cnt == DIVISOR - 1);

  always_ff @(posedge clk) begin
    if (!rst_n) begin
      state    <= IDLE;
      baud_cnt <= '0;
      bit_idx  <= '0;
      shifter  <= '1;
      tx       <= 1'b1;
    end else begin
      baud_cnt <= baud_tick ? '0 : baud_cnt + 1'b1;

      if (baud_tick) begin
        unique case (state)
          IDLE: begin
            shifter <= {1'b1, ui_in, 1'b0};
            bit_idx <= '0;
            tx      <= 1'b0;          // start bit
            state   <= SEND;
          end
          SEND: begin
            tx      <= shifter[1];
            shifter <= {1'b1, shifter[9:1]};
            if (bit_idx == 4'd8) begin
              // next tick starts new frame
              bit_idx <= '0;
              state   <= IDLE;
            end else begin
              bit_idx <= bit_idx + 1'b1;
            end
          end
        endcase
      end
    end
  end

  assign uo_out  = {6'b0, (state == SEND), tx};
  assign uio_out = '0;
  assign uio_oe  = '0;

  logic _unused;
  assign _unused = &{ena, uio_in, 1'b0};

endmodule