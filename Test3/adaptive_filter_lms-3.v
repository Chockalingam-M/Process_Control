// ============================================================
//  LMS Adaptive Filter  –  RTL  (Fixed-point Q16)
//
//  NEW CONCEPT (corrected):
//    x_in  : NOISY input  = 1V/50Hz (clean) + 0.5V/2Hz (noise)  ← BIG signal
//    d_in  : DESIRED      = 1V/50Hz pure clean signal             ← reference
//    y_out : filter output → converges from noisy DOWN to clean
//    e_out : error = d_in - y_out  → shrinks to ~0 at convergence
//
//  Format  : Q16  (32-bit signed, 16 fractional bits)
//  Taps    : 64
//  Step µ  : 1 / 2^MU_SHIFT
// ============================================================
`timescale 1ns/1ps

module lms_adaptive_filter #(
    parameter  N_TAPS   = 64,
    parameter  DATA_W   = 32,
    parameter  MU_SHIFT = 11          // µ = 2^-11 ≈ 0.000488
)(
    input  wire                      clk,
    input  wire                      rst_n,
    input  wire signed [DATA_W-1:0]  x_in,   // noisy input  (large)
    input  wire signed [DATA_W-1:0]  d_in,   // desired clean signal (reference)
    output reg  signed [DATA_W-1:0]  y_out,  // filter output → converges to clean
    output reg  signed [DATA_W-1:0]  e_out,  // error = d - y  → shrinks to 0
    output reg                        valid
);

    reg signed [DATA_W-1:0] w     [0:N_TAPS-1];
    reg signed [DATA_W-1:0] x_buf [0:N_TAPS-1];

    integer idx;

    reg signed [63:0] acc;
    reg signed [31:0] y_reg;
    reg signed [31:0] e_reg;
    reg signed [63:0] wu;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (idx = 0; idx < N_TAPS; idx = idx + 1) begin
                w[idx]     <= 32'sd0;
                x_buf[idx] <= 32'sd0;
            end
            y_out <= 32'sd0;
            e_out <= 32'sd0;
            valid <= 1'b0;
        end else begin
            // 1) Shift delay line with NOISY input
            for (idx = N_TAPS-1; idx > 0; idx = idx - 1)
                x_buf[idx] <= x_buf[idx-1];
            x_buf[0] <= x_in;

            // 2) FIR output: y = Σ w[i]·x_buf[i]  (Q16 result)
            acc = 64'sd0;
            for (idx = 0; idx < N_TAPS; idx = idx + 1)
                acc = acc + ($signed(w[idx]) * $signed(x_buf[idx]));
            y_reg = acc[47:16];

            // 3) Error: e = desired_clean - y_out
            e_reg = d_in - y_reg;

            // 4) LMS update: w[i] += µ·e·x_buf[i]
            for (idx = 0; idx < N_TAPS; idx = idx + 1) begin
                wu     = ($signed(e_reg) * $signed(x_buf[idx]));
                w[idx] <= w[idx] + $signed(wu >>> (16 + MU_SHIFT));
            end

            y_out <= y_reg;
            e_out <= e_reg;
            valid <= 1'b1;
        end
    end

endmodule