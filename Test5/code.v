// ============================================================
// PIPELINED LMS ADAPTIVE FILTER
// Based on Original LMS Structure
// Spartan-6 Optimized
// Q16 Fixed Point
// ============================================================

`timescale 1ns/1ps

module lms_adaptive_filter_pipeline #(
    parameter N_TAPS   = 64,
    parameter DATA_W   = 32,
    parameter MU_SHIFT = 11
)(
    input  wire                      clk,
    input  wire                      rst_n,

    input  wire signed [DATA_W-1:0] x_in,
    input  wire signed [DATA_W-1:0] d_in,

    output reg signed [DATA_W-1:0] y_out,
    output reg signed [DATA_W-1:0] e_out,

    output reg valid
);

integer i;

// ============================================================
// MEMORY
// ============================================================

reg signed [DATA_W-1:0] w     [0:N_TAPS-1];
reg signed [DATA_W-1:0] x_buf [0:N_TAPS-1];

// ============================================================
// STAGE 1
// INPUT SHIFT REGISTER
// ============================================================

reg signed [DATA_W-1:0] x_s1 [0:N_TAPS-1];
reg signed [DATA_W-1:0] d_s1;

always @(posedge clk or negedge rst_n) begin

    if(!rst_n) begin

        for(i=0;i<N_TAPS;i=i+1) begin
            x_buf[i] <= 0;
            x_s1[i]  <= 0;
        end

        d_s1 <= 0;
    end

    else begin

        // shift delay line

        x_buf[0] <= x_in;

        for(i=1;i<N_TAPS;i=i+1)
            x_buf[i] <= x_buf[i-1];

        // register stage

        for(i=0;i<N_TAPS;i=i+1)
            x_s1[i] <= x_buf[i];

        d_s1 <= d_in;
    end
end

// ============================================================
// STAGE 2
// PARALLEL MULTIPLIERS
// ============================================================

reg signed [63:0] mult_s2 [0:N_TAPS-1];
reg signed [DATA_W-1:0] d_s2;

always @(posedge clk) begin

    for(i=0;i<N_TAPS;i=i+1)
        mult_s2[i] <=
            $signed(w[i]) * $signed(x_s1[i]);

    d_s2 <= d_s1;
end

// ============================================================
// STAGE 3
// FIR ACCUMULATION
// ============================================================

reg signed [63:0] acc_s3;
reg signed [DATA_W-1:0] y_s3;
reg signed [DATA_W-1:0] d_s3;

integer k;

always @(posedge clk) begin

    acc_s3 = 0;

    for(k=0;k<N_TAPS;k=k+1)
        acc_s3 = acc_s3 + mult_s2[k];

    y_s3 <= acc_s3[47:16];

    d_s3 <= d_s2;
end

// ============================================================
// STAGE 4
// ERROR CALCULATION
// ============================================================

reg signed [DATA_W-1:0] e_s4;
reg signed [DATA_W-1:0] y_s4;

always @(posedge clk) begin

    e_s4 <= d_s3 - y_s3;

    y_s4 <= y_s3;
end

// ============================================================
// STAGE 5
// WEIGHT UPDATE
// ============================================================

reg signed [63:0] wu;

always @(posedge clk or negedge rst_n) begin

    if(!rst_n) begin

        for(i=0;i<N_TAPS;i=i+1)
            w[i] <= 0;

        y_out <= 0;
        e_out <= 0;
        valid <= 0;
    end

    else begin

        for(i=0;i<N_TAPS;i=i+1) begin

            wu =
                $signed(e_s4) *
                $signed(x_s1[i]);

            w[i] <=
                w[i] +
                $signed(wu >>> (16 + MU_SHIFT));
        end

        y_out <= y_s4;
        e_out <= e_s4;

        valid <= 1'b1;
    end
end

endmodule



// output 

// =====================================
// FINAL FPGA RESULTS
//=====================================
//MSE                : 0.021509
//RMS Error          : 0.146660
//Correlation        : 0.981519
//Output SNR         : 14.094357 dB
//Estimated Delay    : 0.000100 sec
//=====================================

//Excellent convergence achieved.
//High residual error detected.

//FPGA LMS analysis completed.
//====================================

