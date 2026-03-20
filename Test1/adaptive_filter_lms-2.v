// ============================================================================
//  MODULE      : adaptive_filter_lms
//  DESCRIPTION : 16-Tap LMS (Least Mean Squares) Adaptive FIR Filter
//                Designed for FPGA synthesis (Xilinx/Intel compatible)
//
//  ALGORITHM   : w(n+1) = w(n) + 2*mu*e(n)*x(n)
//                y(n)   = w(n)^T * x(n)
//                e(n)   = d(n) - y(n)
//
//  PARAMETERS
//  ----------
//  DATA_WIDTH  : Bit-width of input samples          (default 16)
//  COEFF_WIDTH : Bit-width of filter coefficients    (default 24)
//  ACC_WIDTH   : Accumulator width for MAC            (default 48)
//  MU_SHIFT    : Step-size as right-shift amount (mu = 2^-MU_SHIFT)
//
//  PORTS
//  -----
//  clk         : System clock (rising-edge)
//  rst_n       : Active-low synchronous reset
//  en          : Clock enable / data valid
//  x_in        : Input signal sample  [DATA_WIDTH-1:0]
//  d_in        : Desired signal sample [DATA_WIDTH-1:0]
//  y_out       : Filter output         [DATA_WIDTH-1:0]
//  e_out       : Error signal          [DATA_WIDTH-1:0]
//  coeff_out   : Current coefficient tap 0 (debug/monitor)
//
//  LATENCY     : 3 clock cycles (sample_reg → MAC → output_reg)
//
//  AUTHOR      : Professional VLSI Design  
//  REVISION    : 1.0 — Initial Release
// ============================================================================

`timescale 1ns / 1ps

module adaptive_filter_lms #(
    parameter TAPS        = 16,
    parameter DATA_WIDTH  = 16,
    parameter COEFF_WIDTH = 24,
    parameter ACC_WIDTH   = 48,
    parameter MU_SHIFT    = 10   // step-size mu = 2^(-MU_SHIFT)
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   en,
    input  wire [DATA_WIDTH-1:0]  x_in,
    input  wire [DATA_WIDTH-1:0]  d_in,
    output reg  [DATA_WIDTH-1:0]  y_out,
    output reg  [DATA_WIDTH-1:0]  e_out,
    output wire [COEFF_WIDTH-1:0] coeff_out   // tap-0 monitor port
);

    // -------------------------------------------------------------------------
    //  Internal signal declarations
    // -------------------------------------------------------------------------

    // Delay-line (shift register) for input samples
    reg signed [DATA_WIDTH-1:0]  x_delay [0:TAPS-1];

    // Coefficient registers (signed fixed-point, Q8.16 format by default)
    reg signed [COEFF_WIDTH-1:0] w [0:TAPS-1];

    // Pipeline stage 1 — products  x[k]*w[k]
    reg signed [DATA_WIDTH+COEFF_WIDTH-1:0] prod [0:TAPS-1];

    // Pipeline stage 2 — accumulator
    reg signed [ACC_WIDTH-1:0]   acc;

    // Error and update signals
    reg signed [DATA_WIDTH-1:0]  error_reg;
    reg signed [DATA_WIDTH-1:0]  d_delay;       // d(n) aligned to filter latency

    // Coefficient update products  e(n)*x[k]
    reg signed [DATA_WIDTH*2-1:0] update_prod [0:TAPS-1];

    // Tap index loop variable
    integer k;

    // -------------------------------------------------------------------------
    //  Monitor / debug port — expose tap-0 coefficient
    // -------------------------------------------------------------------------
    assign coeff_out = w[0];

    // =========================================================================
    //  STAGE 1 : Input sample shift register  (delay line)
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < TAPS; k = k + 1)
                x_delay[k] <= {DATA_WIDTH{1'b0}};
        end else if (en) begin
            x_delay[0] <= $signed(x_in);
            for (k = 1; k < TAPS; k = k + 1)
                x_delay[k] <= x_delay[k-1];
        end
    end

    // =========================================================================
    //  STAGE 2 : Multiply-Accumulate  (FIR convolution)
    //            y(n) = sum_{k=0}^{N-1} w[k] * x[n-k]
    // =========================================================================

    // --- Registered multiplier array (parallel MACs) -------------------------
    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < TAPS; k = k + 1)
                prod[k] <= {(DATA_WIDTH+COEFF_WIDTH){1'b0}};
            d_delay <= {DATA_WIDTH{1'b0}};
        end else if (en) begin
            d_delay <= $signed(d_in);
            for (k = 0; k < TAPS; k = k + 1)
                prod[k] <= $signed(w[k]) * $signed(x_delay[k]);
        end
    end

    // --- Accumulation tree (single-cycle adder chain — let synth build tree) -
    always @(posedge clk) begin
        if (!rst_n) begin
            acc <= {ACC_WIDTH{1'b0}};
        end else if (en) begin
            acc <= prod[0]  + prod[1]  + prod[2]  + prod[3]
                 + prod[4]  + prod[5]  + prod[6]  + prod[7]
                 + prod[8]  + prod[9]  + prod[10] + prod[11]
                 + prod[12] + prod[13] + prod[14] + prod[15];
        end
    end

    // =========================================================================
    //  STAGE 3 : Output saturation + error computation
    //            y_out = truncate(acc)
    //            e(n)  = d(n) - y(n)
    // =========================================================================

    // Truncate accumulator (drop COEFF_WIDTH-DATA_WIDTH fraction bits)
    // Coefficients are in Q(COEFF_WIDTH-DATA_WIDTH).DATA_WIDTH format
    localparam FRAC_BITS = COEFF_WIDTH - DATA_WIDTH;

    wire signed [DATA_WIDTH-1:0] y_trunc;
    // Arithmetic right-shift removes fraction, keep DATA_WIDTH MSBs
    assign y_trunc = acc[FRAC_BITS + DATA_WIDTH - 1 : FRAC_BITS];

    always @(posedge clk) begin
        if (!rst_n) begin
            y_out     <= {DATA_WIDTH{1'b0}};
            e_out     <= {DATA_WIDTH{1'b0}};
            error_reg <= {DATA_WIDTH{1'b0}};
        end else if (en) begin
            y_out     <= y_trunc;
            error_reg <= $signed(d_delay) - $signed(y_trunc);
            e_out     <= $signed(d_delay) - $signed(y_trunc);
        end
    end

    // =========================================================================
    //  STAGE 4 : LMS Coefficient Update
    //            w[k](n+1) = w[k](n) + mu * e(n) * x[n-k]
    //            mu = 2^(-MU_SHIFT)  → implemented as arithmetic right-shift
    //
    //  NOTE: update_prod is 2*DATA_WIDTH = 32 bits, COEFF_WIDTH = 24 bits.
    //        Since 32 > 24 we TRUNCATE (keep lower COEFF_WIDTH bits after
    //        the right-shift), which is safe because MU_SHIFT already scales
    //        the step down so the MSBs are redundant sign bits.
    // =========================================================================

    // Signed delta before the coefficient addition  (COEFF_WIDTH wide)
    wire signed [COEFF_WIDTH-1:0] lms_delta [0:TAPS-1];

    genvar gi;
    generate
        for (gi = 0; gi < TAPS; gi = gi + 1) begin : gen_delta
            // Arithmetic right-shift the 32-bit product by MU_SHIFT,
            // then slice the lower COEFF_WIDTH bits — always safe because
            // MU_SHIFT >= 8 guarantees the top 8 bits are pure sign extension.
            assign lms_delta[gi] =
                $signed(update_prod[gi]) >>> MU_SHIFT;
        end
    endgenerate

    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < TAPS; k = k + 1) begin
                w[k]           <= {COEFF_WIDTH{1'b0}};
                update_prod[k] <= {(DATA_WIDTH*2){1'b0}};
            end
        end else if (en) begin
            for (k = 0; k < TAPS; k = k + 1) begin
                // Compute  e(n) * x[n-k]  (registered one cycle)
                update_prod[k] <= $signed(error_reg) * $signed(x_delay[k]);

                // w[k] += truncate( (e * x[k]) >> MU_SHIFT )
                w[k] <= $signed(w[k]) + lms_delta[k];
            end
        end
    end

endmodule
