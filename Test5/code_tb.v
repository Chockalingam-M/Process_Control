// ============================================================
// TESTBENCH
// PIPELINED LMS ADAPTIVE FILTER
// Spartan-6 Q16 Verification
//
// x_in = clean + noise
// d_in = clean reference
//
// Expected:
// y_out -> converges toward clean signal
// e_out -> converges toward 0
// ============================================================

`timescale 1ns/1ps

module lms_adaptive_filter_pipeline_tb;

// ============================================================
// PARAMETERS
// ============================================================

localparam DATA_W   = 32;
localparam N_TAPS   = 64;
localparam MU_SHIFT = 11;

localparam real Q16 = 65536.0;

localparam real FS  = 10000.0;
localparam real TS  = 1.0 / FS;

localparam real PI  = 3.14159265358979;

localparam integer N_SAMPLES = 10000;

// ============================================================
// SIGNALS
// ============================================================

reg clk;
reg rst_n;

reg signed [DATA_W-1:0] x_in;
reg signed [DATA_W-1:0] d_in;

wire signed [DATA_W-1:0] y_out;
wire signed [DATA_W-1:0] e_out;

wire valid;

// ============================================================
// DUT
// ============================================================

lms_adaptive_filter_pipeline #(
    .N_TAPS(N_TAPS),
    .DATA_W(DATA_W),
    .MU_SHIFT(MU_SHIFT)
)
DUT
(
    .clk(clk),
    .rst_n(rst_n),

    .x_in(x_in),
    .d_in(d_in),

    .y_out(y_out),
    .e_out(e_out),

    .valid(valid)
);

// ============================================================
// CLOCK
// 50 MHz
// ============================================================

always #10 clk = ~clk;

// ============================================================
// Q16 FUNCTIONS
// ============================================================

function signed [31:0] real_to_q16;
    input real val;
    begin
        real_to_q16 = $rtoi(val * Q16);
    end
endfunction

function real q16_to_real;
    input signed [31:0] val;
    begin
        q16_to_real = $itor(val) / Q16;
    end
endfunction

// ============================================================
// VARIABLES
// ============================================================

integer fd;
integer n;

real t;

real clean_signal;
real noise_signal;
real noisy_signal;

real mse;
real rms_error;
real error_real;

real peak_error;

// ============================================================
// INITIALIZATION
// ============================================================

initial begin

    clk   = 0;
    rst_n = 0;

    x_in  = 0;
    d_in  = 0;

    mse = 0;
    peak_error = 0;

    // ========================================================
    // CSV FILE
    // ========================================================

    fd = $fopen("lms_pipeline_output.csv","w");

    $fdisplay(fd,
    "sample,time_s,x_in_V,d_in_V,y_out_V,e_out_V");

    // ========================================================
    // RESET
    // ========================================================

    repeat(10)
        @(posedge clk);

    rst_n = 1;

    @(posedge clk);

    // ========================================================
    // MAIN SIMULATION
    // ========================================================

    for(n = 0; n < N_SAMPLES; n = n + 1) begin

        t = n * TS;

        // ----------------------------------------------------
        // CLEAN SIGNAL
        // 1V @ 50Hz
        // ----------------------------------------------------

        clean_signal =
            1.0 *
            $sin(2.0 * PI * 50.0 * t);

        // ----------------------------------------------------
        // NOISE SIGNAL
        // 0.5V @ 2Hz
        // ----------------------------------------------------

        noise_signal =
            0.5 *
            $sin(2.0 * PI * 2.0 * t);

        // ----------------------------------------------------
        // NOISY INPUT
        // ----------------------------------------------------

        noisy_signal =
            clean_signal + noise_signal;

        // ----------------------------------------------------
        // APPLY INPUTS
        // ----------------------------------------------------

        x_in = real_to_q16(noisy_signal);

        d_in = real_to_q16(clean_signal);

        @(posedge clk);

        // ----------------------------------------------------
        // ERROR ANALYSIS
        // ----------------------------------------------------

        error_real =
            q16_to_real(e_out);

        mse =
            mse + (error_real * error_real);

        if(error_real > peak_error)
            peak_error = error_real;

        // ----------------------------------------------------
        // CSV OUTPUT
        // ----------------------------------------------------

        $fdisplay(fd,
        "%0d,%f,%f,%f,%f,%f",
        n,
        t,
        q16_to_real(x_in),
        q16_to_real(d_in),
        q16_to_real(y_out),
        q16_to_real(e_out));

    end

    // ========================================================
    // FINAL REPORT
    // ========================================================

    mse = mse / N_SAMPLES;

    rms_error = $sqrt(mse);

    $display("");
    $display("==========================================");
    $display(" PIPELINED LMS FILTER RESULTS");
    $display("==========================================");
    $display("Total Samples  : %0d", N_SAMPLES);
    $display("Mean Sq Error  : %f", mse);
    $display("RMS Error      : %f", rms_error);
    $display("Peak Error     : %f", peak_error);
    $display("==========================================");

    if(rms_error < 0.05)
        $display("FILTER CONVERGED SUCCESSFULLY");
    else
        $display("FILTER DID NOT FULLY CONVERGE");

    $display("==========================================");

    // ========================================================
    // CLOSE FILE
    // ========================================================

    $fclose(fd);

    $finish;
end

endmodule