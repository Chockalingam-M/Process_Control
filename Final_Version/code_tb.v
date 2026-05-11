// ============================================================
// TESTBENCH
// FULL PARALLEL + PIPELINED LMS FILTER
//
// FPGA DSP Verification
// Spartan-6
// Q16 Fixed Point
//
// INPUT:
// x_in = clean + noise
//
// DESIRED:
// d_in = clean
//
// EXPECTED:
// y_out -> converges to clean signal
// e_out -> converges to ~0
// ============================================================

`timescale 1ns/1ps

module lms_parallel_pipeline_tb;

// ============================================================
// PARAMETERS
// ============================================================

localparam DATA_W   = 32;
localparam N_TAPS   = 64;
localparam MU_SHIFT = 11;

localparam real Q16 = 65536.0;

localparam real FS  = 10000.0;
localparam real TS  = 1.0 / FS;   //100micro second new value will come in the ADC

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

lms_parallel_pipeline #(
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


always #10 clk = ~clk;

// ============================================================
// Q16 FUNCTIONS
// ============================================================

function signed [31:0] real_to_q16;

    input real val;

    begin
        real_to_q16 = $rtoi(val * Q16);  //remove fractional parts rtoi
    end

endfunction

function real q16_to_real;

    input signed [31:0] val;

    begin
        q16_to_real = $itor(val) / Q16;
    end

endfunction

// VARIABLES

integer fd , n;
real t , clean_signal , noise_signal , noisy_signal , error_real , mse , rms_error , peak_error , 
signal_power , noise_power , snr_out , corr_accum_1 , corr_accum_2 , corr_accum_3 , corr_val;

// INITIAL

initial begin

    clk   = 0;
    rst_n = 0;

    x_in  = 0;
    d_in  = 0;

    mse = 0;

    peak_error = 0;

    signal_power = 0;
    noise_power  = 0;

    corr_accum_1 = 0;
    corr_accum_2 = 0;
    corr_accum_3 = 0;

    // OPEN CSV FILE
    fd = $fopen(
        "fpga_parallel_pipeline_output.csv",
        "w"
    );

    $fdisplay(fd,
    "sample,time_s,x_in_V,d_in_V,y_out_V,e_out_V");


    // RESET

    repeat(20) //need some time to stabilise the clock,pipeline ect
        @(posedge clk);

    rst_n = 1;

    @(posedge clk);

    // MAIN SIMULATION LOOP
    for(n=0; n<N_SAMPLES; n=n+1) begin

        t = n * TS;

        // 1V @ 50Hz
        clean_signal =
            1.0 *
            $sin(2.0 * PI * 50.0 * t);

        // 0.5V @ 2Hz
        noise_signal =
            0.5 *
            $sin(2.0 * PI * 200.0 * t);

        // NOISY SIGNAL
        noisy_signal =
            clean_signal +
            noise_signal;

        // APPLY INPUTS

        x_in =
            real_to_q16(noisy_signal);

        d_in =
            real_to_q16(clean_signal);

        @(posedge clk);



        // ERROR ANALYSIS

        error_real = q16_to_real(e_out);

        mse = mse + (error_real * error_real);

        if(error_real > peak_error)
            peak_error = error_real;

        // SNR CALCULATION

        signal_power =
            signal_power +
            (clean_signal * clean_signal);

        noise_power =
            noise_power + ((clean_signal -  q16_to_real(y_out)) * (clean_signal - q16_to_real(y_out)));

        // CORRELATION

        corr_accum_1 =
            corr_accum_1 + (clean_signal * q16_to_real(y_out));

        corr_accum_2 =
            corr_accum_2 + (clean_signal * clean_signal);

        corr_accum_3 =
            corr_accum_3 + (q16_to_real(y_out) *  q16_to_real(y_out));

        // CSV OUTPUT

        $fdisplay(fd,
        "%0d,%f,%f,%f,%f,%f",
        n,
        t,
        q16_to_real(x_in),
        q16_to_real(d_in),
        q16_to_real(y_out),
        q16_to_real(e_out));

    end

    // FINAL CALCULATIONS

    mse = mse / N_SAMPLES;

    rms_error = $sqrt(mse);

    snr_out = 10.0 *  $log10(signal_power / noise_power);

    corr_val = corr_accum_1 / ($sqrt(corr_accum_2) *  $sqrt(corr_accum_3));

    // FINAL REPORT

    $display("");

    $display("=====================================");
    $display(" FPGA PIPELINED PARALLEL LMS");
    $display("=====================================");

    $display("Total Samples      : %0d",  N_SAMPLES);

    $display("Mean Square Error  : %f",   mse);

    $display("RMS Error          : %f",   rms_error);

    $display("Peak Error         : %f", peak_error);

    $display("=====================================");

    // CONVERGENCE STATUS

    if(corr_val > 0.98)
        $display("Excellent convergence achieved.");

    else if(corr_val > 0.90)
        $display("Good convergence achieved.");

    else
        $display("Poor convergence.");

    $display("=====================================");
    $fclose(fd);
    $finish;

end

endmodule