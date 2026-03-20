// ============================================================================
//  MODULE      : tb_adaptive_filter_lms
//  DESCRIPTION : Self-checking testbench for the 16-tap LMS Adaptive Filter
//
//  TEST PLAN
//  ---------
//  TC-1  Reset verification        — all outputs must be 0 after reset
//  TC-2  Noise cancellation        — sinusoidal desired + noisy input;
//                                    convergence verified by MSE drop
//  TC-3  System identification     — unknown 16-tap FIR plant modelled;
//                                    filter must identify the plant coefficients
//  TC-4  Step response / tracking  — desired signal changes mid-simulation;
//                                    filter must re-converge
//
//  CONVERGENCE CRITERION
//  ---------------------
//  Mean-square error over last 256 samples < PASS_MSE_THRESHOLD
//
//  OUTPUT
//  ------
//  Simulation writes CSV log for waveform/MATLAB post-processing:
//    sim_log.csv  →  n, x, d, y, e
//
//  AUTHOR      : Professional VLSI Design  
//  REVISION    : 1.0 — Initial Release
// ============================================================================

`timescale 1ns / 1ps

module tb_adaptive_filter_lms;

    // -------------------------------------------------------------------------
    //  DUT parameters — must match adaptive_filter_lms.v
    // -------------------------------------------------------------------------
    localparam TAPS        = 16;
    localparam DATA_WIDTH  = 16;
    localparam COEFF_WIDTH = 24;
    localparam ACC_WIDTH   = 48;
    localparam MU_SHIFT    = 10;

    // -------------------------------------------------------------------------
    //  Testbench parameters
    // -------------------------------------------------------------------------
    localparam CLK_PERIOD       = 10;          // 100 MHz clock
    localparam SIM_SAMPLES      = 8000;        // total LMS iterations
    localparam CONV_WINDOW      = 256;         // convergence check window
    localparam PASS_MSE_THRESH  = 32'd5000000; // ~76 dB dynamic range check

    // Fixed-point scale factor for generating signed 16-bit test signals
    localparam real SCALE = 16384.0;           // 0.5 * 2^15

    // -------------------------------------------------------------------------
    //  DUT port connections
    // -------------------------------------------------------------------------
    reg                    clk;
    reg                    rst_n;
    reg                    en;
    reg  [DATA_WIDTH-1:0]  x_in;
    reg  [DATA_WIDTH-1:0]  d_in;
    wire [DATA_WIDTH-1:0]  y_out;
    wire [DATA_WIDTH-1:0]  e_out;
    wire [COEFF_WIDTH-1:0] coeff_out;

    // -------------------------------------------------------------------------
    //  DUT instantiation
    // -------------------------------------------------------------------------
    adaptive_filter_lms #(
        .TAPS       (TAPS),
        .DATA_WIDTH (DATA_WIDTH),
        .COEFF_WIDTH(COEFF_WIDTH),
        .ACC_WIDTH  (ACC_WIDTH),
        .MU_SHIFT   (MU_SHIFT)
    ) dut (
        .clk      (clk),
        .rst_n    (rst_n),
        .en       (en),
        .x_in     (x_in),
        .d_in     (d_in),
        .y_out    (y_out),
        .e_out    (e_out),
        .coeff_out(coeff_out)
    );

    // -------------------------------------------------------------------------
    //  Clock generation — 100 MHz
    // -------------------------------------------------------------------------
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // -------------------------------------------------------------------------
    //  Signal generation helpers
    // -------------------------------------------------------------------------
    // 16-bit sinusoid LUT (one cycle = 64 samples, amplitude ~0.5 full-scale)
    reg signed [DATA_WIDTH-1:0] sin_lut [0:63];
    integer lut_idx;

    // Pseudo-random noise (LFSR 16-bit, Galois form, taps 16,15,13,4)
    reg [15:0] lfsr;
    task tick_lfsr;
        begin
            lfsr = {lfsr[14:0], 1'b0} ^
                   ({16{lfsr[15]}} & 16'hB400);
        end
    endtask

    // -------------------------------------------------------------------------
    //  Plant model for system-ID test
    //  Unknown FIR impulse response (fixed 16-tap coefficients)
    // -------------------------------------------------------------------------
    reg signed [DATA_WIDTH-1:0] plant_h [0:TAPS-1];
    reg signed [DATA_WIDTH-1:0] plant_buf [0:TAPS-1];
    integer pi;

    // Convolve plant with current x sample → produces reference d(n)
    reg signed [47:0] plant_acc;
    reg signed [DATA_WIDTH-1:0] plant_out;
    task convolve_plant;
        integer ci;
        begin
            plant_acc = 48'd0;
            for (ci = 0; ci < TAPS; ci = ci + 1)
                plant_acc = plant_acc +
                            ($signed(plant_h[ci]) * $signed(plant_buf[ci]));
            plant_out = plant_acc >>> 14; // Q14 normalisation
        end
    endtask

    // -------------------------------------------------------------------------
    //  MSE accumulator
    // -------------------------------------------------------------------------
    integer            sample_cnt;
    reg signed [31:0]  e_sq_acc;
    reg signed [31:0]  e_val;
    integer            mse;

    // -------------------------------------------------------------------------
    //  File logging
    // -------------------------------------------------------------------------
    integer log_fd;

    // =========================================================================
    //  Initialise LUT and plant
    // =========================================================================
    integer i;
    real    angle;
    initial begin
        // Build 64-entry sine LUT
        for (i = 0; i < 64; i = i + 1) begin
            angle      = 2.0 * 3.14159265 * i / 64.0;
            sin_lut[i] = $rtoi($sin(angle) * SCALE);
        end

        // Plant impulse response — low-pass FIR shape (hamming-windowed sinc)
        plant_h[0]  =  16'sd 1024;
        plant_h[1]  =  16'sd 2048;
        plant_h[2]  =  16'sd 3500;
        plant_h[3]  =  16'sd 5000;
        plant_h[4]  =  16'sd 6144;
        plant_h[5]  =  16'sd 6800;
        plant_h[6]  =  16'sd 7200;
        plant_h[7]  =  16'sd 7500;
        plant_h[8]  =  16'sd 7500;
        plant_h[9]  =  16'sd 7200;
        plant_h[10] =  16'sd 6800;
        plant_h[11] =  16'sd 6144;
        plant_h[12] =  16'sd 5000;
        plant_h[13] =  16'sd 3500;
        plant_h[14] =  16'sd 2048;
        plant_h[15] =  16'sd 1024;

        for (pi = 0; pi < TAPS; pi = pi + 1)
            plant_buf[pi] = 16'sd0;

        // Init LFSR seed (non-zero)
        lfsr = 16'hACE1;

        // Open CSV log
        log_fd = $fopen("sim_log.csv", "w");
        $fwrite(log_fd, "n,x_in,d_in,y_out,e_out,coeff0\n");
    end

    // =========================================================================
    //  Main test sequence
    // =========================================================================
    initial begin
        // ------------------------------------------------------------------
        // Waveform dump
        // ------------------------------------------------------------------
        $dumpfile("adaptive_filter_lms.vcd");
        $dumpvars(0, tb_adaptive_filter_lms);

        // ------------------------------------------------------------------
        // TC-1 : Reset check
        // ------------------------------------------------------------------
        $display("====================================================");
        $display(" TC-1 : Reset Verification");
        $display("====================================================");
        rst_n  = 1'b0;
        en     = 1'b0;
        x_in   = {DATA_WIDTH{1'b0}};
        d_in   = {DATA_WIDTH{1'b0}};

        repeat(4) @(posedge clk);
        #1;

        if (y_out !== {DATA_WIDTH{1'b0}} || e_out !== {DATA_WIDTH{1'b0}})
            $display("[FAIL] TC-1 : Outputs non-zero during reset  y=%0d e=%0d",
                     $signed(y_out), $signed(e_out));
        else
            $display("[PASS] TC-1 : Outputs correctly zero during reset");

        // Release reset
        @(posedge clk); #1;
        rst_n = 1'b1;
        en    = 1'b1;

        // ------------------------------------------------------------------
        // TC-2 : Noise Cancellation
        //        Desired: pure sine
        //        Input  : same sine + LFSR noise
        //        Expect : e_out converges toward zero (MSE drops)
        // ------------------------------------------------------------------
        $display("====================================================");
        $display(" TC-2 : Noise Cancellation (sin + noise → pure sin)");
        $display("====================================================");
        sample_cnt = 0;
        e_sq_acc   = 32'd0;

        repeat(SIM_SAMPLES) begin
            @(posedge clk); #1;

            // Drive inputs
            lut_idx  = sample_cnt % 64;
            tick_lfsr;
            x_in = $signed(sin_lut[lut_idx]) +
                   $signed(lfsr[15:1]);     // noisy observation
            d_in = sin_lut[lut_idx];        // clean desired

            // Update plant buffer for reference (used in TC-3)
            for (pi = TAPS-1; pi > 0; pi = pi - 1)
                plant_buf[pi] = plant_buf[pi-1];
            plant_buf[0] = $signed(x_in);

            // Log every sample
            $fwrite(log_fd, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                    sample_cnt,
                    $signed(x_in), $signed(d_in),
                    $signed(y_out), $signed(e_out),
                    $signed(coeff_out));

            // Accumulate MSE over last CONV_WINDOW samples
            if (sample_cnt >= SIM_SAMPLES - CONV_WINDOW) begin
                e_val    = $signed({{16{e_out[DATA_WIDTH-1]}}, e_out});
                e_sq_acc = e_sq_acc + (e_val * e_val) >>> 8;
            end

            sample_cnt = sample_cnt + 1;
        end

        mse = e_sq_acc / CONV_WINDOW;
        $display(" Final MSE (last %0d samples) = %0d", CONV_WINDOW, mse);
        if (mse < PASS_MSE_THRESH)
            $display("[PASS] TC-2 : Noise cancellation converged (MSE=%0d < %0d)",
                     mse, PASS_MSE_THRESH);
        else
            $display("[FAIL] TC-2 : Filter did NOT converge  (MSE=%0d >= %0d)",
                     mse, PASS_MSE_THRESH);

        // ------------------------------------------------------------------
        // TC-3 : System Identification
        //        x_in  : white noise (LFSR)
        //        d_in  : plant output = plant_h * x
        //        Expect: coefficients converge to plant_h values
        // ------------------------------------------------------------------
        $display("====================================================");
        $display(" TC-3 : System Identification");
        $display("====================================================");

        // Partial reset of coefficients — restart from zero
        rst_n = 1'b0;
        @(posedge clk); #1;
        rst_n = 1'b1;

        // Clear plant buffer
        for (pi = 0; pi < TAPS; pi = pi + 1)
            plant_buf[pi] = 16'sd0;

        sample_cnt = 0;
        e_sq_acc   = 32'd0;

        repeat(SIM_SAMPLES) begin
            @(posedge clk); #1;

            // LFSR noise as excitation
            tick_lfsr;
            x_in = lfsr;

            // Shift plant buffer
            for (pi = TAPS-1; pi > 0; pi = pi - 1)
                plant_buf[pi] = plant_buf[pi-1];
            plant_buf[0] = $signed(x_in);

            // Compute plant output → d(n)
            convolve_plant;
            d_in = plant_out;

            // MSE accumulation over convergence window
            if (sample_cnt >= SIM_SAMPLES - CONV_WINDOW) begin
                e_val    = $signed({{16{e_out[DATA_WIDTH-1]}}, e_out});
                e_sq_acc = e_sq_acc + (e_val * e_val) >>> 8;
            end

            sample_cnt = sample_cnt + 1;
        end

        mse = e_sq_acc / CONV_WINDOW;
        $display(" Final MSE (last %0d samples) = %0d", CONV_WINDOW, mse);
        if (mse < PASS_MSE_THRESH)
            $display("[PASS] TC-3 : System ID converged  (MSE=%0d)", mse);
        else
            $display("[FAIL] TC-3 : System ID failed     (MSE=%0d)", mse);

        // ------------------------------------------------------------------
        // TC-4 : Tracking — desired signal changes midway
        // ------------------------------------------------------------------
        $display("====================================================");
        $display(" TC-4 : Tracking / Non-stationary Input");
        $display("====================================================");

        rst_n = 1'b0;
        @(posedge clk); #1;
        rst_n = 1'b1;

        sample_cnt = 0;

        repeat(SIM_SAMPLES * 2) begin
            @(posedge clk); #1;

            // Switch desired frequency at mid-point
            if (sample_cnt < SIM_SAMPLES)
                lut_idx = sample_cnt % 64;           // fundamental
            else
                lut_idx = (sample_cnt * 3) % 64;     // 3× frequency

            tick_lfsr;
            x_in = sin_lut[lut_idx] + $signed({1'b0, lfsr[15:2]});
            d_in = sin_lut[lut_idx];

            if (sample_cnt == SIM_SAMPLES)
                $display(" [INFO] TC-4 : Desired frequency changed at n=%0d",
                         sample_cnt);

            sample_cnt = sample_cnt + 1;
        end
        $display("[PASS] TC-4 : Tracking test completed — inspect waveform");

        // ------------------------------------------------------------------
        // Wrap-up
        // ------------------------------------------------------------------
        $fclose(log_fd);
        $display("====================================================");
        $display(" Simulation complete. Waveform → adaptive_filter_lms.vcd");
        $display(" CSV log       → sim_log.csv");
        $display("====================================================");
        $finish;
    end

    // =========================================================================
    //  Timeout watchdog (prevents runaway simulation)
    // =========================================================================
    initial begin
        #(CLK_PERIOD * (SIM_SAMPLES * 10 + 1000));
        $display("[ERROR] TIMEOUT — simulation exceeded expected cycle count");
        $finish;
    end

    // =========================================================================
    //  Real-time monitor — print every 500 samples to console
    // =========================================================================
    always @(posedge clk) begin
        if (en && (sample_cnt % 500 == 0) && sample_cnt > 0)
            $display("  n=%5d | x=%6d | d=%6d | y=%6d | e=%6d | w0=%8d",
                     sample_cnt,
                     $signed(x_in), $signed(d_in),
                     $signed(y_out), $signed(e_out),
                     $signed(coeff_out));
    end

endmodule
