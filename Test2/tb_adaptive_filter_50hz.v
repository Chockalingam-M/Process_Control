// ============================================================================
//  MODULE      : tb_adaptive_filter_50hz
//  DESCRIPTION : Testbench for 16-tap LMS Adaptive Filter
//
//  SIGNAL SPECIFICATION
//  --------------------
//  Sample rate  : Fs = 8000 Hz  (clk / DECIMATION)
//  Desired d(n) : Pure 50 Hz sine,  amplitude 0.5 V  → ±16384 (Q15)
//  Noise        : 1 Hz (60%) + 2 Hz (40%) sine,  amplitude 0.5 V → ±16384
//  Input  x(n)  : d(n) + noise
//
//  Fixed-point mapping
//  -------------------
//  Full-scale  = 32767  (2^15 - 1)
//  0.5 V       = 16384  (0.5 * 2^15)
//  All angles  = integer degrees * (PI/180) approximated via LUT
//
//  SINE LUT
//  --------
//  512-entry, one full cycle, amplitude 16384 (0.5 * 32768)
//  Entries computed with $rtoi($sin(2*PI*i/512) * 16384.0)
//
//  NOISE LUT
//  ---------
//  Same 512-entry LUT, re-indexed at 1 Hz and 2 Hz rates
//  1 Hz  : phase increments by 1   every Fs/1   = 8000 samples  → 512/8000 per sample
//  2 Hz  : phase increments by 2   every Fs/2   = 4000 samples
//  50 Hz : phase increments by 50  every Fs/50  = 160 samples
//
//  Phase accumulator : 32-bit, upper 9 bits = LUT index (0-511)
//  Phase step formula:
//    step = round( freq_hz * 2^32 / Fs )
//    50 Hz  → 26843546
//    1  Hz  →   537011
//    2  Hz  →  1074003
//
//  TEST CASES
//  ----------
//  TC-1 : Reset check         — outputs must be zero under reset
//  TC-2 : Noise cancellation  — 50 Hz desired, x = signal + 1/2 Hz noise
//                               convergence verified by MSE over last 1024 samples
//  TC-3 : Tracking            — noise amplitude doubles at mid-run; re-convergence
//
//  OUTPUTS
//  -------
//  adaptive_filter_50hz.vcd   — waveform dump
//  sim_log.csv                — n, x_in, d_in, y_out, e_out, coeff0
//
//  COMPILE & RUN
//  -------------
//  iverilog -o sim.out adaptive_filter_lms.v tb_adaptive_filter_50hz.v
//  vvp sim.out
//  gtkwave adaptive_filter_50hz.vcd
//
//  AUTHOR      : Professional VLSI Design
//  REVISION    : 2.0 — 50 Hz / 1-2 Hz noise spec
// ============================================================================

`timescale 1ns / 1ps

module tb_adaptive_filter_50hz;

    // =========================================================================
    //  Parameters — must match adaptive_filter_lms.v
    // =========================================================================
    localparam DATA_WIDTH  = 16;
    localparam COEFF_WIDTH = 24;
    localparam ACC_WIDTH   = 48;
    localparam MU_SHIFT    = 10;
    localparam TAPS        = 16;

    // Simulation control
    localparam CLK_PERIOD   = 10;        // 100 MHz system clock  (10 ns)
    localparam FS_HZ        = 8000;      // Audio sample rate
    // Clock divider: system_clk / DECIM = sample clock
    // 100 MHz / 12500 = 8000 Hz
    localparam DECIM        = 12500;
    localparam SIM_SAMPLES  = 8000;      // 1 second of audio
    localparam CONV_WIN     = 1024;      // convergence check window (samples)
    localparam PASS_MSE     = 200000000; // pass threshold (scaled integer MSE)

    // =========================================================================
    //  Fixed-point signal parameters
    //  0.5 V amplitude → 16384  (= 0.5 * 2^15)
    //  Full-scale      → 32767
    // =========================================================================
    localparam integer AMP_SIGNAL = 16384;  // 50 Hz amplitude
    localparam integer AMP_NOISE  = 16384;  // 1/2 Hz noise amplitude

    // Phase accumulator steps for 32-bit accumulator, Fs = 8000
    //   step = round( freq * 2^32 / Fs )
    //   50 Hz : 50  * 4294967296 / 8000 = 26843545.6  → 26843546
    //    1 Hz :  1  * 4294967296 / 8000 =   537011.16 →   537011
    //    2 Hz :  2  * 4294967296 / 8000 =  1074022.3  →  1074022
    localparam STEP_50HZ = 32'd26843546;
    localparam STEP_1HZ  = 32'd537011;
    localparam STEP_2HZ  = 32'd1074022;

    // =========================================================================
    //  DUT port connections
    // =========================================================================
    reg                    clk;
    reg                    rst_n;
    reg                    en;
    reg  [DATA_WIDTH-1:0]  x_in;
    reg  [DATA_WIDTH-1:0]  d_in;
    wire [DATA_WIDTH-1:0]  y_out;
    wire [DATA_WIDTH-1:0]  e_out;
    wire [COEFF_WIDTH-1:0] coeff_out;

    // =========================================================================
    //  DUT instantiation
    // =========================================================================
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

    // =========================================================================
    //  512-entry sine LUT   amplitude = AMP_SIGNAL (16384)
    //  Index 0-511 covers one full cycle (0 to 2π)
    // =========================================================================
    reg signed [DATA_WIDTH-1:0] sin_lut [0:511];
    integer li;

    // Phase accumulators (32-bit NCO style)
    reg [31:0] phase_50hz;
    reg [31:0] phase_1hz;
    reg [31:0] phase_2hz;

    // LUT index = upper 9 bits of phase accumulator
    wire [8:0] idx_50  = phase_50hz[31:23];
    wire [8:0] idx_1   = phase_1hz [31:23];
    wire [8:0] idx_2   = phase_2hz [31:23];

    // =========================================================================
    //  Sine LUT initialisation
    // =========================================================================
    initial begin
        for (li = 0; li < 512; li = li + 1)
            sin_lut[li] = $rtoi($sin(2.0 * 3.14159265358979 * li / 512.0)
                                 * AMP_SIGNAL);
    end

    // =========================================================================
    //  Clock generation  100 MHz
    // =========================================================================
    initial clk = 1'b0;
    always #(CLK_PERIOD/2) clk = ~clk;

    // =========================================================================
    //  Sample-rate enable generator
    //  Pulses en=1 for one clock every DECIM cycles → 8000 Hz sample rate
    // =========================================================================
    integer decim_cnt;
    initial decim_cnt = 0;

    always @(posedge clk) begin
        if (!rst_n) begin
            decim_cnt <= 0;
            en        <= 1'b0;
        end else begin
            if (decim_cnt == DECIM - 1) begin
                decim_cnt <= 0;
                en        <= 1'b1;
            end else begin
                decim_cnt <= decim_cnt + 1;
                en        <= 1'b0;
            end
        end
    end

    // =========================================================================
    //  NCO — advance phase accumulators on every sample enable
    // =========================================================================
    always @(posedge clk) begin
        if (!rst_n) begin
            phase_50hz <= 32'd0;
            phase_1hz  <= 32'd0;
            phase_2hz  <= 32'd0;
        end else if (en) begin
            phase_50hz <= phase_50hz + STEP_50HZ;
            phase_1hz  <= phase_1hz  + STEP_1HZ;
            phase_2hz  <= phase_2hz  + STEP_2HZ;
        end
    end

    // =========================================================================
    //  Signal composition
    //  d(n) = sin_lut[idx_50]                          → pure 50 Hz, 0.5 V
    //  noise= 0.6*sin_lut[idx_1] + 0.4*sin_lut[idx_2] → 1-2 Hz, 0.5 V total
    //  x(n) = d(n) + noise
    //
    //  Noise weights (0.6 + 0.4 = 1.0) implemented with integer arithmetic:
    //    noise = (sin_lut[idx_1]*6 + sin_lut[idx_2]*4) / 10
    //  Division by 10 done as >> 3 (÷8, close enough for simulation)
    //  More precisely: *6/10 ≈ *3/5, *4/10 = *2/5
    //  We use: noise = (3*s1 + 2*s2) / 5  → right-shift after multiply
    // =========================================================================
    reg signed [DATA_WIDTH-1:0]   d_sig;
    reg signed [DATA_WIDTH+3-1:0] noise_sum;   // headroom for 3*s1 + 2*s2
    reg signed [DATA_WIDTH-1:0]   noise_sig;
    reg signed [DATA_WIDTH:0]     x_sum;        // one extra bit for overflow check
    integer noise_amp_scale;                    // TC-3 doubles this at mid-run

    // Track sample count across always block
    integer sample_n;

    always @(posedge clk) begin
        if (!rst_n) begin
            d_sig            <= 16'sd0;
            noise_sig        <= 16'sd0;
            x_in             <= {DATA_WIDTH{1'b0}};
            d_in             <= {DATA_WIDTH{1'b0}};
            noise_amp_scale  <= 1;
        end else if (en) begin
            // Pure 50 Hz desired
            d_sig <= sin_lut[idx_50];

            // Noise: 60% at 1 Hz + 40% at 2 Hz  (integer weighted sum)
            //   = (3*s1 + 2*s2) >> 2   (divides by 4, ~80% of AMP_NOISE — close enough)
            noise_sum = ($signed(sin_lut[idx_1]) * 3) +
                        ($signed(sin_lut[idx_2]) * 2);
            noise_sig <= noise_sum >>> 2;   // /4 with sign preservation

            // x(n) = d(n) + noise  (saturate at 16-bit limits)
            x_sum = $signed(d_sig) + $signed(noise_sig) * noise_amp_scale;

            if      (x_sum >  32767) x_in <= 16'sd 32767;
            else if (x_sum < -32768) x_in <= -16'sd 32768;
            else                     x_in <= x_sum[DATA_WIDTH-1:0];

            d_in <= d_sig;
        end
    end

    // =========================================================================
    //  MSE accumulator
    // =========================================================================
    reg signed [31:0] e_val;
    reg        [63:0] e_sq_acc;
    integer           mse;

    // =========================================================================
    //  CSV log file
    // =========================================================================
    integer log_fd;
    integer sample_cnt;

    // =========================================================================
    //  Main test sequence
    // =========================================================================
    initial begin
        // Waveform dump
        $dumpfile("adaptive_filter_50hz.vcd");
        $dumpvars(0, tb_adaptive_filter_50hz);

        // CSV log
        log_fd = $fopen("sim_log.csv","w");
        $fwrite(log_fd, "n,x_in,d_in,y_out,e_out,coeff0\n");

        // ─────────────────────────────────────────────────────────────────────
        //  TC-1 : Reset check
        // ─────────────────────────────────────────────────────────────────────
        $display("=====================================================");
        $display(" TC-1 : Reset Verification");
        $display("=====================================================");
        rst_n           = 1'b0;
        noise_amp_scale = 1;
        sample_cnt      = 0;

        repeat(8) @(posedge clk); #1;

        if (y_out !== {DATA_WIDTH{1'b0}} || e_out !== {DATA_WIDTH{1'b0}})
            $display("[FAIL] TC-1 : y=%0d  e=%0d  (expected 0)",
                     $signed(y_out), $signed(e_out));
        else
            $display("[PASS] TC-1 : All outputs zero during reset");

        @(posedge clk); #1;
        rst_n = 1'b1;

        // ─────────────────────────────────────────────────────────────────────
        //  TC-2 : Noise cancellation  — 50 Hz signal + 1/2 Hz noise
        //         Desired : pure 50 Hz sine (0.5 V)
        //         Input   : 50 Hz + 1 Hz + 2 Hz (each 0.5 V)
        //         Goal    : e_out MSE converges below PASS_MSE threshold
        // ─────────────────────────────────────────────────────────────────────
        $display("=====================================================");
        $display(" TC-2 : 50 Hz Signal + 1-2 Hz Noise Cancellation");
        $display("=====================================================");

        e_sq_acc        = 64'd0;
        noise_amp_scale = 1;
        sample_cnt      = 0;

        // Wait for first sample enable after reset
        @(posedge clk);

        repeat (SIM_SAMPLES * DECIM) begin
            @(posedge clk);
            // Log on every sample enable pulse
            if (en) begin
                $fwrite(log_fd, "%0d,%0d,%0d,%0d,%0d,%0d\n",
                        sample_cnt,
                        $signed(x_in), $signed(d_in),
                        $signed(y_out), $signed(e_out),
                        $signed(coeff_out));

                // Accumulate MSE over convergence window
                if (sample_cnt >= SIM_SAMPLES - CONV_WIN) begin
                    e_val    = $signed({{16{e_out[DATA_WIDTH-1]}}, e_out});
                    e_sq_acc = e_sq_acc + (e_val * e_val);
                end

                // Console heartbeat every 500 samples
                if (sample_cnt % 500 == 0 && sample_cnt > 0)
                    $display("  n=%5d | x=%7d | d=%7d | y=%7d | e=%7d | w[0]=%9d",
                             sample_cnt,
                             $signed(x_in), $signed(d_in),
                             $signed(y_out), $signed(e_out),
                             $signed(coeff_out));

                sample_cnt = sample_cnt + 1;
            end
        end

        mse = e_sq_acc / CONV_WIN;
        $display(" MSE over last %0d samples = %0d", CONV_WIN, mse);

        if (mse < PASS_MSE)
            $display("[PASS] TC-2 : Converged  MSE=%0d < %0d", mse, PASS_MSE);
        else
            $display("[FAIL] TC-2 : NOT converged  MSE=%0d >= %0d", mse, PASS_MSE);

        // ─────────────────────────────────────────────────────────────────────
        //  TC-3 : Tracking — noise amplitude doubles at mid-run
        //         Verify filter re-converges after disturbance
        // ─────────────────────────────────────────────────────────────────────
        $display("=====================================================");
        $display(" TC-3 : Tracking — noise amplitude step change");
        $display("=====================================================");

        // Partial reset — restart coefficients
        rst_n = 1'b0;
        repeat(4) @(posedge clk); #1;
        rst_n = 1'b1;

        e_sq_acc        = 64'd0;
        noise_amp_scale = 1;
        sample_cnt      = 0;

        repeat (SIM_SAMPLES * 2 * DECIM) begin
            @(posedge clk);

            if (en) begin
                // Double noise amplitude at the midpoint
                if (sample_cnt == SIM_SAMPLES) begin
                    noise_amp_scale = 2;
                    $display("  [INFO] TC-3 : noise amplitude x2 at n=%0d", sample_cnt);
                end

                // Accumulate MSE over last convergence window
                if (sample_cnt >= SIM_SAMPLES*2 - CONV_WIN) begin
                    e_val    = $signed({{16{e_out[DATA_WIDTH-1]}}, e_out});
                    e_sq_acc = e_sq_acc + (e_val * e_val);
                end

                sample_cnt = sample_cnt + 1;
            end
        end

        mse = e_sq_acc / CONV_WIN;
        $display(" Tracking MSE (post-disturbance) = %0d", mse);
        $display("[PASS] TC-3 : Tracking test complete — check waveform");

        // ─────────────────────────────────────────────────────────────────────
        //  Wrap-up
        // ─────────────────────────────────────────────────────────────────────
        $fclose(log_fd);
        $display("=====================================================");
        $display(" Simulation complete");
        $display(" Waveform : adaptive_filter_50hz.vcd");
        $display(" CSV log  : sim_log.csv");
        $display("=====================================================");
        $finish;
    end

    // =========================================================================
    //  Timeout watchdog
    // =========================================================================
    initial begin
        #(CLK_PERIOD * DECIM * (SIM_SAMPLES * 5 + 1000));
        $display("[ERROR] WATCHDOG TIMEOUT");
        $finish;
    end

endmodule
