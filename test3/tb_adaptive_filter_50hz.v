// ============================================================
//  Testbench for lms_adaptive_filter
//
//  Generates:
//    x_in  : 0.5 V  @ 2 Hz  (noise reference)
//    d_in  : 1.0 V  @ 50 Hz + 0.5 V @ 2 Hz  (desired + noise)
//    Dumps all signals to lms_output.csv
//
//  Clock: 10 kHz sampling rate  (Ts = 100 µs → 100 ns period here,
//         time-scale mapped to samples so 1 "second" = 10000 clocks)
//
//  Run 5 seconds worth → 50 000 samples (enough for convergence)
// ============================================================
`timescale 1ns/1ps

module lms_tb;

    // -----------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------
    localparam DATA_W    = 32;
    localparam N_TAPS    = 32;
    localparam MU_SHIFT  = 10;

    // Q16 scale factor
    localparam real Q16  = 65536.0;   // 2^16

    // Sampling frequency (samples/sec) – simulated as 1 ns per sample
    localparam real FS   = 10000.0;   // 10 kHz
    localparam real TS   = 1.0/FS;    // 100 µs per sample

    // Signal parameters
    localparam real A_NOISE   = 0.5;   // noise amplitude  (V)
    localparam real F_NOISE   = 2.0;   // noise frequency  (Hz)
    localparam real A_CLEAN   = 1.0;   // clean amplitude  (V)
    localparam real F_CLEAN   = 50.0;  // clean frequency  (Hz)

    localparam real PI = 3.14159265358979;

    // Total samples to simulate  (5 s × 10 kHz)
    localparam integer N_SAMPLES = 50000;

    // -----------------------------------------------------------
    // DUT signals
    // -----------------------------------------------------------
    reg                      clk   = 0;
    reg                      rst_n = 0;
    reg  signed [DATA_W-1:0] x_in;
    reg  signed [DATA_W-1:0] d_in;
    wire signed [DATA_W-1:0] y_out;
    wire signed [DATA_W-1:0] e_out;
    wire                      valid;

    // -----------------------------------------------------------
    // DUT instantiation
    // -----------------------------------------------------------
    lms_adaptive_filter #(
        .N_TAPS  (N_TAPS),
        .DATA_W  (DATA_W),
        .MU_SHIFT(MU_SHIFT)
    ) DUT (
        .clk   (clk),
        .rst_n (rst_n),
        .x_in  (x_in),
        .d_in  (d_in),
        .y_out (y_out),
        .e_out (e_out),
        .valid (valid)
    );

    // -----------------------------------------------------------
    // Clock: 1 ns period (each rising edge = 1 sample)
    // -----------------------------------------------------------
    always #1 clk = ~clk;

    // -----------------------------------------------------------
    // Real-to-Q16 and Q16-to-real helpers via integer
    // -----------------------------------------------------------
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

    // -----------------------------------------------------------
    // Simulation variables
    // -----------------------------------------------------------
    integer sample_idx;
    real    t_sec;
    real    noise_val, clean_val, desired_val;
    integer fd;
    real    x_r, d_r, y_r, e_r;

    // -----------------------------------------------------------
    // Main stimulus
    // -----------------------------------------------------------
    initial begin
        // Open CSV
        fd = $fopen("lms_output.csv", "w");
        if (fd == 0) begin
            $display("ERROR: Cannot open lms_output.csv");
            $finish;
        end
        // CSV header
        $fdisplay(fd, "sample,time_s,x_in_V,d_in_V,y_out_V,e_out_V");

        // Reset for 5 cycles
        rst_n = 0;
        x_in  = 0;
        d_in  = 0;
        repeat(5) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        // ---- Main sample loop ----
        for (sample_idx = 0; sample_idx < N_SAMPLES; sample_idx = sample_idx + 1) begin
            // Compute time
            t_sec = sample_idx * TS;

            // Generate signals
            noise_val   = A_NOISE * $sin(2.0 * PI * F_NOISE   * t_sec);
            clean_val   = A_CLEAN * $sin(2.0 * PI * F_CLEAN   * t_sec);
            desired_val = clean_val + noise_val;

            // Drive inputs
            x_in = real_to_q16(noise_val);
            d_in = real_to_q16(desired_val);

            @(posedge clk);  // one clock tick
            #0;              // let outputs settle

            // Capture real values
            x_r = q16_to_real(x_in);
            d_r = q16_to_real(d_in);
            y_r = q16_to_real(y_out);
            e_r = q16_to_real(e_out);

            // Write CSV row
            $fdisplay(fd, "%0d,%.6f,%.6f,%.6f,%.6f,%.6f",
                      sample_idx, t_sec, x_r, d_r, y_r, e_r);
        end

        $fclose(fd);
        $display("Simulation complete. CSV written: lms_output.csv");
        $display("Total samples: %0d  (%.1f seconds at %.0f Hz)",
                  N_SAMPLES, N_SAMPLES*TS, FS);
        $finish;
    end

    // -----------------------------------------------------------
    // Optional waveform dump (uncomment if using GTKWave)
    // -----------------------------------------------------------
    // initial begin
    //     $dumpfile("lms_tb.vcd");
    //     $dumpvars(0, lms_tb);
    // end

endmodule