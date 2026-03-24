// ============================================================
//  Testbench  –  LMS Adaptive Filter (corrected concept)
//
//  x_in  = 1.0V/50Hz + 0.5V/2Hz   (NOISY, large amplitude)
//  d_in  = 1.0V/50Hz               (PURE clean reference)
//  y_out → starts large/noisy, converges DOWN to 1V/50Hz clean
//  e_out → starts large, shrinks toward 0 as y_out → d_in
//
//  Fs = 10 kHz, 50 000 samples (5 seconds), CSV output
// ============================================================
`timescale 1ns/1ps

module lms_tb;

    localparam DATA_W   = 32;
    localparam N_TAPS   = 64;
    localparam MU_SHIFT = 11;

    localparam real Q16      = 65536.0;
    localparam real FS       = 10000.0;
    localparam real TS       = 1.0 / FS;
    localparam real PI       = 3.14159265358979;
    localparam real A_NOISE  = 0.5;
    localparam real F_NOISE  = 2.0;
    localparam real A_CLEAN  = 1.0;
    localparam real F_CLEAN  = 50.0;
    localparam integer N_SAMPLES = 10000;

    reg                      clk   = 0;
    reg                      rst_n = 0;
    reg  signed [DATA_W-1:0] x_in;
    reg  signed [DATA_W-1:0] d_in;
    wire signed [DATA_W-1:0] y_out;
    wire signed [DATA_W-1:0] e_out;
    wire                      valid;

    lms_adaptive_filter #(
        .N_TAPS  (N_TAPS),
        .DATA_W  (DATA_W),
        .MU_SHIFT(MU_SHIFT)
    ) DUT (
        .clk(clk), .rst_n(rst_n),
        .x_in(x_in), .d_in(d_in),
        .y_out(y_out), .e_out(e_out), .valid(valid)
    );

    always #1 clk = ~clk;

    function signed [31:0] r2q16;
        input real v;
        begin r2q16 = $rtoi(v * Q16); end
    endfunction

    function real q2r;
        input signed [31:0] v;
        begin q2r = $itor(v) / Q16; end
    endfunction

    integer n, fd;
    real t_s, noise_v, clean_v, x_v, d_v;

    initial begin
        fd = $fopen("lms_output.csv", "w");
        $fdisplay(fd, "sample,time_s,x_in_V,d_in_V,y_out_V,e_out_V");

        rst_n = 0; x_in = 0; d_in = 0;
        repeat(8) @(posedge clk);
        rst_n = 1;
        @(posedge clk);

        for (n = 0; n < N_SAMPLES; n = n + 1) begin
            t_s     = n * TS;
            noise_v = A_NOISE * $sin(2.0 * PI * F_NOISE * t_s);
            clean_v = A_CLEAN * $sin(2.0 * PI * F_CLEAN * t_s);

            // x_in = noisy (large), d_in = pure clean
            x_in = r2q16(clean_v + noise_v);
            d_in = r2q16(clean_v);

            @(posedge clk); #0;

            $fdisplay(fd, "%0d,%.6f,%.6f,%.6f,%.6f,%.6f",
                n, t_s,
                q2r(x_in), q2r(d_in),
                q2r(y_out), q2r(e_out));
        end

        $fclose(fd);
        $display("Done. lms_output.csv written (%0d samples)", N_SAMPLES);
        $finish;
    end

endmodule