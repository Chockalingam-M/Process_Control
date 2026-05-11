
`timescale 1ns/1ps

module lms_parallel_pipeline #(
    parameter N_TAPS   = 64,
    parameter DATA_W   = 32,
    parameter MU_SHIFT = 11
)(
    input wire clk,
    input wire rst_n,

    input wire signed [DATA_W-1:0] x_in,
    input wire signed [DATA_W-1:0] d_in,

    output reg signed [DATA_W-1:0] y_out,
    output reg signed [DATA_W-1:0] e_out,

    output reg valid
);

integer i;

// MEMORY


reg signed [DATA_W-1:0] w [0:N_TAPS-1];   // Which create a 64 locations to store the value of 32 bit values..
reg signed [DATA_W-1:0] x_buf [0:N_TAPS-1]; //used to store the input in reg for after uses..

// STAGE 1
// INPUT SHIFT REGISTER

always @(posedge clk or negedge rst_n) begin   //use DDR(double data rate), switch is pull-up

    if(!rst_n) begin
        for(i=0;i<N_TAPS;i=i+1)
            x_buf[i] <= 0;


    end

    else begin  //shiffting the value after getting new value
        x_buf[0] <= x_in;
        for(i=1;i<N_TAPS;i=i+1)
            x_buf[i] <= x_buf[i-1];
    end
end

// STAGE 2
// REGISTERED PARALLEL MULTIPLIERS

reg signed [63:0] mult_r [0:N_TAPS-1];

always @(posedge clk) begin

    for(i=0;i<N_TAPS;i=i+1) begin

        mult_r[i] <= $signed(w[i]) * $signed(x_buf[i]);
    end
end

// STAGE 3
// REGISTERED ADDER TREE LEVEL 1

reg signed [63:0] sum_l1 [0:31];
always @(posedge clk) begin
    for(i=0;i<32;i=i+1) begin

        sum_l1[i] <= mult_r[2*i] + mult_r[2*i+1];
    end
end

// STAGE 4
// REGISTERED ADDER TREE LEVEL 2

reg signed [63:0] sum_l2 [0:15];

always @(posedge clk) begin

    for(i=0;i<16;i=i+1) begin

        sum_l2[i] <= sum_l1[2*i] + sum_l1[2*i+1];
    end
end

// STAGE 5
// REGISTERED ADDER TREE LEVEL 3

reg signed [63:0] sum_l3 [0:7];

always @(posedge clk) begin

    for(i=0;i<8;i=i+1) begin

        sum_l3[i] <= sum_l2[2*i] + sum_l2[2*i+1];
    end
end

// STAGE 6
// REGISTERED ADDER TREE LEVEL 4

reg signed [63:0] sum_l4 [0:3];

always @(posedge clk) begin

    for(i=0;i<4;i=i+1) begin

        sum_l4[i] <= sum_l3[2*i] + sum_l3[2*i+1];
    end
end

// STAGE 7
// REGISTERED ADDER TREE LEVEL 5

reg signed [63:0] sum_l5 [0:1];

always @(posedge clk) begin

    for(i=0;i<2;i=i+1) begin

        sum_l5[i] <= sum_l4[2*i] + sum_l4[2*i+1];
    end
end

// STAGE 8
// FINAL SUM REGISTER

reg signed [63:0] final_sum_r;

always @(posedge clk) begin

    final_sum_r <= sum_l5[0] + sum_l5[1];

end

// STAGE 9
// OUTPUT + ERROR

reg signed [DATA_W-1:0] y_r;
reg signed [DATA_W-1:0] e_r;

always @(posedge clk) begin

    y_r <= final_sum_r[47:16];
    e_r <= d_in - final_sum_r[47:16];

end

// STAGE 10
// WEIGHT UPDATE

reg signed [63:0] wu_r [0:N_TAPS-1];
always @(posedge clk) begin

    for(i=0;i<N_TAPS;i=i+1) begin
        wu_r[i] <= $signed(e_r) * $signed(x_buf[i])  ;
    end
end

// STAGE 11
// UPDATE COEFFICIENTS

always @(posedge clk or negedge rst_n) begin

    if(!rst_n) begin
        for(i=0;i<N_TAPS;i=i+1)
            w[i] <= 0;
            y_out <= 0;
            e_out <=  0;
            valid <= 0;
    end

    else begin
        for(i=0;i<N_TAPS;i=i+1) begin

            w[i] <= w[i] + $signed(wu_r[i] >>> (16 + MU_SHIFT));

        end

        y_out <= y_r;
        e_out <= e_r;
        valid <= 1'b1;

    end
end

endmodule