/* verilator lint_off WIDTH */
`include "./mult.sv"
module multiplier (
      input logic clk,
      input logic reset,
      input logic [15:0] a,
      input logic [15:0] b,
      input logic start,
      output logic [31:0] product,
      output logic done
);


logic [3:0][3:0] A ;
logic [3:0][3:0] B;

// pseudo-reduction layers
logic [7:0][15:0] rl1;

logic [15:0] counter;

always_ff @ (posedge clk) begin

      if(reset) counter <= 16'd0;
      else if(start & (counter > 16'd3)) counter <= 16'd0;
      else if(start) counter <= counter + 16'd1;
      else counter <= 16'd0;

      A <= {a[15:12], a[11:8], a[7:4], a[3:0]};
      B <= {b[15:12], b[11:8], b[7:4], b[3:0]};

      rl1[0]  <= mult(A[0], B[0]);
      rl1[1]  <= mult(A[0], B[1]) + mult(A[1], B[0]);
      rl1[2]  <= mult(A[0], B[2]) + mult(A[1], B[1]) + mult(A[2], B[0]);
      rl1[3]  <= mult(A[0], B[3]) + mult(A[1], B[2]) + mult(A[2], B[1]) + mult(A[3], B[0]);
      rl1[4]  <= mult(A[1], B[3]) + mult(A[2], B[2]) + mult(A[3], B[1]);
      rl1[5]  <= mult(A[2], B[3]) + mult(A[3], B[2]);
      rl1[6]  <= mult(A[3], B[3]);


      product <= (rl1[0] << 0) + (rl1[1] << 4) + (rl1[2] << 8) + (rl1[3] << 12) + (rl1[4] << 16) + (rl1[5] << 20) + (rl1[6] << 24);
      done <= (counter == 16'd3);
end
endmodule // multiplier
/* verilator lint_on WIDTH */
