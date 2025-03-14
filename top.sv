`include "./config.sv"
`include "./constants.sv"

module top(input logic clk,
           input logic reset,
	   output logic regwritememwb);
   
   
//data cache signals
   logic  dcache_data_ready;
   logic [31:0] dcache_data_out;
   logic     dcache_valid;
   logic [3:0] dcache_byte_en;
   logic [31:0] dcache_data_in;
   logic [`DRAM_ADDRESS_SIZE-1:0] dcache_address;
   logic     dcahce_mem_valid;
   logic     dcache_rw;
   
//instruction cache signals 
   logic  [`DRAM_ADDRESS_SIZE-1:0] icache_address;
   logic     icache_valid;
   logic [31:0] icache_data_out;
   logic     icache_data_ready;
   logic icache_mem_valid;
   
//dram signals
   logic dram_busy;
   logic cache_miss_stall;
assign cache_miss_stall = dram_busy || (icache_mem_valid && !icache_data_ready) || (dcahce_mem_valid && !dcache_data_ready);
   //cache to mem signals
   logic dram_port2_acknowledge;
   logic [`DRAM_WORD_SIZE-1:0]     dram_port2_write_data[`DRAM_BLOCK_SIZE-1:0];
   logic [`DRAM_WORD_SIZE-1:0]    dram_port2_read_data[`DRAM_BLOCK_SIZE-1:0];
   logic dram_port2_we;
   logic [`DRAM_ADDRESS_SIZE-1:0]  dram_port2_address;
   logic dram_port2_request;
   logic dram_port1_acknowledge;
   logic [`DRAM_WORD_SIZE-1:0]    dram_port1_read_data[`DRAM_BLOCK_SIZE-1:0];
   logic dram_port1_we;
   logic [`DRAM_ADDRESS_SIZE-1:0]  dram_port1_address;
   logic dram_port1_request;
   logic [`DRAM_WORD_SIZE-1:0] mem_data_out[`DRAM_BLOCK_SIZE-1:0];
   logic [`DRAM_ADDRESS_SIZE-1:0] icache_mem_address;
   logic [`DRAM_ADDRESS_SIZE-1:0] dcache_mem_address;
   logic [`DRAM_WORD_SIZE-1:0] 	  dcache_mem_data_out[`DRAM_BLOCK_SIZE-1:0];
   logic 			  dcache_mem_rw;
   logic 			  mem_rw;

   
  
   
   

	cpu cpu(.clk(clk), .reset(reset), .cache_miss_stall(cache_miss_stall),
                .icache_address(icache_address), .icache_valid(icache_valid), 
                .icache_data_out(icache_data_out), .dcache_data_out(dcache_data_out),
                .dcache_address(dcache_address), .dcache_data_in(dcache_data_in), 
                .dcache_byte_en(dcache_byte_en), .dcache_rw(dcache_rw), .dcache_valid(dcache_valid),
                .regwritememwb(regwritememwb));

	dcache_controller dcache_controller(.clk(clk), .reset(reset), .dcache_data_ready(dcache_data_ready), .dcache_data_out(dcache_data_out), 
        .dcache_valid(dcache_valid), .dcache_rw(dcache_rw), .dcache_byte_en(dcache_byte_en), .dcache_data_in(dcache_data_in), .dcache_address(dcache_address), 
        .mem_valid(dcahce_mem_valid), .mem_ready(dram_port2_acknowledge), .mem_data_in(dram_port2_read_data), .mem_address(dcache_mem_address),
        .mem_data_out(dcache_mem_data_out), .mem_rw(dcache_mem_rw));
	
       
	icache_controller icache_controller(.clk(clk), .reset(reset), .icache_address(icache_address), .icache_valid(icache_valid), 
        .icache_data_out(icache_data_out), .icache_data_ready(icache_data_ready), .mem_valid(icache_mem_valid), .mem_address(icache_mem_address),
        .mem_ready(dram_port1_acknowledge), .mem_data_in(dram_port1_read_data));
	/* verilator lint_off PINMISSING */
	dram_controller dram_controller(.clk(clk), .reset(reset), .dram_busy(dram_busy), .dram_port2_acknowledge(dram_port2_acknowledge),
        .dram_port2_read_data(dram_port2_read_data), .dram_port1_acknowledge(dram_port1_acknowledge), .dram_port1_read_data(dram_port1_read_data),
        .dram_port1_request(icache_mem_valid), .dram_port1_address(icache_mem_address), .dram_port2_request(dcahce_mem_valid), .dram_port2_address(dcache_mem_address), 
        .dram_port2_we(dcache_mem_rw), .dram_port2_write_data(dcache_mem_data_out));
        /* verilator lint_on PINMISSING */

endmodule

