// It is a simple byte-writeable main memory bus design with programmable 
// read and write (clock) latency, which has been developed by Ali Muhtaroglu 
// in support of education for Computer Architecture / Organization courses 
// at METU Northern Cyprus Campus.

`include "./config.sv"
`include "./constants.sv"


module dram (
		    
	     input 			      clk, reset,
	     input [`DRAM_ADDRESS_SIZE-1:0]   address,
	     inout wire [`DRAM_WORD_SIZE-1:0] data,
	     input 			      read_enable,
	     input 			      write_enable,
	     output 			      write_data_enable,
	     output 			      read_data_enable,
	     output 			      acknowledge
);

    /* verilator lint_off LITENDIAN */
    logic [(`DRAM_READ_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2:0] read_acknowledge_delay_line;
    logic [(`DRAM_WRITE_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2:0] write_acknowledge_delay_line;
    /* verilator lint_on LITENDIAN */
    logic int_read_acknowledge;
    logic int_write_acknowledge;

    `ifdef DRAM_WRITE_ACCESS_TIME
        assign write_data_enable =  write_acknowledge_delay_line[`DRAM_WRITE_ACCESS_TIME-`DRAM_CYCLE_TIME-2];
    `else
        assign write_data_enable = write_enable;
    `endif

    `ifdef DRAM_READ_ACCESS_TIME
        assign read_data_enable =  read_acknowledge_delay_line[`DRAM_READ_ACCESS_TIME-`DRAM_CYCLE_TIME+1];
    `else
        assign read_data_enable = read_enable;
    `endif
   
    dram_array dram_array (
        .clk      (clk),
	.address    (address),
        .data       (data),
        .wren       (write_data_enable)
    );

    // acknowledge logic for generating memory latency
   
    `ifdef DRAM_READ_ACCESS_TIME
       if((`DRAM_READ_ACCESS_TIME+`DRAM_BLOCK_SIZE) > 0) begin
	  always @(posedge clk) 
	  begin 
	       if (reset || !read_enable) 
		 read_acknowledge_delay_line <= 'b0; 
	       else 
		 begin
                    if((`DRAM_READ_ACCESS_TIME+`DRAM_BLOCK_SIZE) > 1) begin
		       /* verilator lint_off WIDTH */
                       /* verilator lint_off SELRANGE */ 
		       read_acknowledge_delay_line <= {read_acknowledge_delay_line[(`DRAM_READ_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2:0], read_enable};
		       /* verilator lint_on SELRANGE */
		       /* verilator lint_on WIDTH */
                    end else begin
		       /* verilator lint_off WIDTH */
		       read_acknowledge_delay_line <= read_enable;
		       /* verilator lint_on WIDTH */
		    end
		 end  
	  end // always @ (posedge clock or negedge read_enable)
          assign int_read_acknowledge = read_acknowledge_delay_line[(`DRAM_READ_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2];
       end else begin // if (`DRAM_READ_ACCESS_TIME > 0)
	  assign int_read_acknowledge = read_enable;
       end // else: !if(`DRAM_READ_ACCESS_TIME > 0)
    `else // !`ifdef DRAM_READ_LATENCY
       assign int_read_acknowledge = read_enable;
    `endif 

    `ifdef DRAM_WRITE_ACCESS_TIME
       if((`DRAM_WRITE_ACCESS_TIME+`DRAM_BLOCK_SIZE) > 0) begin
	  always @(posedge clk) 
	  begin 
	       if (reset || !write_enable) 
		 write_acknowledge_delay_line <= 'b0; 
	       else 
		 begin
                    if((`DRAM_WRITE_ACCESS_TIME+`DRAM_BLOCK_SIZE) > 1) begin
		       /* verilator lint_off WIDTH */
                       /* verilator lint_off SELRANGE */ 
		       write_acknowledge_delay_line <= {write_acknowledge_delay_line[(`DRAM_WRITE_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2:0], write_enable};
		       /* verilator lint_on WIDTH */
                       /* verilator lint_on SELRANGE */
                    end else begin
		       /* verilator lint_off WIDTH */
		       write_acknowledge_delay_line <= write_enable;
		       /* verilator lint_on WIDTH */
		    end
		 end  
	  end // always @ (posedge clock or negedge read_enable)
          assign int_write_acknowledge = write_acknowledge_delay_line[(`DRAM_WRITE_ACCESS_TIME+((`DRAM_BLOCK_SIZE-1)*`DRAM_CYCLE_TIME))-2];
       end else begin // if (`DRAM_WRITE_ACCESS_TIME > 0)
	  assign int_write_acknowledge = write_enable;
       end // else: !if(`DRAM_WRITE_ACCESS_TIME > 0)
    `else // !`ifdef DRAM_WRITE_ACCESS_TIME
       assign int_write_acknowledge = write_enable;
    `endif 

    assign acknowledge = int_read_acknowledge || int_write_acknowledge;
   
endmodule

