// This is a common configuration file for SystemVerilog macros to support the development
// in  Computer Architecture / Organization courses at METU Northern Cyprus Campus.
// Ali Muhtaroglu
// 

//////////////////////////////////////////
//              DRAM config             //
//////////////////////////////////////////

// DRAM address and data bus/word size in bits
`define DRAM_ADDRESS_SIZE   12
`define DRAM_WORD_SIZE      32

// Definition of DRAM Block Size in terms of 
// # of Data Words. Block size is assumed same 
// for the full memory hierarchy.
`define DRAM_BLOCK_SIZE           4

// Definition of DRAM latencies in clock cycles
`define DRAM_READ_ACCESS_TIME    16
`define DRAM_WRITE_ACCESS_TIME   24
`define DRAM_CYCLE_TIME           4     

// Definition of DRAM file
`define DRAM_HEX  "simple_test.hex.dat"

//////////////////////////////////////////
//              I-CACHE config          //
//////////////////////////////////////////

// CACHE index and data bus sizes for direct-mapped cache
`define ICACHE_INDEX       4

// number of blocks in cache
`define ICACHE_SIZE        2**`ICACHE_INDEX 

`define ITAGMSB            `DRAM_ADDRESS_SIZE-1  //tag msb
`define ITAGLSB            8  //tag lsb

//////////////////////////////////////////
//              D-CACHE config          //
//////////////////////////////////////////

// CACHE index and data bus sizes for direct-mapped cache
`define DCACHE_INDEX       5

// number of blocks in cache
`define DCACHE_SIZE        2**`DCACHE_INDEX 

`define DTAGMSB            `DRAM_ADDRESS_SIZE-1  //tag msb
`define DTAGLSB            9  //tag lsb
