// Adder top level verilator simulation file: 
// EEE 446 Spring 2021
// Ali Muhtaroglu, Middle East Technical University - Northern Cyprus Campus

#include <stdio.h>
#include <verilated.h>
#include <verilated_vcd_c.h>
#include "testbench.h"
#include "Vtop.h"

// Top level interface signals defined here:

// Internal signals defined here:
// Note systemverilog design hierarchy can be traced by appending __DOT__ at every level:
#define Op                 top__DOT__Op
#define MemRead            top__DOT__MemRead
#define MemWrite           top__DOT__MemWrite
#define Memtoreg           top__DOT__Memtoreg
#define ALUSrc             top__DOT__ALUSrc
#define RegWrite           top__DOT__RegWrite
#define Jump               top__DOT__Jump
#define Branch             top__DOT__Branch
#define ExMemRegWrite      top__DOT__ExMemRegWrite
#define MemWbRegWrite      top__DOT__MemWbRegWrite
#define ExMemSecond_alu_en      top__DOT__ExMemSecond_alu_en
#define ForwardA                top__DOT__ForwardA
#define ForwardB                top__DOT__ForwardB
#define ForwardE                top__DOT__ForwardE
#define Instruction             top__DOT__Instruction
#define IMM                     top__DOT__IMM

// In case you would like he simulator to do operations conditional to DEBUG mode:
#define DEBUG       1

// Note the use of top level design name here after 'V' as class type:
class	TOPLEVEL_TB : public TESTBENCH<Vtop> {

  long m_tickcount;

public:

  TOPLEVEL_TB(void) {
  }

  // Every time this procedure is called, clock is ticked once and associated
  // simulation tests and/or signal outputs for debug can be put into action:
  void tick(void) {

    TESTBENCH<Vtop>::tick();

    // we are often interested in keeping track of number of clock ticks:
    m_tickcount++;
    /////
    if (DEBUG) {
      printf("%08lx-ADDER: ", m_tickcount);
   }   
  }
};    

int main(int argc, char** argv, char** env){
  // input and output numbers to help with testing:


  
  // some parameters we may want to keep track of:
  long clock_count = 0;
//long error_count = 0;
//bool test_pass = false;

  // Initialize Verilators variables
  Verilated::commandArgs(argc, argv);
  // Create an instance of our module under test
  TOPLEVEL_TB *tb = new TOPLEVEL_TB;

  // Message to standard output that test is starting:
  //printf("Executing the test ...\n");

  // Data will be dumped to trace file in gtkwave format to look at waveforms later:
  tb->opentrace("top.vcd");

  // Note this message will only be output if we are in DEBUG mode:
  if (DEBUG) printf("Giving the system 1 cycle to initialize with reset...\n");
  

  // Hit that reset button for one clock cycle:
  tb->reset();
  clock_count++;
  //tb->m_topsim->RF = RF_WriteData;      
  
  // Change the inputs if you wish, check the outputs if you wish, and
  // tick the clock until done:

  // first enable register updates:

  // verify the unsigned adder operation using 10000 random numbers for inputs:
  if (DEBUG) printf("Running verification ...\n");
  for (int k = 0; k < 8000 ; k++){
    tb->tick();
    clock_count++;
    // check for any functional bugs; but skip first iteration to allow data to propagate:
    //if ((tb->m_topsim->RF != 264198)  & (k > 0)){
    // error_count++;
    //}
    //if (DEBUG){
    //printf("Clock: %08lx ::  PrevA: %04x :: PrevB: %04x :: PrevRes: %04x ::   result: %04x\n",clock_count,Aprev,Bprev,resultprev,tb->m_topsim->result);    
    // }
    //Aprev = A;
    //Bprev = B;
    //resultprev = Aprev * Bprev ;
  }

  //test_pass = (error_count > 0) ? 0 : 1; 
printf("Execution completed successfully (simulation waveforms in .vcd file) ... !\n");
printf("Elapsed Clock Cycles: %ld\n",clock_count);
//printf("Functional verification Status: %s\n",test_pass?"PASS":"FAIL");
//if (error_count > 0){
//  printf("Error count is: %ld\n",error_count);
//}
  exit(EXIT_SUCCESS);
  
}
