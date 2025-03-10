// Adder top level verilator simulation testbench (.h) file for standard module
// descriptions - one will not have to change this file very often: 
// EEE 446 Spring 2021
// Ali Muhtaroglu, Middle East Technical University - Northern Cyprus Campus

#include <stdio.h>
#include <stdint.h>
#include <verilated_vcd_c.h>

#define	TBASSERT(TB,A) do { if (!(A)) { (TB).closetrace(); } assert(A); } while(0);

template <class MODULE> class TESTBENCH {
 public:
  MODULE	  *m_topsim;
  VerilatedVcdC*  m_trace;
  uint64_t        m_tickcount;

  TESTBENCH(void) : m_trace(NULL), m_tickcount(0l) {
    m_topsim = new MODULE;
    Verilated::traceEverOn(true);
    m_topsim->clk = 0;
    // Get our initial values set properly.
    eval();
  }

  virtual ~TESTBENCH(void) {
    closetrace();
    delete m_topsim;
    m_topsim = NULL;
  }

  virtual void opentrace(const char *vcdname) {
    if (!m_trace) {
      m_trace = new VerilatedVcdC;
      m_topsim->trace(m_trace, 99);
      m_trace->open(vcdname);
    }
  }

  virtual void closetrace(void) {
    if (m_trace) {
      m_trace->close();
      delete m_trace;
      m_trace = NULL;
    }
  }

  virtual void eval(void) {
    m_topsim->eval();
  }

  virtual void	tick(void) {
    // Increment our own internal time reference
    m_tickcount++;

    // Make sure any combinatorial logic depending upon
    // inputs that may have changed before we called tick()
    // has settled before the rising edge of the clock.
    eval();
    if (m_trace) m_trace->dump((vluint64_t)(10*m_tickcount-2));
    // Toggle the clock
    // Rising edge
    m_topsim->clk = 1;
    eval();
    if (m_trace) m_trace->dump((vluint64_t)(10*m_tickcount));
    // Falling edge
    m_topsim->clk = 0;
    eval();
    if (m_trace) {
      m_trace->dump((vluint64_t)(10*m_tickcount+5));
      m_trace->flush();
    }
  }
  
  virtual void	reset(void) {
    m_topsim->reset = 1;
    // Reset for one clock cycle
    tick();
    eval();
    m_topsim->reset = 0;
    eval();
  }

  unsigned long	tickcount(void) {
    return m_tickcount;
  }

};

