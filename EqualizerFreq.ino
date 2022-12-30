/*

    EQ2300 Digital Signal Processing Lab, 2017

    The code implements the Arduino Equalizer using overlap save

*/

// Filter to be implemented on Arduino
#define FILTERLENGTH 51
#define FRACTIONALBITS 12
int Filter[] = {-78,-64,-38,-5,30,61,82,89,80,56,18,-26,-70,-107,-128,-128,-103,-51,24,117,221,325,420,496,545,562,545,496,420,325,221,117,24,-51,-103,-128,-128,-107,-70,-26,18,56,80,89,82,61,30,-5,-38,-64,-78};


#define BUFSIZE 0x1000
#define BUFMASK 0xFFF
#define CANREAD() (((ADC->ADC_ISR & ADC_ISR_EOC7) && (ADC->ADC_ISR & ADC_ISR_EOC6)) ? true : false )
#define READ(chan) ( ((*(ADC->ADC_CDR + 7 - (chan))) & 0xFFF) - 0x800 )
#define WRITE(a, b) do { DACC->DACC_CDR = (( ((a) + 0x800) & 0xFFF) | (0x00 << 12) | (( ((b) + 0x800) & 0xFFF) << 16) | (0x01 << 28) ) ; } while ( DACC->DACC_ISR & DACC_ISR_TXRDY )
#define READ_POT(num) ((*(ADC->ADC_CDR + 5 - num) & 0xFFF ))

// FFT parameters
#define FFT_SIZE 256 // Must also set N_STAGES appropriately
#define N_STAGES 8
#define BLOCK_LENGTH (FFT_SIZE-FILTERLENGTH+1)
#define DELAY (2*BLOCK_LENGTH)
#define NPI 3.141592653589793

#define TEST_REP 200

struct complex_int {
   int  re;
   int  im;
};

// Input buffers
int InputBufferL [BUFSIZE];
int InputBufferR [BUFSIZE];

// Output buffers
int OutputBufferL [BUFSIZE];
int OutputBufferR [BUFSIZE];

struct complex_int W[FFT_SIZE/2];

// Button stuff
int ledPins [] = {52, 50, 48, 46, 44, 42, 40, 38} ;
int buttonPins [] = {34, 36} ;
boolean lastButtons [] = {true, true} ; 
boolean currentButtons [] = {true, true} ;
volatile int state = 0 ;
int lastTime = 0;
volatile int isr_count = 0;

// Counters for buffer handling
unsigned int buffer_pos = 0;
unsigned int block_size = 0;
unsigned int block_start;
volatile boolean block_ready = false;

// FFT time and frequency blocks
int fft_reorder[FFT_SIZE];
struct complex_int block_A[FFT_SIZE];
struct complex_int freq_LowPass[FFT_SIZE];

// Filter
int LowPassL = 41;
int LowPass_re[64];
float f_LowPass[] = {0.0065634,0.0084631,0.010436,0.012465,0.014532,0.016615,0.018696,0.020752,0.022762,0.024706,0.026562,0.028311,0.029932,0.031409,0.032725,0.033865,0.034816,0.035567,0.03611,0.036438,0.036548,0.036438,0.03611,0.035567,0.034816,0.033865,0.032725,0.031409,0.029932,0.028311,0.026562,0.024706,0.022762,0.020752,0.018696,0.016615,0.014532,0.012465,0.010436,0.0084631,0.0065634};
int LowPass_im[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int freq_LowPass_re [FFT_SIZE];
int freq_LowPass_im [FFT_SIZE];

void setup() {
  //Serial.begin(115200);
  //Serial.println("Hello world");
   
  int32_t mask_PWM_pin = digitalPinToBitMask(6);
  REG_PMC_PCER1 = 1<<4;               // activate clock for PWM controller
  REG_PIOC_PDR |= mask_PWM_pin;  // activate peripheral functions for pin (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_PWM_pin; // choose peripheral option B    
  REG_PWM_CLK = 0;                     // choose clock rate, 0 -> full MCLK as reference 84MHz
  REG_PWM_CMR7 = 0<<9;             // select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
  REG_PWM_CPRD7 = 84;                // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=8 -> 10.5MHz
  REG_PWM_CDTY7 = 42;                // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 8/4 = 50%
  REG_PWM_ENA = 1<<7;               // enable PWM on PWM channel (pin 7 = PWML6)

  // Fix weight table
  for(int i = 0; i< FFT_SIZE/2; i++) {
    W[i].re = cos(2*NPI*i/FFT_SIZE) * 0x10000;
    W[i].im = sin(2*NPI*i/FFT_SIZE) * 0x10000;    
  } 
  
  // Fix FFT reordering
  for(int i = 0; i< FFT_SIZE; i++) {
    int n = i; int m=0;
    for(int j=0; j < N_STAGES; j++) {
      m = (m << 1) + (n & 0x01);
      n >>= 1;
    }
    //Serial.println(m);
    fft_reorder[i] = m;
  }

  // Fix filter
  for(int i = 0; i < FFT_SIZE; i++) {
    freq_LowPass[i].re = 0;
    freq_LowPass[i].im = 0;
  }
  for(int i = 0; i < FILTERLENGTH; i++) {
    freq_LowPass[i].re = Filter[i];
  }
  // FFT of filter
  fft(freq_LowPass);
    
  for (int j = 0; j < 8; j++ ) {
    pinMode (ledPins [j], OUTPUT) ; 
  }

  // Uncomment to enable push button interrupt
  //for (int j = 0; j < 2; j++ ) {
  //  pinMode (buttonPins [j], INPUT) ; 
  //  digitalWrite (buttonPins [j], HIGH) ; 
  //  attachInterrupt (digitalPinToInterrupt(buttonPins [j]), buttonPressed, CHANGE) ;
  //}

  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);   
  configure();

  // Set all leds to low
  for( int i=0; i<8; i++) digitalWrite(ledPins[i], LOW);
}

void loop() {
  int pos, posH;
  if (block_ready) {
    REG_PIOB_SODR |= (0x01 << 25); // Mark start
    block_ready = false;

    // Copy from input ring buffer to FFT block
    pos = block_start & BUFMASK;
    for(int k=0; k < FFT_SIZE; k++) {
      block_A[k].re = InputBufferL[pos];
      block_A[k].im = InputBufferR[pos];
      pos = (pos + 1) & BUFMASK;
    }

    // Compute FFT to get into the frequency domain
    fft(block_A);

    // Filter in frequency domain    
    complex_int t;
    if(state == 0) {
      for(int i=0; i < FFT_SIZE; i++) {
        t.re = (((long long) block_A[i].re)*((long long) freq_LowPass[i].re) - ((long long) block_A[i].im)*((long long) freq_LowPass[i].im)) >> FRACTIONALBITS;
        t.im = (((long long) block_A[i].re)*((long long) freq_LowPass[i].im) + ((long long) block_A[i].im)*((long long) freq_LowPass[i].re)) >> FRACTIONALBITS;
        block_A[i].re = t.re;
        block_A[i].im = t.im;
      }
    } else {
      for(int i=0; i < FFT_SIZE; i++) { // No filtering for comparison
        t.re = block_A[i].re;
        t.im = block_A[i].im;
        block_A[i].re = t.re;
        block_A[i].im = t.im;
      }       
    }

    // Compute IFFT to get back into time domain
    ifft(block_A);    


    // Combine equalizer function and save operation from overlap save
    int pot0 = READ_POT(0);
    int pot1 = READ_POT(1);
    pos = (block_start+FILTERLENGTH-1) & BUFMASK;
    posH = (pos- ((FILTERLENGTH - 1) >> 1)) & BUFMASK;
    for(int k=FILTERLENGTH-1; k < FFT_SIZE; k++) {
      int xLL = block_A[k].re;
      int xLR = block_A[k].im;
      int xHL = InputBufferL[posH]-xLL;
      int xHR = InputBufferR[posH]-xLR;
      OutputBufferL[pos] = (xLL * pot0 + xHL * pot1) >> 12;
      OutputBufferR[pos] = (xLR * pot0 + xHR * pot1) >> 12;
      pos = (pos + 1) & BUFMASK;
      posH = (posH + 1) & BUFMASK;   
    }

    REG_PIOB_CODR |= (0x01 << 25);
  }
}
      
#ifdef __cplusplus
extern "C"
{
#endif
void ADC_Handler (void) {
  if (CANREAD()) {
    int val0 = READ(1);
    int val1 = READ(0);

    unsigned int out_pos = (buffer_pos - DELAY) & BUFMASK;
    WRITE(OutputBufferL[out_pos], OutputBufferR[out_pos]);

    InputBufferL [buffer_pos] = val0;
    InputBufferR [buffer_pos] = val1;

    block_size++;
    
    if (block_size == BLOCK_LENGTH) {
      block_start = (buffer_pos - FFT_SIZE) & BUFMASK;
      block_ready = true;
      block_size = 0;
    }
    buffer_pos = (buffer_pos + 1) & BUFMASK;
  }
}
}

void buttonPressed() {
  currentButtons[0] = digitalRead (buttonPins[0]) ; 
  currentButtons[1] = digitalRead (buttonPins[1]) ;
  
  if ( ((lastButtons[0] & !currentButtons[0]) ^ (lastButtons[1] & !currentButtons[1])) && ((millis() - lastTime) > 50)) { 
    
    if (state != 0) digitalWrite(ledPins[state - 1], LOW) ;

    if (currentButtons[1]) { 
      if (state != 8) state ++ ;
      else state = 0 ;
    } else if (currentButtons[0]) { 
      if (state != 0) state-- ;
      else state = 8 ;
    }

    if (state != 0) digitalWrite(ledPins[state - 1], HIGH) ; 

    Serial.print("State : ");
    Serial.println(state);
  }
  lastTime = millis();
  lastButtons[0] = currentButtons[0] ; 
  lastButtons[1] = currentButtons[1] ;
}

void configure() {
  adc_setup() ;
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3 + 0) ; //clock the TC0 channel 0
  TcChannel * t = &(TC0->TC_CHANNEL)[0] ;  // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;  // disable interrupts
  t->TC_SR ;  // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |  // use TCLK1 (prescale by 2, = 42MHz)
        TC_CMR_WAVE |  // waveform mode
        TC_CMR_WAVSEL_UP_RC |  // count-up PWM using RC as threshold
        TC_CMR_EEVT_XC0 |  // Set external events from XC0 (this setup TIOB as output)
        TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
        TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
  
  t->TC_RC =  875 ;  // counter resets on RC, so sets period in terms of 42MHz clock (47kHz sampling frequency)
  t->TC_RA =  440 ;  // roughly square wave
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source
  dac_setup() ;
  }
  
void adc_setup() {
  NVIC_EnableIRQ (ADC_IRQn) ; // enable ADC interrupt vector
  ADC->ADC_IDR = 0xFFFFFFFF ; // disable interrupts
  ADC->ADC_IER = 0x80 ; // enable End-Of-Conv interrupt for AD7
  ADC->ADC_CHDR = 0xFFFF ; // disable all channels
  ADC->ADC_CHER = 0xF0 ; // enable channels AD7 and AD6
  //ADC->ADC_CGR = 0x15555555 ; // set channel gains to 1
  ADC->ADC_CGR = 0x1555A555 ; // set channel gains to 2
  //ADC->ADC_CGR = 0x1555F555 ; // set channel gains to 4
  ADC->ADC_COR = 0x000000C0 ; // set channel offsets

  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN | ADC_MR_ANACH ;  // 1 = trig source TIO from TC0
  
}

void dac_setup() {
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  DACC->DACC_CR = DACC_CR_SWRST ; // reset DAC
  DACC->DACC_MR = DACC_MR_TAG | DACC_MR_WORD | DACC_MR_REFRESH (0x0F) | (24 << DACC_MR_STARTUP_Pos) ; // configure DACC Mode Register
  DACC->DACC_IDR = 0xFFFFFFFF ; // disable all interrupts
  DACC->DACC_CHER = 0x3 ; // enable DAC0 and DAC1
}


//#pragma GCC push_options
//#pragma GCC optimize ("O3")
// FFT Decimation-in-Frequency
void fft(struct complex_int *pfs) {

  complex_int a,at,b,bt,t,w;

  for(int stage = 0; stage < N_STAGES; stage++) {

    // Stage parameters
    int blocks = 1 << stage;
    int bsize = FFT_SIZE >> (stage);
    int bstride = FFT_SIZE >> (stage+1);

    for(int block = 0; block < blocks; block ++) {

      // Data pointers
      complex_int *pfs_pa = &pfs[block*bsize];
      complex_int *pfs_pb = &pfs[block*bsize+bstride];
      struct complex_int *Wp = W;

      for(int k = 0; k < bstride; k++) {

        // Weight factors
        w = *Wp;
        Wp += blocks;
        
        // Load data
        a = *pfs_pa;
        b = *pfs_pb;       

        // Butterfly additions (before addition)
        at.re = a.re + b.re;
        at.im = a.im + b.im;
        *(pfs_pa++) = at;     
        
        bt.re = a.re - b.re;
        bt.im = a.im - b.im;

        // Butterfly multiplication (after addition)
        t.re = (((long long) w.re)*((long long) bt.re) + ((long long) w.im)*((long long) bt.im)) >> 16;
        t.im = (((long long) w.re)*((long long) bt.im) - ((long long) w.im)*((long long) bt.re)) >> 16;
        
        // Save data
        *(pfs_pb++) = t;      
        
      } 
    } 
  }
}

// IFFT Decimation-in-Time
void ifft(struct complex_int *pfs) {

  complex_int a,at,b,bt,t,w;

  for(int stage = 0; stage < N_STAGES; stage++) {

    // Stage parameters
    int blocks = FFT_SIZE >> (stage + 1);
    int bsize = 2 << stage;
    int bstride = 1 << stage;

    for(int block = 0; block < blocks; block ++) {

      // Data pointers
      complex_int *pfs_pa = &pfs[block*bsize];
      complex_int *pfs_pb = &pfs[block*bsize+bstride];
      struct complex_int *Wp = W;

      for(int k = 0; k < bstride; k++) {

        // Weight factors
        w = *Wp;
        Wp += blocks;

        // Load data
        a = *pfs_pa;
        b = *pfs_pb;       

        // Butterfly multiplication
        t.re = (((long long) w.re)*((long long) b.re) - ((long long) w.im)*((long long) b.im)) >> 16;
        t.im = (((long long) w.re)*((long long) b.im) + ((long long) w.im)*((long long) b.re)) >> 16;

        // Butterfly additions
        at.re = a.re + t.re;
        at.im = a.im + t.im;
        bt.re = a.re - t.re;
        bt.im = a.im - t.im;

        // Shift down for division by N
        //at.re >>= 1;
        //at.im >>= 1;
        //bt.re >>= 1;
        //bt.im >>= 1;
  
        // Save data
        *(pfs_pa++) = at;
        *(pfs_pb++) = bt;        
      } 
    } 
  }
  for(int k=0; k < FFT_SIZE; k++) { // Shift down by N
    pfs[k].re >>= N_STAGES;
    pfs[k].im >>= N_STAGES;    
  }
}
//#pragma GCC pop_options
