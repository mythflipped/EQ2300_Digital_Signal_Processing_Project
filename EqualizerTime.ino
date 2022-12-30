/*

    EQ2300 Digital Signal Processing Lab, 2017

    The code implements the Arduino Equalizer using time domain processing

*/

// Filter to be implemented on Arduino
#define FILTERLENGTH 51
#define FRACTIONALBITS 12
int Filter[] = {-78,-64,-38,-5,30,61,82,89,80,56,18,-26,-70,-107,-128,-128,-103,-51,24,117,221,325,420,496,545,562,545,496,420,325,221,117,24,-51,-103,-128,-128,-107,-70,-26,18,56,80,89,82,61,30,-5,-38,-64,-78};


// Program parameters
#define BUFSIZE 0x1000
#define BUFMASK 0xFFF
#define CANREAD() (((ADC->ADC_ISR & ADC_ISR_EOC7) && (ADC->ADC_ISR & ADC_ISR_EOC6)) ? true : false )
#define READ(chan) ( ((*(ADC->ADC_CDR + 7 - (chan))) & 0xFFF) - 0x800 )
#define READ_POT(num) ((*(ADC->ADC_CDR + 5 - num) & 0xFFF ))
#define WRITE(a, b) do { DACC->DACC_CDR = (( ((a) + 0x800) & 0xFFF) | (0x00 << 12) | (( ((b) + 0x800) & 0xFFF) << 16) | (0x01 << 28) ) ; } while ( DACC->DACC_ISR & DACC_ISR_TXRDY )

volatile int InputBufferL [BUFSIZE] ;
volatile int InputBufferR [BUFSIZE] ;
volatile int OutputBufferL [BUFSIZE] ;
volatile int OutputBufferR [BUFSIZE] ;

volatile int buffer_pos1 = 0;
volatile int buffer_pos2 = BUFSIZE / 2;
int ledPins [] = {52, 50, 48, 46, 44, 42, 40, 38} ;
int buttonPins [] = {34, 36} ;

boolean lastButtons [] = {true, true} ;
boolean currentButtons [] = {true, true} ;
volatile int state = 0 ;
int lastTime = 0;
volatile int isr_count = 0;

// State variables
volatile int LastL = 0;
volatile int LastA = 0;

void setup() {
  Serial.begin(57600);

  //Serial.begin(115200); // Activate the serial port

  int32_t mask_PWM_pin = digitalPinToBitMask(6);
  REG_PMC_PCER1 = 1 << 4;        // activate clock for PWM controller
  REG_PIOC_PDR |= mask_PWM_pin;  // activate peripheral functions for pin (disables all PIO functionality)
  REG_PIOC_ABSR |= mask_PWM_pin; // choose peripheral option B
  REG_PWM_CLK = 0;               // choose clock rate, 0 -> full MCLK as reference 84MHz
  REG_PWM_CMR7 = 0 << 9;         // select clock and polarity for PWM channel (pin7) -> (CPOL = 0)
  REG_PWM_CPRD7 = 84;            // initialize PWM period -> T = value/84MHz (value: up to 16bit), value=8 -> 10.5MHz
  REG_PWM_CDTY7 = 42;            // initialize duty cycle, REG_PWM_CPRD6 / value = duty cycle, for 8/4 = 50%
  REG_PWM_ENA = 1 << 7;          // enable PWM on PWM channel (pin 7 = PWML6)

  for (int j = 0; j < BUFSIZE; j++) {
    InputBufferL [j] = 0 ;
    InputBufferR [j] = 0 ;
    OutputBufferL [j] = 0;
    OutputBufferR [j] = 0;
  }
  for (int j = 0; j < 8; j++ ) {
    pinMode (ledPins [j], OUTPUT) ;
  }


  for (int j = 0; j < 2; j++ ) {
    pinMode (buttonPins [j], INPUT) ;
    digitalWrite (buttonPins [j], HIGH) ;
    attachInterrupt (digitalPinToInterrupt(buttonPins [j]), buttonPressed, CHANGE) ;
  }
  digitalWrite(ledPins[0], HIGH);
  for ( int i = 1; i < 8; i++) digitalWrite(ledPins[i], LOW); // Set pins to low

  pinMode(9, OUTPUT);
  pinMode(2, OUTPUT);

  configure() ;

}

void loop() {
}


volatile int uval0 = 0;
volatile int uval1 = 0;


//#pragma GCC push_options
//#pragma GCC optimize ("O3")
void FilterFun(int *a, int *b, int *buf0, int *buf1) {
  int ufval0 = 0, ufval1 = 0;
  for (int n; n < FILTERLENGTH; n++) {
    ufval0 += buf0[n] * Filter[n];
    ufval1 += buf1[n] * Filter[n];
  }
  *a = ufval0;
  *b = ufval1;
}
//#pragma GCC pop_options

#ifdef __cplusplus
extern "C"
{
#endif

void ADC_Handler (void) {
  if (CANREAD())  {
    REG_PIOB_SODR |= (0x01 << 25); // Set right debug pin high
    int s = state;
    int val0 = READ(0);
    int val1 = READ(1);
    int ax2 = buffer_pos1;
    int ax = buffer_pos2;
    int ufval0 = 0;
    int ufval1 = 0;
    int nn = 0;

    // Read from ADCs
    val0 = READ(0);
    val1 = READ(1);
    // Write to DACs (last values computed)
    WRITE(uval0, uval1);

    InputBufferL [ax] = val0;
    InputBufferL [ax + BUFSIZE / 2] = val0;
    InputBufferR [ax] = val1;
    InputBufferR [ax + BUFSIZE / 2] = val1;

    int *buf0 = (int*) &InputBufferL[ax];
    int *buf1 = (int*) &InputBufferR[ax];
    int lval0, lval1, hval0, hval1;

    switch (s) {
      case 0: // Pass the signal through
        uval0 = val0;
        uval1 = val1;
        break;

      case 1: // Compute the output of the low pass filter
        FilterFun(&ufval0,&ufval1,buf0,buf1);
        uval0 = ufval0 >> FRACTIONALBITS;
        uval1 = ufval1 >> FRACTIONALBITS;

        break;

      case 2: // Compute the output of the high pass filter
        FilterFun(&ufval0,&ufval1,buf0,buf1);
        uval0 = buf0[(FILTERLENGTH - 1) >> 1] - (ufval0 >> FRACTIONALBITS);
        uval1 = buf1[(FILTERLENGTH - 1) >> 1] - (ufval1 >> FRACTIONALBITS);
        break;

      case 3: // Implement the whole equalizer
        FilterFun(&ufval0,&ufval1,buf0,buf1);
        lval0 = ufval0 >> FRACTIONALBITS;
        lval1 = ufval1 >> FRACTIONALBITS;
        hval0 = buf0[(FILTERLENGTH - 1) >> 1] - lval0;
        hval1 = buf1[(FILTERLENGTH - 1) >> 1] - lval1;
        int pot0 = READ_POT(0);
        int pot1 = READ_POT(1);
        uval0 = (lval0 * pot0 + hval0 * pot1) >> 12;
        uval1 = (lval1 * pot0 + hval1 * pot1) >> 12;
        break;

    }

    buffer_pos1 = (ax2 + 1) & BUFMASK;
    buffer_pos2 = (ax - 1) & (BUFMASK >> 1) ;
    isr_count ++;
    REG_PIOB_CODR |= (0x01 << 25); // Set right debug pin low
  }
}

#ifdef __cplusplus
}
#endif
void buttonPressed() {
  currentButtons[0] = digitalRead (buttonPins[0]) ;
  currentButtons[1] = digitalRead (buttonPins[1]) ;

  if ( ((lastButtons[0] & !currentButtons[0]) ^ (lastButtons[1] & !currentButtons[1])) && ((millis() - lastTime) > 50)) {

    // Turn off old led for state
    digitalWrite(ledPins[state], LOW) ;

    if (currentButtons[1]) {
      if (state != 7) state ++ ;
      else state = 0 ;
    } else if (currentButtons[0]) {
      if (state != 0) state-- ;
      else state = 7 ;
    }

    // Turn on new led for state
    digitalWrite(ledPins[state], HIGH) ;

    Serial.print("State : ");
    Serial.println(state);
  }
  lastTime = millis();
  lastButtons[0] = currentButtons[0] ;
  lastButtons[1] = currentButtons[1] ;
}

void configure() {
  adc_setup() ;
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0 * 3 + 0) ; //clock the TC0 channel 0
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

  t->TC_RC =  875 ;  // counter resets on RC, so sets period in terms of 42MHz clock (48kHz sampling frequency)
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
