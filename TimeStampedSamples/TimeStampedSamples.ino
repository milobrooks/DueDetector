

/*
 
 Time stamped record output
 
 use software timer, and save data with time stamp
 
 This example only samples one channel using the Due's onboard ADC
 
 To do: adapt sketch to sample ADS1115 at 1 Hz with three single ended channels
 sampled per second
 
 To do: add in DMA sine wave generation

*/


// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0 
#define REC_TIMER TC1
#define REC_CHNL 0
#define REC_IRQ TC3_IRQn
#define SAMPLE_RATE 1
#define NUM_SAMPLES 25
#define OUTPUT_PIN 5

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz

static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = REC_TIMER;
static uint32_t chNo = REC_CHNL;
// static uint32_t sample[ NUM_SAMPLES ] = { 2048,2557,3034,3449,3777,3995,4091,4059,3901,3626,3251,2801,2304,1791,1294,844,469,194,36,4,100,318,646,1061,1538 } ;
volatile static int32_t s_index = 0;
static int32_t prev_s_index = -1;
volatile static int16_t sensorValue;


void setup() {
    const uint32_t rc = VARIANT_MCK / 128 / SAMPLE_RATE; 
    
    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)REC_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
      NVIC_EnableIRQ(REC_IRQ);
      TCChanEnabled = 1;
    }
    pinMode(OUTPUT_PIN, OUTPUT);
    TC_Stop(chTC, chNo);
    TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
    Serial.begin(9600);
}

void loop() {

  while (prev_s_index == s_index) {
    delay(10);
  }
  
  Serial.print("s_index = ");
  Serial.print(s_index);
  Serial.print(", A0 = ");
  Serial.println(sensorValue);
  prev_s_index = s_index;
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);

  //toggle OUTPUT_PIN so we can check timing on oscilloscope
  digitalWrite(OUTPUT_PIN, pin_state);
  pin_state = !pin_state;
  
  // read the input on analog pin 0:
  sensorValue = analogRead(A0);
  
  // increase the seconds count
  s_index++;
}
