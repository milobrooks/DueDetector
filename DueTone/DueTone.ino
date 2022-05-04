

/*
 Tone generator
 v1  use timer, and toggle any digital pin in ISR
   funky duration from arduino version
   TODO use FindMckDivisor?
   timer selected will preclude using associated pins for PWM etc.
    could also do timer/pwm hardware toggle where caller controls duration
*/


// timers TC0 TC1 TC2   channels 0-2 ids 0-2  3-5  6-8     AB 0 1
// use TC1 channel 0 
#define TONE_TIMER TC1
#define TONE_CHNL 0
#define TONE_IRQ TC3_IRQn
#define NUM_SAMPLES 25
#define SAMPLE_RATE 26250

// TIMER_CLOCK4   84MHz/128 with 16 bit counter give 10 Hz to 656KHz
//  piano 27Hz to 4KHz

/*
  Melody on pin 8 of DUE
 
 http://arduino.cc/en/Tutorial/Tone
 
 */

void setup() {
  analogWriteResolution(12);  // set the analog output resolution to 12 bit (4096 levels)
  tone(DAC1, SAMPLE_RATE);
}

void loop() {
}


static uint8_t pinEnabled[PINS_COUNT];
static uint8_t TCChanEnabled = 0;
static boolean pin_state = false ;
static Tc *chTC = TONE_TIMER;
static uint32_t chNo = TONE_CHNL;
static uint32_t tone_pin;
static uint32_t sample[ NUM_SAMPLES ] = { 2048,2557,3034,3449,3777,3995,4091,4059,3901,3626,3251,2801,2304,1791,1294,844,469,194,36,4,100,318,646,1061,1538 } ;
volatile static uint8_t s_index = 0;

// sample rate (in hertz)).

void tone(uint32_t ulPin, uint32_t sr)
{
    const uint32_t rc = VARIANT_MCK / 128 / sr; 
    tone_pin = ulPin;

    if (!TCChanEnabled) {
      pmc_set_writeprotect(false);
      pmc_enable_periph_clk((uint32_t)TONE_IRQ);
      TC_Configure(chTC, chNo,
        TC_CMR_TCCLKS_TIMER_CLOCK4 |
        TC_CMR_WAVE |         // Waveform mode
        TC_CMR_WAVSEL_UP_RC ); // Counter running up and reset when equals to RC
  
      chTC->TC_CHANNEL[chNo].TC_IER=TC_IER_CPCS;  // RC compare interrupt
      chTC->TC_CHANNEL[chNo].TC_IDR=~TC_IER_CPCS;
       NVIC_EnableIRQ(TONE_IRQ);
                         TCChanEnabled = 1;
    }
    if (!pinEnabled[ulPin]) {
      pinMode(ulPin, OUTPUT);
      pinEnabled[ulPin] = 1;
    }
    TC_Stop(chTC, chNo);
                TC_SetRC(chTC, chNo, rc);    // set frequency
    TC_Start(chTC, chNo);
}

void noTone(uint32_t ulPin)
{
  TC_Stop(chTC, chNo);  // stop timer
  digitalWrite(ulPin,LOW);  // no signal on pin
}

// timer ISR  TC1 ch 0
void TC3_Handler ( void ) {
  TC_GetStatus(TC1, 0);

  analogWrite(DAC1, sample[s_index]);  // write the selected waveform on DAC0
  s_index++;
  if(s_index == NUM_SAMPLES) s_index = 0; // Reset the counter to repeat the wave
}
