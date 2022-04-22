#define    NWAVE               80

uint16_t  Sinewave[2][NWAVE] = {
  {
   +4095,   +4093,   +4089,   +4081,   +4070,   +4056,   +4038,   +4018,   +3995,   +3968,   +3939,   +3907,   +3872,   +3834,   +3793,   +3750,
   +3704,   +3656,   +3605,   +3551,   +3495,   +3438,   +3377,   +3315,   +3251,   +3185,   +3118,   +3048,   +2977,   +2905,   +2831,   +2757,
   +2681,   +2604,   +2526,   +2447,   +2368,   +2289,   +2209,   +2128,   +2048,   +1968,   +1887,   +1807,   +1728,   +1649,   +1570,   +1492,
   +1415,   +1339,   +1265,   +1191,   +1119,   +1048,    +978,    +911,    +845,    +781,    +719,    +658,    +601,    +545,    +491,    +440,
    +392,    +346,    +303,    +262,    +224,    +189,    +157,    +128,    +101,     +78,     +58,     +40,     +26,     +15,      +7,      +3,
  },
  {
      +1,      +3,      +7,     +15,     +26,     +40,     +58,     +78,    +101,    +128,    +157,    +189,    +224,    +262,    +303,    +346,
    +392,    +440,    +491,    +545,    +601,    +658,    +719,    +781,    +845,    +911,    +978,   +1048,   +1119,   +1191,   +1265,   +1339,
   +1415,   +1492,   +1570,   +1649,   +1728,   +1807,   +1887,   +1968,   +2048,   +2128,   +2209,   +2289,   +2368,   +2447,   +2526,   +2604,
   +2681,   +2757,   +2831,   +2905,   +2977,   +3048,   +3118,   +3185,   +3251,   +3315,   +3377,   +3438,   +3495,   +3551,   +3605,   +3656,
   +3704,   +3750,   +3793,   +3834,   +3872,   +3907,   +3939,   +3968,   +3995,   +4018,   +4038,   +4056,   +4070,   +4081,   +4089,   +4093
  }
};

uint16_t  SineFast[2][NWAVE] = {
  {
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939
  },
  {
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939,
   +4095,   +3939,   +3495,   +2831,   +2048,   +1265,    +601,    +157,      +1,    +157,    +601,   +1265,   +2048,   +2831,   +3495,   +3939
  }
};

            int       fast_mode  =    0;
            int       freq_inhz  = 1000; // Default Hz
            int       freq_intc  =    0; 
            int       user_intf  =    0;

volatile   uint16_t   sptr       =    0;
int input = A0;
int led = 13;
int val;

void setup()
{
  Serial.begin (115200); 
  dac_setup();        
  freq_intc = freqToTc(freq_inhz);
  TC_setup();        
  setup_pio_TIOA0(); 
  Serial.begin(9600);
  pinMode(input,INPUT);
  pinMode(led,OUTPUT);
  // Setup all registers
  pmc_enable_periph_clk(ID_ADC); // To use peripheral, we must enable clock distributon to it
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST); // initialize, set maximum posibble speed
  adc_disable_interrupt(ADC, 0xFFFFFFFF);
  adc_set_resolution(ADC, ADC_12_BITS);
  adc_configure_power_save(ADC, 0, 0); // Disable sleep
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1); // Set timings - standard values
  adc_set_bias_current(ADC, 1); // Bias current - maximum performance over current consumption
  adc_stop_sequencer(ADC); // not using it
  adc_disable_tag(ADC); // it has to do with sequencer, not using it
  adc_disable_ts(ADC); // deisable temperature sensor
  adc_disable_channel_differential_input(ADC, ADC_CHANNEL_7);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 1); // triggering from software, freerunning mode
  adc_disable_all_channel(ADC);
  adc_enable_channel(ADC, ADC_CHANNEL_7); // just one channel enabled
  adc_start(ADC);
}

int tcToFreq( int tc_cntr)
{
  int freq_hz;     

  if( tc_cntr == 0 ) return 1000;
  if( fast_mode ) freq_hz = (420000000UL / tc_cntr) / (2 * NWAVE); 
  else            freq_hz = ( 42000000UL / tc_cntr) / (2 * NWAVE);
  return freq_hz;   
}

int freqToTc( int freq_hz)
{
  int tc_cntr = 0;

  if( freq_hz == 0 ) return 25;
  if( fast_mode ) tc_cntr = (420000000UL / freq_hz) / (2 * NWAVE); 
  else            tc_cntr = ( 42000000UL / freq_hz) / (2 * NWAVE);
  return tc_cntr;
}

void switch_mode( int mode)
{
  if( mode == 0 ) 
  {
    DACC->DACC_TPR  =  (uint32_t)  Sinewave[0];      // DMA buffer
    DACC->DACC_TCR  =  NWAVE;
    DACC->DACC_TNPR =  (uint32_t)  Sinewave[1];      // next DMA buffer
    DACC->DACC_TNCR =  NWAVE;
  }
  else
  {
    DACC->DACC_TPR  =  (uint32_t)  SineFast[0];      // DMA buffer
    DACC->DACC_TCR  =  NWAVE;
    DACC->DACC_TNPR =  (uint32_t)  SineFast[1];      // next DMA buffer
    DACC->DACC_TNCR =  NWAVE;
  }
}



void loop() 
{

}

void DACC_Handler(void)
{
  if((dacc_get_interrupt_status(DACC) & DACC_ISR_ENDTX) == DACC_ISR_ENDTX) {
    ++sptr; 
    sptr &=  0x01;
    if(fast_mode == 0) 
    {
      DACC->DACC_TNPR =  (uint32_t)  Sinewave[sptr];      // next DMA buffer
      DACC->DACC_TNCR =  NWAVE;
    }
  else
    {
      DACC->DACC_TNPR =  (uint32_t)  SineFast[sptr];      // next DMA buffer
      DACC->DACC_TNCR =  NWAVE;
    }
  }
}

void setup_pio_TIOA0()  
{
  PIOB->PIO_PDR = PIO_PB25B_TIOA0;  
  PIOB->PIO_IDR = PIO_PB25B_TIOA0;  
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;
}


void TC_setup ()
{
  pmc_enable_periph_clk(TC_INTERFACE_ID + 0 *3 + 0); 

  TcChannel * t = &(TC0->TC_CHANNEL)[0];            
  t->TC_CCR = TC_CCR_CLKDIS;                        
  t->TC_IDR = 0xFFFFFFFF;                           
  t->TC_SR;                                         
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |          
              TC_CMR_WAVE |                         
              TC_CMR_WAVSEL_UP_RC |                 
              TC_CMR_EEVT_XC0 |     
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR;
  
  t->TC_RC = freq_intc;
  t->TC_RA = freq_intc /2;       
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET; 
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;    
}

void dac_setup ()
{
  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  dacc_reset(DACC);
  dacc_set_transfer_mode(DACC, 0);
  dacc_set_power_save(DACC, 0, 1);            // sleep = 0, fastwkup = 1
  dacc_set_analog_control(DACC, DACC_ACR_IBCTLCH0(0x02) | DACC_ACR_IBCTLCH1(0x02) | DACC_ACR_IBCTLDACCORE(0x01));
  dacc_set_trigger(DACC, 1);
  
  //dacc_set_channel_selection(DACC, 1);
  //dacc_enable_channel(DACC, 1);
  dacc_set_channel_selection(DACC, 0);
  dacc_enable_channel(DACC, 0);

  NVIC_DisableIRQ(DACC_IRQn);
  NVIC_ClearPendingIRQ(DACC_IRQn);
  NVIC_EnableIRQ(DACC_IRQn);
  dacc_enable_interrupt(DACC, DACC_IER_ENDTX);

  DACC->DACC_TPR  =  (uint32_t)  Sinewave[0];      // DMA buffer
  DACC->DACC_TCR  =  NWAVE;
  DACC->DACC_TNPR =  (uint32_t)  Sinewave[1];      // next DMA buffer
  DACC->DACC_TNCR =  NWAVE;
  DACC->DACC_PTCR =  0x00000100;  //TXTEN - 8, RXTEN - 1.
}
