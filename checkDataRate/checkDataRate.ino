#include <Adafruit_ADS1X15.h>

 Adafruit_ADS1115 ads;  /* Use this for the 16-bit version */

constexpr int READY_PIN = 3;

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif
long timeer = 0;
volatile int new_data = 0;
int peat =0;

void setup() {
  timeer = millis();
  Serial.begin(9600);
  Serial.println("Hello!");

  Serial.println("Getting single ended reading from AIN0 ");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV/ADSfalse 1015, 0.1875mV/ADS1115)");

  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }

  // set data rate to 8 samples per second
  ads.setDataRate(RATE_ADS1115_16SPS);

  pinMode(READY_PIN, INPUT);
  // We get a falling edge every time a new sample is ready.
  attachInterrupt(digitalPinToInterrupt(READY_PIN), NewDataReadyISR, FALLING);

  // Start continuous conversions.
  ads.startADCReading(ADS1X15_REG_CONFIG_MUX_SINGLE_0, /*continuous=*/true);
}

void loop(){
 if(timeer + 999 <= millis()) {
  Serial.println(peat);
  timeer = millis();
  peat = 0;
  }
}

void IRAM_ATTR NewDataReadyISR() {
ads.getLastConversionResults();
peat++;
}
