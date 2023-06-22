#include <Arduino.h>
#include "globals.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Timer Counter configuration (TC0)

void TC0configure(uint16_t rcValue)
{ // turn on the timer clock in the power management controller

  pmc_set_writeprotect(false);
  pmc_enable_periph_clk(ID_TC1);

  /* we want wavesel 01 with RC */
  TC_Configure(/* clock */ TC0, /* channel */ 1, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_SetRC(TC0, 1, rcValue); //
  TC_Start(TC0, 1);

  // enable timer interrupts on the timer
  TC0->TC_CHANNEL[1].TC_IER = TC_IER_CPCS;
  TC0->TC_CHANNEL[1].TC_IDR = ~TC_IER_CPCS;

  //NVIC_EnableIRQ(TC1_IRQn); ///* Enable the interrupt in the nested vector interrupt controller */
  //analogWrite(DAC0, 0);     // this is a cheat - enable the DAC
}
// void setup_pio_TIOA0() //Enables pin 2 pwm output based on Interrupt
// {
//   PIOB->PIO_PDR = PIO_PB25B_TIOA0;
//   PIOB->PIO_IDR = PIO_PB25B_TIOA0;
//   PIOB->PIO_ABSR |= PIO_PB25B_TIOA0;
// }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Create Sine with DAC
void DAC_outputSine(uint16_t frequ, float amplitude, float offset) // [Hz, A, A] create the individual samples for our sinewave table 
{
  Serial.println("Calculating Sine Wave...");
  for (uint32_t nIndex = 0; nIndex < WAVE_SAMPLES; nIndex++)  {
    // normalised to 12 bit range 0-4095
    nSineTable[nIndex] = (uint16_t)((( offset + amplitude * sin(((2.0 * PI) / WAVE_SAMPLES) * nIndex)) * 4095.0) / 2);
  }
  
  if (frequ < f_dac / (min_rc_value * WAVE_SAMPLES / 1))
  { //alle erzeugten Datenpunkte können verwendet werden, die RC Value wird angepasst
    Serial.print("Divider "); Serial.println("1");
    skipEveryNumber = 1;
    rcValue = round(f_dac / (frequ * WAVE_SAMPLES / 1)); // rc = f_dac /(f_out * WAVE_SAMPLES) -> f_out = f_dac/(rc * WAVE_SAMPLES)
    Serial.print("Calculated Frequency from rc_Value: ");Serial.println(1.0* f_dac/(1.0*rcValue * WAVE_SAMPLES));
    TC0configure(rcValue);                                    //starts TC for DAC
    Serial.print("Wave Samples:");
    Serial.println(WAVE_SAMPLES/skipEveryNumber);
    Serial.print("RC Value set to: ");
    Serial.println(rcValue);
    return;
  }
  else //für divier 1 müsste i-1 durch 0 geteilt werden, deshalb muss dieser fall oben abgedeckt werden
  {
    for (size_t i = 2; i < WAVE_SAMPLES / 20; i++) //20 because 20 ist the minimal ammount of points per cycle that make sense
    {
      if ((frequ >= f_dac / (min_rc_value * WAVE_SAMPLES / (i-1))) && (frequ < f_dac / (min_rc_value * WAVE_SAMPLES / i)))
      {
        Serial.print("Divider "); Serial.println(i);
        skipEveryNumber = i;
        rcValue = round(f_dac / (frequ * WAVE_SAMPLES / i)); // rc = f_dac /(f_out * WAVE_SAMPLES) -> f_out = f_dac/(rc * WAVE_SAMPLES)
        Serial.print("Calculated Frequency from rc_Value: ");Serial.println(i* f_dac/(1.0*rcValue * WAVE_SAMPLES));
        TC0configure(rcValue);                                    //starts TC for DAC
        Serial.print("Wave Samples:");
        Serial.println(WAVE_SAMPLES/skipEveryNumber);
        Serial.print("RC Value set to: ");
        Serial.println(rcValue);
        return;
      }
    }
  }
}
