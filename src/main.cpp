// Projektbeschreibung


// Bsp Über Serial Monitor: 100.0,0.4,0.2 (frequenz, amplituden Faktor, offset faktor), maximal möglicher amplitudenfaktor 0.5, da 
// Literatur /Links
// Sinusgenerator mit ad9850: https://elektronikbasteln.pl7.de/ad9850-sinus-dds-atmega328-arduino
// Display: https://www.jvzdigitalsourcing.com/blogs/projects/arduino-due-with-1-8-tft-display
// Fast ADC (56kHZ) https://forum.arduino.cc/index.php?topic=6549.15
// Digital Potentiometer https://www.arduino.cc/en/Tutorial/LibraryExamples/DigitalPotentiometer
// DAC 0-3.3V https://create.arduino.cc/projecthub/ArduPic/how-to-modify-analog-output-range-of-arduino-due-6edfb5

#include <Arduino.h>
#include <Wire.h>            //TFT TFTscreen = TFT(TFT_CS, TFT_A0, TFT_Reset);
#include <SPI.h>             // Bibliothek, um mit SPI-Geräten kommunizieren zu können (Arduino als Master)
#include <BasicLinearAlgebra.h>
using namespace BLA;

#include <SD.h>
#include <can.h>
#include <leastSquaresCalculation.h>
#include <globals.h>
#include <init.h>
#include <sine.h>

#define numOfPeriods 40 //<<-------------- defines length of sine periods per measurement, whereas ammount of measurements is defined in arraySizeMeasurements and 
//n_samples_per_cycle defines the measurements per sine cycle

// some defines for PIN Numbers
#define relay 26
#define analogVoltage 3

// variables for measurements
float voltage; //measured batt voltage
float current;  //measured current via shunt direct to adc
float current_OP; //measured currend via ad623
int adc_gnd;  //adc value of gnd voltage 
int adc_batt_minus; //ad value of batt minus voltage should be the same as adc_gnd
float v_ref; //v_ref not unusable due to wrong connection of lt1009..

#define dacSine A6
#define adcVoltage A0
#define adcCurrent A2
#define opCurrent A4
#define ADC_GND A3
#define ADC_BATT_MINUS A1
#define VREF A5


// vairables for sine calculation
float meas_Rg = 1.998; //factor for voltage divider 2x 100k Ohm : (100.1+99.9)/100.1 R21 = 100.1 R10 = 99.9
float v_off; //offset voltage from gnd
int measurementDelay; //stores time for delay between measurements so that n samples per cycle are beeing taken
float freq; //temporary variable for frequency
volatile uint32_t ulOutput; //holds the sine output voltage value
bool measurementDone = false; //holds information if frequency calulation can be started

//values for period duration measurement, currently not used/active because of instability
volatile float T; 
volatile float tic;
volatile float toc;
volatile int32_t ti = 0;
volatile float fs[numOfPeriods]; //holds the time delays. Later the average of the times is beeing calculated
volatile float periodTime = 0;

//System parameters
uint8_t operationMode = 1; //holds the different modes to sperate the steps in loop funciton
volatile int periodCounter = 0; //hols the periods beeing outputted so it can be stopped after a given number (numOfPeriods)
bool firstTimeInLoop = true; //checks in operation Mode 1 if the mode is beeing called the first time
bool updateFrequency = false;  //flag set to refresh the Frequency and other parameters if nessecary
File dataFile;
char sdFileName[64];

//measurment results beeing stored in these matrix objects
BLA::Matrix<1, arraySizeMeasurements> timeVoltageMatrix, adcVoltageMatrix;
BLA::Matrix<1, arraySizeMeasurements> timeCurrentMatrix, adcCurrentMatrix;
int result_storage_counter = 0;

// Variables for Sine generation
// Standardwerte für Sinuseinstellungen, können über Serial geändert werden
double frequency;
uint16_t amplitude = 1;
uint16_t offset = 1;

uint16_t nSineTable[WAVE_SAMPLES]; //defines the number of samples per sine cycle-> more results in a higher resolution but lower max output freq
uint16_t rcValue;
uint16_t min_rc_value = 150; //dac needs some time to calc the next dac value (around 50), but we need extra time to perform the measurements
uint32_t f_dac = 42000000;   //base clock of dac runs with 42Mhz wich is a prescaler of 2 to base clk
uint8_t skipEveryNumber; //holds the value of how many numbers beeing skipped in table at higher frequencies
uint32_t ulInput = 0;    // to set freq, 0-1023 value to be sent to dac, wich is store in the sine Table and piced by the interrupt TC1 handler
uint16_t tableIndex = 1; //counter Value to count up the inex in the sineTable

int n_samples_per_cycle = 25; //measurement points per sin cycle
volatile int measurementCounter = 0; //volatile to make sure the variable is checked every time the TC handler is executed

float f_A_off[6]; //array that holds the input value via serial

void refreshFrequency(float arr[6])
{
  DAC_outputSine(arr[0], arr[1], arr[2]); //f , A , off
  Serial.print("Frequenz:");Serial.print(arr[0]);
  Serial.print("Hz  Amplituden-Faktor: ");Serial.print(arr[1]);
  Serial.print(" Offset Faktor: ");Serial.println(arr[2]);
  frequency = arr[0];
  updateFrequency = false;
}

//unused function to set up interrups at due
// void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
//   pmc_set_writeprotect(false);
//   pmc_enable_periph_clk((uint32_t)irq);
//   TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
//   uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
//   TC_SetRA(tc, channel, rc/2); //50% high, 50% low
//   TC_SetRC(tc, channel, rc);
//   TC_Start(tc, channel);
//   tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
//   tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
//   NVIC_EnableIRQ(irq);
// }


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
void setup() {

  analogReadResolution(12);

  Serial.begin(9600);Serial.println("\n");
  pinMode(relay, OUTPUT);digitalWrite(relay, LOW);
  digitalWrite(relay, HIGH);delay(500);digitalWrite(relay, LOW); //TEST RELAIS

  //init
  digitalWrite(CAN_CS, LOW); initCAN();digitalWrite(CAN_CS, HIGH);
  digitalWrite(TFT_CS, LOW); initDisplay(); digitalWrite(TFT_CS, HIGH);
  digitalWrite(SD_CARD_CS, LOW); delay(100);initSDCard(); digitalWrite(SD_CARD_CS, HIGH);
  
  //display
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_WHITE, ST7735_BLACK);
  tft.fillScreen(ST7735_BLACK);

  //sets a low value on dac and actives it (0 is somehow not possible)
  analogWrite(DAC1, 1); //initializes DAC 1 to be used
  operationMode = 1;

  //Hier die Startwerte: Frequ, Amplitude, Offset
  float startFreq[6] =  {500,0.5,0.5,1,1,1};
  refreshFrequency(startFreq);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LOOP

// Operation modes:
// case1 = Versuchsvorbereitung Parametrisierung
// case2 = Durchführung Sinusgenerator + Messwerterfassung
// case3 = Messwertaufbereitung Export Messwerte/Darstellung

void loop() {
  //TODO: Speicherung Messwertes
  switch (operationMode) {
    case 1: {
      if (firstTimeInLoop)
      {
        firstTimeInLoop = false;
        Serial.println("\nSTEP 1: Sine Setup");
      }
      digitalWrite(DAC1, 0);
      
      v_ref = analogRead(VREF)/4095.0 * 3.3;
      voltage = (analogRead(adcVoltage) / 4095.0 * 3.3 * meas_Rg) + v_off;
      adc_batt_minus = analogRead(ADC_BATT_MINUS);
      adc_gnd = analogRead(ADC_GND);
      v_off = ((adc_batt_minus+adc_gnd)/2)/4095 * 3.3;
      //current = analogRead(adcCurrent) / 4095.0 * 3.3 / 0.02; //direct current via shunt
      current = 40.943/4095 * analogRead(opCurrent); //40A = 3,224V -> 40,943V = 3.3V mit R = 4 Rg = 33k current measurement via ad623

      //Display important variables
      tft.setCursor(0, 20);
      tft.print("Freq:   ");tft.print(frequency);tft.println("Hz");
      tft.print("Curr:   ");tft.print(current);tft.println("A"); //TFT Strom
      tft.print("Volt:   ");tft.print(voltage);tft.println("V"); //TFT Spanung
      tft.print("B-:     ");tft.print(adc_batt_minus);tft.println("");
      tft.print("GND:    ");tft.print(adc_gnd);tft.println("");
      tft.print("VREF:   ");tft.print(v_ref,4);tft.println("V");
      tft.print("vOff:   ");tft.print(v_off,4);tft.println("V");


      ////CAN TEST:
      //digitalWrite(SD_CARD_CS, HIGH);digitalWrite(CAN_CS, LOW);
      //CAN_RX(); //CAN Nachrichten empfangen;
      //unsigned char can_byte[8] = { 10,  10,1, 10,2, 3, 4, 2};
      //CAN_TX(can_byte);
      //digitalWrite(SD_CARD_CS, HIGH);digitalWrite(CAN_CS, HIGH);

      //EInlesen von Frequenz, Offset und Amplitude
      int i=0;
      while (Serial.available() > 0) { //serial msg bsp: 1.0,3.3,1.2
        f_A_off[i] = Serial.parseFloat();
        Serial.println("Input recognized");
        updateFrequency = true;
        i++;
      }
      
      // Update sine table with new parameters from serial input
      if (updateFrequency) {
        refreshFrequency(f_A_off);
        updateFrequency = false;

        delay(100);
        
        //reset some values for new measurement
        periodCounter = 0; periodTime = 0;
        measurementCounter = 0;
        ulOutput = 0; tableIndex = 0;
        tic=0; toc = 0; T = 0;
        for (size_t i = 0; i < numOfPeriods; i++) { fs[i] = 0; }
        operationMode = 2;
        break;
      }

      break;
    }

    case 2: {
      Serial.println("\nSTEP 2: Performing Measurement");
      digitalWrite(relay, HIGH);delay(50);
      digitalWrite(TFT_CS, HIGH); //turn of tft com
      digitalWrite(SD_CARD_CS, HIGH); //turn off com

      measurementDelay = 1/frequency/n_samples_per_cycle * 1000; //f=1/T -> T = 1/f-> measurement Delay = T/measurementsperCycle


      //TODO: Bei der Zweiten Messung spinnt der DAC Output, ich vermute es liegt am Interrupt Handler aber konnte das Problem noch nicht lösen. Der Arduino muss also nach jeder Messung zurückgesetzt werden.
      NVIC_ClearPendingIRQ(TC1_IRQn);
      NVIC_EnableIRQ(TC1_IRQn); // Re enable the interrupt in the nested vector interrupt controller
      TC_Start(TC0, 1);

      //Here the measurement is beeing performed
      while (measurementCounter <= arraySizeMeasurements)
      {
        timeVoltageMatrix(0, measurementCounter) = double(micros()); // Zeitbasis s
        adcVoltageMatrix(0, measurementCounter) = double(analogRead(adcVoltage));//4096.0*3.3);
        timeCurrentMatrix(0, measurementCounter) = double(micros()); // Zeitbasis s
        adcCurrentMatrix(0, measurementCounter) = double(analogRead(adcCurrent));// 4095.0 * 3.3 / 0.02); //Berechnung Strom mit R=0.02
        measurementCounter++;
        delay(measurementDelay);
      }
      while (measurementDone != true) //Program stays in here until measurement is done
      {
        delay(1);
      }

      measurementDone = false; //reset
      digitalWrite(relay, LOW); //turn of relais
      Serial.println("Measurement taken");
      /* This part is meant to calc the freq of the measured period durations but is somehow unstable-> for leastsqare the input freq. is beeing used
      delay(300);
      //berechne frequenz
      for (size_t i = 1; i < numOfPeriods; i++) //erster messwert ist schrott, da viel zu lange //100 zum einschwingen
      {
        //Serial.println(fs[i]);
        delay(2); //muss drinnen bleiben? -> ja Wieso? keine Ahnung!
        freq = freq + 1.0*(fs[i]/1E6);
      }
      frequency = 1.0/(freq/(numOfPeriods)); //Durchschnitt berechnen 1/ da eigentlich die periodendauer gespeichert wird //101 dalmatiner
      Serial.print("Mittlere Frequenz: ");Serial.println(frequency);
      */

      operationMode = 3;
      break; 
    }

    case 3: {
      Serial.println("\nSTEP 3: Save Results");
      operationMode = 1; firstTimeInLoop = true; 
      leastSquares(); // Here the amplitude and phase of voltage and current sine are beeing calculated
      
      Serial.print("Writing to sd...");
      delay(200);

      //creating file and dumping values to file
      digitalWrite(TFT_CS, HIGH); //turn of tft comm
      digitalWrite(CAN_CS, HIGH); //turn off can com
      digitalWrite(SD_CARD_CS, LOW); //turn on sd card communcation
      int n = 0;
      snprintf(sdFileName, sizeof(sdFileName), "%d_%03d.csv",int(frequency),  n); // includes a three-digit sequence number in the file name
      Serial.println(sdFileName);
      
      //////////////////////////////////////////// WRITING MEASUREMENTS TO FILE //////////////////////////////////////////////////////
      while (SD.exists(sdFileName)) {
        n++;
        snprintf(sdFileName, sizeof(sdFileName), "%d_%03d.csv", int(frequency), n);
      }
      File dataFile = SD.open(sdFileName, FILE_WRITE); // create New File with write permission

      dataFile.println("time,voltage, current,  frequencymeasured");
      for (size_t i = 0; i < arraySizeMeasurements; i++) {
        String dataString =String(String(timeCurrentMatrix(0,i),5) + "," + String(adcVoltageMatrix(0,i)) + "," + String(adcCurrentMatrix(0,i)) +"," + String(fs[i]));
        dataFile.println(dataString); //should always be a new line
      }
      dataFile.close();
      Serial.println("Messwerte gespeichert");
      
      //////////////////////////////////////////// WRITING IMPEDANCE RESULTS TO FILE //////////////////////////////////////////////////////
      if (!SD.exists("Impedanz.csv"))
      {
        File impFile = SD.open("Impedanz.csv", FILE_WRITE);  // create New File with write permission

        impFile.println("frequency,impedance, phase_delta, u_amp, i_amp, u_phase, i_phase");
        String dataString =String(String(frequency) + "," + String(z,4) + "," + String(delta_phase,4) +"," + String(u_amplitude,4)+"," + String(i_amplitude,4)+"," + String(u_phase,4)+"," + String(i_phase,4));
        Serial.println(dataString);
        impFile.println(dataString); //should always be a new line
        impFile.close();
      }
      if(SD.exists("Impedanz.csv")) {
        File impFile = SD.open("Impedanz.csv", FILE_WRITE);
        String dataString =String(String(frequency) + "," + String(z,4) + "," + String(delta_phase,4) +"," + String(u_amplitude,4)+"," + String(i_amplitude,4)+"," + String(u_phase,4)+"," + String(i_phase,4));
        impFile.println(dataString); //should always be a new line
        impFile.close();
      }

      Serial.println("Messwerte Impedanzmessung gespeichert");
      Serial.println("Writing to SD done");
      digitalWrite(SD_CARD_CS, HIGH); //turn off sd card communcation
      digitalWrite(TFT_CS, LOW); //turn on TFT 

      delay(200); //just add some more random delays as a bugfix ¯\_(ツ)_/¯   
    }
  }
}



void TC1_Handler() { //Handles the interrupt to output sine 
  // We need to get the status to clear it and allow the interrupt to fire again
  
  TC_GetStatus(TC0, 1);

  if (tableIndex >= WAVE_SAMPLES) { //überlauf nach einer Periode

    toc = micros(); //zweite messung gestartet
    T = toc-tic; //berechne Periodendauer
    fs[ti] = T;//1/(T/1.0E6); //speichere Periodendauer
    ti++;

    tableIndex = 0;
    periodCounter++;
    tic = micros();

    if (periodCounter >= numOfPeriods) //interrupt has to end use looop to start new interrupt?
    { 
      measurementDone = true;
      operationMode = 3;
      analogWrite(DAC1,1);
      NVIC_DisableIRQ(TC1_IRQn);
      TC_Stop(TC0, 1);
    }
  }
  else {
    ulOutput = nSineTable[tableIndex]; // get the current sample
    tableIndex = tableIndex + skipEveryNumber; //increments the table Index
    dacc_write_conversion_data(DACC_INTERFACE, ulOutput); // we cheated and user analogWrite to enable the dac, but here we want to be fast so write directly

  }
}

