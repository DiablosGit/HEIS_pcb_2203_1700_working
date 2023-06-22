#include <Arduino.h>
#include "SD.h"
#include "Adafruit_I2CDevice.h"
#include "Adafruit_GFX.h"       // Core graphics library
#include "Adafruit_ST7735.h"    // Hardware-specific library for ST7735
#include "Adafruit_ST7789.h"    // Hardware-specific library for ST7789
#include "BasicLinearAlgebra.h"
#include <mcp_can.h>

// Instanzen für SD Karte
extern Sd2Card card;
extern SdVolume volume;
extern SdFile rootSD;

extern File root;

// Instanz für CAN
extern MCP_CAN CAN;

// DISPLAY
extern Adafruit_ST7735 tft; 
#define TFT_CS 10       // Hallowing display control pins: chip select
#define TFT_RST 8       // Display reset
#define TFT_DC 5        // Display data/command select
#define TFT_BACKLIGHT 7 // Display backlight pin

// SD_CAD
#define SD_CARD_CS 24
#define SD_DETECT 25
extern bool sd_detected;
//CAN
#define CAN_CS 25


// some defines for the running program
#define WAVE_SAMPLES 400 // Create a table to hold pre computed sinewave, the table has a resolution of 1000 samples
#define arraySizeMeasurements 400

//Setup parameters
extern double frequency;
extern uint16_t amplitude;
extern uint16_t offset;
extern uint16_t nSineTable[WAVE_SAMPLES]; //defines the number of samples
extern uint16_t rcValue;
extern uint16_t min_rc_value; //dac needs some time to calc the next dac value (around 50), but we need extra time to perform the measurements
extern uint32_t f_dac;   //base clock of dac runs with 42Mhz wich is a prescaler of 2 to base clk
extern uint8_t skipEveryNumber;
extern uint32_t ulInput;    // to set freq, 0-1023 value to be sent to dac, wich is store in the sine Table and piced by the interrupt TC1 handler
extern uint16_t tableIndex; //counter Value to count up the inex in the sineTable

extern uint16_t addVoltage;

extern float f_A_off[6] ;

extern int refresh; //flag set to refresh the Frequency and other parameters if nessecary

//for Impedanz calculation
extern BLA::Matrix<1, arraySizeMeasurements> timeVoltageMatrix, adcVoltageMatrix;
extern BLA::Matrix<1, arraySizeMeasurements> timeCurrentMatrix, adcCurrentMatrix;

extern float phase; //phase angle == phase?
extern float i_amplitude; //current amplitude
extern float u_amplitude; //voltage amplitude
extern float i_phase;
extern float u_phase;
extern float z; //impedance
extern float delta_phase; //Phasenversatz