#include <Arduino.h>
#include "globals.h"
#include "Adafruit_I2CDevice.h"
#include "Adafruit_GFX.h"       // Core graphics library
#include <mcp_can.h>

// Instanzen für SD Karte
Sd2Card card;
SdVolume volume;
SdFile rootSD;
File root;
bool sd_detected;

//Instanzen für TFT Display
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

//Instanz für CAN

MCP_CAN CAN(CAN_CS);  
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Display initialisation
void initDisplay()
{ // Tutnr on backlight and display black background
  pinMode(TFT_BACKLIGHT, OUTPUT);

  digitalWrite(TFT_BACKLIGHT, HIGH); // Backlight on
  tft.initR(INITR_BLACKTAB);         // Init ST7735S chip, black tab

  tft.fillScreen(ST77XX_BLACK);

  tft.setRotation(1);
  tft.setTextSize(1);
  Serial.println("Display Initialized");
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SD Card reader initialisation
void initSDCard()
{
  digitalWrite(TFT_CS, LOW);
  // pinMode(SD_DETECT, INPUT_PULLUP);
  // if (SD_DETECT == LOW)
  // {
  //   Serial.println("SD Card inserted");
  //   sd_detected = true;
  // }
  // else if (SD_DETECT == HIGH);
  // {
  //   Serial.println("SD Card missing");
  //   sd_detected = false;
  // }
  
  
  Serial.print("Initializing SD card...");

  if (!SD.begin(SD_CARD_CS))
  {
    //TODO: Add information on Screen if something is Wrong with SD Card / no SD Card inserted
    Serial.println("initialization failed!");
  }
  Serial.println("initialization done.");

  Serial.print("Card type: ");
  switch (card.type())
  {
  case SD_CARD_TYPE_SD1:
    Serial.println("SD1");
    break;
  case SD_CARD_TYPE_SD2:
    Serial.println("SD2");
    break;
  case SD_CARD_TYPE_SDHC:
    Serial.println("SDHC");
    break;
  default:
    Serial.println("Unknown");
  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// CAN initialisation

void initCAN() {
  
   MCP_CAN CAN(CAN_CS);                                                        // Set CS pin

  if (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
  {
    Serial.println("CAN BUS Shield init fail");
  }
  else {Serial.println("CAN BUS Shield init ok!");}
  
} 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


