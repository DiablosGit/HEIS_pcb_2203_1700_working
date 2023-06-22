#include <Arduino.h>
#include <mcp_can.h>
#include <globals.h>

//******************** Empfangen der CAN-Nachrichten *********************

void CAN_RX() {
  int can_id;
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive())                                     // Überprüfen auf neue Nachrichten
  {
    CAN.readMsgBuf(&len, buf);                                                //Buffer auslesen
    can_id = CAN.getCanId();                                                  // ID der Nachricht auslesen 
    unsigned long StartTime = millis();
    Serial.print(can_id); Serial.print(" :"); 
    for (size_t i = 0; i < 8; i++)
    {
        Serial.print(buf[i]);
    }
    Serial.println("");
  }

}

//******************** Senden der CAN-Nachrichten *********************

void CAN_TX(unsigned char can_byte[8]) {

  int can_id = 10;

  //CAN-nachricht senden
  CAN.sendMsgBuf(can_id, 0, 8, can_byte);


  
}