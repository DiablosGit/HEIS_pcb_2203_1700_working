#include <Arduino.h>

void initCAN();
void CAN_RX();
void CAN_TX(unsigned char can_byte[8]);
