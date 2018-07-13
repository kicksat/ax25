#include <ax25.h>

#define KICKSAT_RADIO_SS        6
#define KICKSAT_RADIO_INTERUPT  2
#define KICKSAT_RADIO_SDN       3
#define TEENSY_LED              9
#define RH_RF22_HAVE_SERIAL

AX25 radio = AX25(KICKSAT_RADIO_SS, KICKSAT_RADIO_INTERUPT);

void setup() {
  SerialUSB.begin(9600);
  pinMode(TEENSY_LED, OUTPUT);
  pinMode(KICKSAT_RADIO_SDN, OUTPUT);
  SerialUSB.write("Testing 123");
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(TEENSY_LED, HIGH);
  SerialUSB.println("sending");
  radio.powerAndInit();
  radio.transmit("FFF", 12);
  digitalWrite(TEENSY_LED, LOW);

  delay(100);

}
