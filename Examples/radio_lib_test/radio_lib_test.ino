#include <ax25.h>

#define KICKSAT_RADIO_SS        10
#define KICKSAT_RADIO_INTERUPT  2
#define KICKSAT_RADIO_SDN       9
#define TEENSY_LED              13
#define RH_RF22_HAVE_SERIAL

AX25 radio = AX25(KICKSAT_RADIO_SS, KICKSAT_RADIO_INTERUPT);

void setup() {
  Serial.begin(9600);
  pinMode(TEENSY_LED, OUTPUT);
  pinMode(KICKSAT_RADIO_SDN, OUTPUT);
  Serial.write("Testing 123");
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(TEENSY_LED, HIGH);
  Serial.println("sending");
  radio.powerAndInit(KICKSAT_RADIO_SDN);
  radio.transmit("FFF");
  digitalWrite(TEENSY_LED, LOW);

  delay(1000);

}
