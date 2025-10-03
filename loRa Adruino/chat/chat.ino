#include <SPI.h>
#include <LoRa.h>

#define LORA_RST   25
#define LORA_NSS   4
#define LORA_DIO0  32
#define LORA_SCK   26
#define LORA_MISO  33
#define LORA_MOSI  27
#define LORA_BAND  443E6

void onTxDone() {
  Serial.println("DIO0 interrupt: TX Done!");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("LoRa SX1279 DIO0 Test...");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Attach the DIO0 interrupt handler
  LoRa.onTxDone(onTxDone);

  Serial.println("LoRa init succeeded.");
}

void loop() {
  Serial.println("Sending packet...");
  LoRa.beginPacket();
  LoRa.print("test");
  LoRa.endPacket(true); // true = async (DIO0 will fire on completion)

  delay(3000);
}