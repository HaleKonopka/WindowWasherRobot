#include "YACL.h"

int sensorData = 0;

boolean sendLog(CBORPair data)
{
  // Convert from CBOR Pair Ojeect to byte array to be sent over serial
  const uint8_t *cbor_encoded = data.to_CBOR();
  
  // Send payload length (2 bites for a size_t on Arduino) followed by payload
  // Format: [ payload len LSB | payload len MSB | payload ]
  size_t payload_len = data.length();  // sizeof(size_t) = 2 bytes
  Serial.write(payload_len % 256);     // Send LSB
  Serial.write(payload_len >> 8);      // Send MSB
  Serial.write(cbor_encoded, payload_len);  // Send Payload
}

void setup() 
{
  Serial.begin(115200);         //Starting serial communication
}
  
void loop() {
  sensorData++;                 // a value increase every loop
  
  // Create CBOR Pair Object. Add items by appending dictionary pairs
  CBORPair data = CBORPair(500);

  // Example data entries
  data.append("time (ms)", millis());
  data.append("x pos (cm)", sensorData * 10);
  data.append("y pos (cm)", sensorData * 6);
  data.append("heading (deg)", sensorData * 5 % 360);

  // Send Log over Serial
  sendLog(data);
  
  delay(1000);                  // give the loop some break
}
