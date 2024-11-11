//| 0000 0000 | | 0000 0000 | | 0000 0000 |
//|start byte | |  1st byte | | 2nd byte  |

//1st 0000  -> gives the start bit 0200
//2nd 0000 -> gives the number of bytes to be received typically 2.

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:

uint16_t received = 320;
uint16_t hex;
uint8_t received_upper = 0;
uint8_t received_lower = 0;

if(Serial.read() == ',')
{
Serial.print(',');
// Copy the bits of the double into a uint64_t for hex representation

  while (Serial.available() == 0) {
  }

received_upper = Serial.read();

Serial.print(',');

while (Serial.available() == 0) {
}

received_lower = Serial.read();


hex = received_upper;
hex = hex << 8;
hex |= received_lower;
Serial.println(hex);
//Serial.println("Done");
Serial.flush();


}


}
