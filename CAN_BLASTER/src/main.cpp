#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>


#define CAN0_CS 10                          
#define CAN0_INT 9                          


#define BLUE_LED 3
#define RED_LED 2
#define BUTTON 4

#define GNSSUTCtimeID 0xC9
#define orientationID 0xCA
#define GNSS_PositionID 0xCC


int buttonState;               // current stable button state
int lastButtonState = LOW;     // previous reading from the button
unsigned long lastDebounceTime = 0;  // last time the button input changed
const unsigned long debounceDelay = 100; // debounce time in milliseconds


unsigned long looptime, acumaltor;
long SekTimer, SekTimer2;
int iterations;

MCP_CAN CAN0(CAN0_CS);     // Set CS to pin 10
byte data[8];
byte dataMock[8] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07};
byte gnssData[8];
byte UTCData[8];

enum State {
  TXSINGLE,
  TXMANY
};

State currentState = TXSINGLE;


static void generateMockUTC(byte data[8]){
  unsigned long m = millis();
  unsigned long totalSeconds = m / 1000;
  int hours = (totalSeconds / 3600) % 24;         // HH (0-23)
  int minutes = (totalSeconds / 60) % 60;           // MM (0-59)
  int seconds = totalSeconds % 60;                  // SS (0-59)
  int hundredths = (m % 1000) / 10;                 // hundredths (0-99)  
  data[0] = hours / 10;
  data[1] = hours % 10;
  data[2] = minutes / 10;
  data[3] = minutes % 10;
  data[4] = seconds / 10;
  data[5] = seconds % 10;
  data[6] = hundredths / 10;
  data[7] = hundredths % 10;
}

static void generateMockGNSS(byte data[8]) {
  // Base coordinates: for example, Google HQ roughly: 37.4220° N, -122.0841° W
  // Multiply by 1e7 to convert to integer representation.
  int32_t baseLat = 374220000;    // 37.4220° * 1e7
  int32_t baseLon = -1220841000;   // -122.0841° * 1e7

  // Create a small variation based on millis() for dynamic simulation.
  // This introduces a subtle change over time.
  long variation = millis() % 1000;  // Variation between 0 and 999

  // Apply the variation (example: increase latitude and decrease longitude slightly)
  int32_t lat = baseLat + variation;
  int32_t lon = baseLon - variation;

  // Pack the 32-bit latitude into data[0..3] in little-endian order.
  data[0] = lat & 0xFF;
  data[1] = (lat >> 8) & 0xFF;
  data[2] = (lat >> 16) & 0xFF;
  data[3] = (lat >> 24) & 0xFF;

  // Pack the 32-bit longitude into data[4..7] in little-endian order.
  data[4] = lon & 0xFF;
  data[5] = (lon >> 8) & 0xFF;
  data[6] = (lon >> 16) & 0xFF;
  data[7] = (lon >> 24) & 0xFF;
}

void sendCANMessage(unsigned long id, byte ext, byte len, byte* data) {
  byte sndStat = CAN0.sendMsgBuf(id, ext, len, data);
  if (sndStat == CAN_OK) {
    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  } 
}
void printGNSS(){
  Serial.print("GNSS Data: ");
  for (int i = 0; i < 8; i++) {
    if (gnssData[i] < 16) Serial.print("0"); // Format single digit with a leading zero.
    Serial.print(gnssData[i], HEX);
    Serial.print(" ");
  }
  int32_t lat = ((int32_t)gnssData[0]) 
              | (((int32_t)gnssData[1]) << 8)
              | (((int32_t)gnssData[2]) << 16)
              | (((int32_t)gnssData[3]) << 24);
  int32_t lon = ((int32_t)gnssData[4])
              | (((int32_t)gnssData[5]) << 8)
              | (((int32_t)gnssData[6]) << 16)
              | (((int32_t)gnssData[7]) << 24);

  // For tabulated display, we'll extract the last 6 digits of the absolute values.
  // This is just one example of formatting. Adjust as needed.
  long tabLon = abs(lon) % 1000000;  // 6-digit field for longitude
  long tabLat = abs(lat) % 1000000;  // 6-digit field for latitude

  // Create a string in the format "xxxxxxExxxxxxxN".
  // Here we assume 'E' follows the longitude and 'N' follows the latitude.
  char tabulated[20];
  sprintf(tabulated, "%06ldE%06ldN", tabLon, tabLat);

  Serial.println(tabulated);
}

void printUTC(){
  Serial.print("Simulated UTC Time: ");
  Serial.print(UTCData[0]); Serial.print(UTCData[1]); Serial.print(":");
  Serial.print(UTCData[2]); Serial.print(UTCData[3]); Serial.print(":");
  Serial.print(UTCData[4]); Serial.print(UTCData[5]); Serial.print(".");
  Serial.print(UTCData[6]); Serial.println(UTCData[7]);
}

bool miliSEKtimer(long x){  
  SekTimer = millis();  
  if (SekTimer-SekTimer2 > x){
      SekTimer2 = SekTimer; 
      return true;
  }
  else{
      return false;
  }
}


void setup()
{

  Serial.begin(115200);

  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUTTON, INPUT);


  // Initialize MCP2515 running at 16MHz with a baudrate of 1000kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  delay(500);
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}

void loop()
{
  looptime = micros();
  int reading = digitalRead(BUTTON);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      
      // On a rising edge (button press), toggle the state.
      if (buttonState == HIGH) {
        if (currentState == TXSINGLE) {
          currentState = TXMANY;
          Serial.println("State changed to: TXMANY");
        } else {
          currentState = TXSINGLE;
          Serial.println("State changed to: TXSINGLE");
        }
      }
    }
  }

  if (miliSEKtimer(100)){
    switch (currentState) {
      case TXSINGLE:
        digitalWrite(BLUE_LED, LOW);
        digitalWrite(RED_LED, HIGH);
    
        // send data:  ID = 666, Standard CAN Frame, Data length = 8 bytes, 'data' = 0 1 2 3 4 5 6 7

        sendCANMessage(0xC8, 0, 8, dataMock);
        break;
    
      case TXMANY:
        digitalWrite(BLUE_LED, HIGH);
        digitalWrite(RED_LED, LOW);
        generateMockGNSS(gnssData);
        generateMockUTC(UTCData);

        // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
        sendCANMessage(GNSS_PositionID, 0, 8, gnssData);
        sendCANMessage(GNSSUTCtimeID, 0, 8, UTCData);

        printUTC();
        printGNSS();
        break;
    }
  }


  //timer function
  lastButtonState = reading;

}