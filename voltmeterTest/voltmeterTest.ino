#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)

#define zoomOut 12
#define zoomIn 7

// #define TFT_SCLK 13   // set these to be whatever pins you like!
// #define TFT_MOSI 11   // set these to be whatever pins you like!
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

int zoomScale = 4;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize the TFT screen with 128x160 resolution
  tft.initR(INITR_BLACKTAB);  // Use this initialization method

  Serial.println("Initialized");
  // Fill the screen with black color
  tft.fillScreen(ST7735_BLACK);

  // Optionally, display some text for testing
  tft.setTextColor(ST7735_WHITE); 
  tft.setTextSize(1); 
  tft.setCursor(10, 10); 
  pinMode(3, OUTPUT);
  pinMode(zoomOut, INPUT);
  pinMode(zoomIn, INPUT);
}

int voltReads[160];
int output = 0;
bool high = false;

bool outbuttonPress = false;
bool inbuttonPress = false;

void loop() {
  int i;
  // put your main code here, to run repeatedly:
  // Serial.print("Voltage read: ");
  analogWrite(3, 122);
  for(i = 0; i < 160/zoomScale; i++) {
    // if (high) {
    //   if (output>0) {
    //     analogWrite(3, output);
    //     output--;
    //   }
    //   else {
    //     high = false;
    //   }
    // }
    // else {
    //   if (output < 255) {
    //     analogWrite(3, output);
    //     output++;
    //   }
    //   else {
    //     high = true;
    //   }
    // }
    voltReads[i] = map(analogRead(A0)  - analogRead(A5), -1023, 1023, 0, 127);
  }

  for(i = 0; i < 160/zoomScale; i++) {
    tft.fillCircle(voltReads[i], i*zoomScale, 0, ST77XX_WHITE);
    if (i * zoomScale < 159) {
      tft.drawLine(voltReads[i], i * zoomScale, voltReads[i+1], (i+1) * zoomScale, ST7735_WHITE);
    }
  }
  // digitalWrite(bit0, HIGH);
  delay(100);
  tft.fillScreen(0);
  // for(i = 0; i < 160; i++) {
  //   tft.fillCircle(voltReads[i], i, 0, ST77XX_BLACK);
  //   if (i < 159) {
  //     if (voltReads[i] != voltReads[i+1]) {
  //       tft.drawLine(voltReads[i], i, voltReads[i+1], i+1, ST7735_BLACK);
  //     }
  //   }
  // }

  if (digitalRead(zoomOut) && !outbuttonPress) {
    if (zoomScale>1) {
      zoomScale--;
    }
    outbuttonPress = true;
  } else if (outbuttonPress && !digitalRead(zoomOut)) {
    outbuttonPress = false;
  }

  if (digitalRead(zoomIn) && !inbuttonPress) {
    if (zoomScale<10) {
      zoomScale++;
    }
    inbuttonPress = true;
  } else if (inbuttonPress && !digitalRead(zoomIn)) {
    inbuttonPress = false;
  }
}