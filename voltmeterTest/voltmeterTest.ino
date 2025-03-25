#include <SPI.h>
#include <Adafruit_ST7735.h>
#include <TimerOne.h>

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)

#define interruptTest 4
#define zoomOut 3
#define zoomIn 2
#define squareWaveTest 5

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

int zoomScale = 4;
volatile bool ledState = LOW;
const unsigned long debounceDelay = 500;
volatile unsigned long lastPressIn = 0;
volatile unsigned long lastPressOut = 0;

void toggleLED() {
  ledState = !ledState;
  digitalWrite(interruptTest, ledState);
}

void zoomOutHandler() {
  unsigned long currentTime = millis();
  if (currentTime-lastPressOut>debounceDelay) {
    if (zoomScale>3) {
      zoomScale--;
    }
    lastPressOut = currentTime;
  }
}

void zoomInHandler() {
  unsigned long currentTime = millis();
  if (currentTime-lastPressIn>debounceDelay) {
    if (zoomScale<10) {
      zoomScale++;
    }
    lastPressIn = currentTime;
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  tft.initR(INITR_BLACKTAB);
  Serial.println("Initialized");

  // Fill the screen with black color
  tft.fillScreen(ST7735_BLACK);

  pinMode(squareWaveTest, OUTPUT);
  pinMode(zoomOut, INPUT);
  pinMode(zoomIn, INPUT);
  pinMode(interruptTest, OUTPUT);

  Timer1.initialize(500000);
  Timer1.attachInterrupt(toggleLED);
  attachInterrupt(digitalPinToInterrupt(zoomOut), zoomOutHandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(zoomIn), zoomInHandler, FALLING);
}

// uint16_t buffer[160][128];


// void drawLineToBuff(int x0, int y0, int x1, int y1, uint16_t color) {
//   double dx = ((double)x1-(double)x0) / ((double)y1 - (double)y0);
//   double dy = ((double)y1 - (double)y0) / ((double)x1-(double)x0);
//   double x0New = (double)x0, y0New = (double)y0;
  
//   while (true) {
//     if (x0New < 160&&x0New>=0 && y0New<128&&y0New>=0) {
//       buffer[round(x0New)][round(y0New)] = color;
//     }
//     if (round(x0New)==x1 || round(y0New)==y1) break;
//     x0New += dx;
//     y0New += dy;
//   }
// }

// bool outbuttonPress = false;
// bool inbuttonPress = false;
int voltReads[160]; // array of voltage readings to populate screen with
// maximum size of array is width of screen.

void loop() {
  int i;

  // Half duty square wave
  analogWrite(squareWaveTest, 122);

  // Take as many measurements as determined by the zoomscale - only so many points can fit on the screen!
  for(i = 0; i < 160/zoomScale; i++) {
    // populate array with mappings from ADC scale to width of LCD panel, with 0V in the middle.
    voltReads[i] = map(analogRead(A0)  - analogRead(A5), -1023, 1023, 0, 127);
  }
  
  tft.fillScreen(0);

  // Mark the pixels for each data point and connect them with lines
  /* NOTE: any TFT commands take hella long so keep that in mind - 
    in fact, you need to hold the button long enough for the program to 
    pass these next couple of stages and get to the button detection part as of now.
  */

  // memset(buffer, 0, sizeof(buffer));
  for(i = 0; i < 160/zoomScale; i++) {
    // tft.drawPixel(voltReads[i], i*zoomScale, ST7735_WHITE);
    if (i * zoomScale < 159) {
      tft.drawLine(voltReads[i], i * zoomScale, voltReads[i+1], (i+1) * zoomScale, ST7735_WHITE);
      // drawLineToBuff(i * zoomScale, voltReads[i], (i+1) * zoomScale, voltReads[i+1], ST7735_WHITE);
    }
  }
  // tft.drawRGBBitmap(0, 0, buffer, 160, 128);

  Serial.println(zoomScale);
  // delay(100);
  // 100 Millis after finished populating, clear screen.

  // Zooming functionality with debouncing.
  // if (digitalRead(zoomOut) && !outbuttonPress) {
  //   if (zoomScale>1) {
  //     zoomScale--;
  //   }
  //   outbuttonPress = true;
  // } else if (outbuttonPress && !digitalRead(zoomOut)) {
  //   outbuttonPress = false;
  // }

  // if (digitalRead(zoomIn) && !inbuttonPress) {
  //   if (zoomScale<10) {
  //     zoomScale++;
  //   }
  //   inbuttonPress = true;
  // } else if (inbuttonPress && !digitalRead(zoomIn)) {
  //   inbuttonPress = false;
  // }
}