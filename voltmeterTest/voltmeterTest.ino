#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)

#define zoomOut 12
#define zoomIn 7
#define squareWaveTest 3

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

int zoomScale = 4;

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
}

int voltReads[160]; // array of voltage readings to populate screen with
// maximum size of array is width of screen.

bool outbuttonPress = false;
bool inbuttonPress = false;

void loop() {
  int i;

  // Half duty square wave
  analogWrite(3, 122);

  // Take as many measurements as determined by the zoomscale - only so many points can fit on the screen!
  for(i = 0; i < 160/zoomScale; i++) {
    // populate array with mappings from ADC scale to width of LCD panel, with 0V in the middle.
    voltReads[i] = map(analogRead(A0)  - analogRead(A5), -1023, 1023, 0, 127);
  }

  // Mark the pixels for each data point and connect them with lines
  for(i = 0; i < 160/zoomScale; i++) {
    tft.fillCircle(voltReads[i], i*zoomScale, 0, ST7735_WHITE);
    if (i * zoomScale < 159) {
      tft.drawLine(voltReads[i], i * zoomScale, voltReads[i+1], (i+1) * zoomScale, ST7735_WHITE);
    }
  }

  delay(100);
  // 100 Millis after finished populating, clear screen.
  tft.fillScreen(0);

  // Zooming functionality with debouncing.
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