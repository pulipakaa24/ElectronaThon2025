#include <arduinoFFT.h>
#include <TimerOne.h>
#include <Adafruit_ST7735.h>

#define SAMPLES 128
#define MAXVAL 2047

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)
#define signalPin 5
#define posTerminal A0
#define vertKnob A1
#define horKnob A2
#define toggleText 2
#define selectAxis 3 

float vertScale = 1.0;
bool axisToggle = false; // false for vertical axis labels, true for horizontal

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Array to hold the real and imaginary parts of the FFT
float uReal[SAMPLES];
float uImag[SAMPLES];

const unsigned short debounceDelay = 500;
volatile unsigned long lastPressText = 0;
volatile unsigned long lastPressAxis = 0;

volatile unsigned char txt = 0;
unsigned long currentTime;
void togText() {
  currentTime = millis();
  if (currentTime - lastPressText > debounceDelay && txt < 2) txt++;
  else if (txt >= 2) txt = 0;
  lastPressText = currentTime;
}

volatile bool axis = false; // true for vertical (voltage), false for horizontal (time)
void selAxis() {
  currentTime = millis();
  if (currentTime - lastPressAxis > debounceDelay) axis = !axis;
  lastPressAxis = currentTime;
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  pinMode(signalPin, OUTPUT);
  pinMode(toggleText, INPUT_PULLUP);
  pinMode(selectAxis, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(toggleText), togText, RISING);
  attachInterrupt(digitalPinToInterrupt(selectAxis), selAxis, RISING);
}

void loop() {
  vertScale = (float)analogRead(vertKnob) * 9.0 / 1023.0 + 1.0;
  unsigned short delayTime = 10*analogRead(horKnob);

  float avg = 0.0;
  unsigned long begin;
  for (int i = 0; i < SAMPLES; i++) {
    begin = micros();
    uReal[i] = (float)analogRead(posTerminal);
    if (delayTime >= 10) delayMicroseconds(delayTime);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
    // if delay is less than 10 us, there seem to be issues with the delay and how it shows up on LCD, so just remove delay.
    begin = micros() - begin;
  }

  for (int16_t i = 0; i < SAMPLES - 1; i++) {
    tft.drawLine(i, map((int)(uReal[i] * vertScale), 2047, 0, 0, 159), (i+1), 
                  map((int)(uReal[i+1] * vertScale), 2047, 0, 0, 159), ST7735_RED);
  }

  delay(100);

  for (int i = 0; i < SAMPLES; i++) {
    avg+=uReal[i];
  }

  avg /= (float)SAMPLES;

  unsigned long trueFreq = 1000000 / begin;
  float t_TOTms = 1000.0*(double)SAMPLES/(double)trueFreq;

  ArduinoFFT<float> FFT = ArduinoFFT<float>(uReal, uImag, SAMPLES, trueFreq);
  for (int i = 0; i < SAMPLES; i++) {
    uReal[i] = uReal[i] - avg;
    uImag[i] = 0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
  FFT.compute(FFTDirection::Forward);  // Perform FFT
  FFT.complexToMagnitude();
  float peak = FFT.majorPeak();
  Serial.println(String(vertScale, 2));
  tft.fillScreen(ST7735_BLACK);
  if (txt == 0) {
    if (axis) {
      tft.setCursor(0, 150);
      tft.print("0V");
      tft.setCursor(0, 0);
      tft.print(String(10.0/vertScale, 2));
      tft.print("V");
      for (int16_t i = 10; i < 150; i+=16) {
        tft.setCursor(0, i);
        tft.print("-");
      }
    } else {
      tft.setRotation(3);
      tft.setCursor(0,0);
      tft.print("t0");
      tft.setCursor(0, 118);
      tft.print("+");
      tft.print(String(t_TOTms, 2));
      tft.print("ms");
      for (int16_t i = 5; i < 118; i+=13) {
        tft.setCursor(0, i);
        tft.print("-");
      }
      tft.setRotation(0);
    }
    tft.setCursor(55, 0);
    tft.print("SFreq:");
    tft.print(String(trueFreq));
    tft.print("Hz");
  } else if (txt == 1) {
    tft.setCursor(40, 0);
    tft.print("DomFreq:");
    tft.print(String((int)peak));
    tft.print("Hz");
  }
}