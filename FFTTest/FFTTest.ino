#include <arduinoFFT.h>
#include <TimerOne.h>
#include <Adafruit_ST7735.h>

#define SAMPLES_IN 32            // Number of samples for FFT
#define SAMPLES_OUT 128
#define SAMPLING_FREQUENCY 1000  // Sampling frequency in Hz (1 kHz)
#define signalPin 5

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)

// Array to hold the real and imaginary parts of the FFT
double uReal[SAMPLES_IN];
double uImag[SAMPLES_IN];
double vReal[SAMPLES_OUT];
double vImag[SAMPLES_OUT];

// Create FFT object
ArduinoFFT<double> FFT = ArduinoFFT<double>(uReal, uImag, SAMPLES_IN, SAMPLING_FREQUENCY);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
// Variables for signal input
int analogPin = A5;  // Pin where your signal is connected

volatile bool signalState = LOW;

void toggleSignal() {
  signalState = !signalState;
  digitalWrite(signalPin, signalState);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);
  
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  pinMode(signalPin, OUTPUT);

  Timer1.initialize(8000);
  Timer1.attachInterrupt(toggleSignal);
}

void loop() {
  // Nothing to do here, as we're just doing a one-time FFT on the samples
  // Sampling the signal at a fixed rate
  // FFT.setArrays(uReal, uImag, SAMPLES_IN);
  for (int i = 0; i < SAMPLES_IN; i++) {
    uReal[i] = analogRead(analogPin);  // Read analog input
    uImag[i] = 0;  // Imaginary part is 0 for real signal
    delayMicroseconds(1000);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
  }

  // Perform FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
  FFT.compute(FFTDirection::Forward);  // Perform FFT
  FFT.complexToMagnitude();
  Serial.println(FFT.majorPeak());
  // FFT.setArrays(vReal, vImag, SAMPLES_OUT);
  // FFT.compute(FFTDirection::Reverse); // Inverse FFT

  // for (int i = 0; i < 32; i++) {
  //   Serial.print(i);
  //   Serial.print("|");
  //   Serial.print(uReal[i]);
  //   Serial.print("|");
  //   Serial.println(uImag[i]);
  // }
  tft.fillScreen(0);
  for (int i = 0; i < 127; i++) {
    // Serial.println(vReal[i]);
    tft.drawLine(i, constrain(map(uReal[i], 0, 1023, 0, 159), 0, 159), i+1,
                constrain(map(uReal[i+1], 0, 1023, 0, 159), 0, 159), ST7735_WHITE);
  }
  delay(100);
}