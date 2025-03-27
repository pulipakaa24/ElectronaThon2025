#include <arduinoFFT.h>
#include <Adafruit_ST7735.h>

#define SAMPLES_IN 32            // Number of samples for the low-res signal
#define SAMPLES_OUT 128          // Desired upsampled signal resolution
#define SAMPLING_FREQUENCY 1000  // Sampling frequency in Hz (1 kHz)

#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)

#define sigPin 5

double uReal[SAMPLES_IN];  // Low-res signal
double uImag[SAMPLES_IN];  
double vReal[SAMPLES_OUT];  // High-res signal after IFFT
double vImag[SAMPLES_OUT];

// Create FFT object
ArduinoFFT<double> FFT = ArduinoFFT<double>(uReal, uImag, SAMPLES_IN, SAMPLING_FREQUENCY);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// Pin for analog input
int analogPin = A5;

void setup() {
  Serial.begin(9600);
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  pinMode(sigPin, OUTPUT);
}

void loop() {
  // Sample the low-resolution signal
  analogWrite(sigPin, 122);
  for (int i = 0; i < SAMPLES_IN; i++) {
    uReal[i] = analogRead(analogPin);  // Read analog input
    uImag[i] = 0;                      // Imaginary part is 0 for real signals
    delayMicroseconds(1000);           // Sample at 1 kHz
  }

  // Apply Hann window to reduce spectral leakage
  FFT.windowing(FFTWindow::Hann, FFTDirection::Forward);
  
  // Perform FFT on the original signal
  FFT.compute(FFTDirection::Forward);

  // Zero-pad the frequency domain to upscale the resolution
  for (int i = 0; i < SAMPLES_IN / 2; i++) {
    vReal[i] = uReal[i];
    vImag[i] = uImag[i];
    Serial.println(vReal[i]);
  }
  
  for (int i = SAMPLES_IN / 2; i < SAMPLES_OUT - SAMPLES_IN / 2; i++) {
    vReal[i] = 0;  // Zero-padding in the middle
    vImag[i] = 0;
    Serial.println(vReal[i]);
  }
  
  for (int i = SAMPLES_OUT - SAMPLES_IN / 2; i < SAMPLES_OUT; i++) {
    vReal[i] = uReal[i - (SAMPLES_OUT - SAMPLES_IN)];
    vImag[i] = uImag[i - (SAMPLES_OUT - SAMPLES_IN)];
    Serial.println(vReal[i]);
  }

  delay(10000);
  // Perform inverse FFT (IFFT) to get higher-resolution signal
  FFT.setArrays(vReal, vImag, SAMPLES_OUT);
  FFT.compute(FFTDirection::Reverse);  // IFFT to reconstruct the signal

  // Normalize the IFFT result
  for (int i = 0; i < SAMPLES_OUT; i++) {
    vReal[i] /= SAMPLES_OUT;
  }

  // Plot the higher-resolution signal on the TFT
  tft.fillScreen(ST7735_BLACK);
  for (int i = 0; i < SAMPLES_OUT - 1; i++) {
    tft.drawLine(i, map(vReal[i], 0, 1023, 0, 159), i + 1,
                 map(vReal[i + 1], 0, 1023, 0, 159), ST7735_WHITE);
  }

  delay(100);
}