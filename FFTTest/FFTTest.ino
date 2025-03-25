#include <arduinoFFT.h>
#include <math.h>
// #include <fix_fft.h>
#include <TimerOne.h>
#include <Adafruit_ST7735.h>

#define SAMPLES_IN 32            // Number of samples for FFT
#define SAMPLES_OUT 128
#define SAMPLING_FREQUENCY 1000  // Sampling frequency in Hz (1 kHz)
#define SQUAREWAVE_FREQ 100 // frequency of square wave output for testing
#define MAXVAL 1023

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)
#define signalPin 5
#define posTerminal A0
#define negTerminal A5

// Array to hold the real and imaginary parts of the FFT
double uReal[SAMPLES_IN];
double uImag[SAMPLES_IN];
double outResults[SAMPLES_OUT];
double timeSpacing[SAMPLES_OUT];

double t_TOT = (double)SAMPLES_IN/(double)SAMPLING_FREQUENCY;

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

  for(int i = 0; i < SAMPLES_OUT; i++) {
    timeSpacing[i] = t_TOT*(double)i/SAMPLES_OUT;
  }

  Timer1.initialize(1000000/(SQUAREWAVE_FREQ*2));
  Timer1.attachInterrupt(toggleSignal);
}

void realResult(double a[], double b[], int N, double t[], int samplesOut, double res[]) {
  for (int i = 0; i < samplesOut; i++) {
    double sum = 0.0;
    for (int n = 0; n < (N+1)/2 + 1; n++) {
      sum += a[n]*cos(2*M_PI*(double)n*t[i]) - b[n]*sin(2*M_PI*(double)n*t[i]);
    }
    for (int n = (N+1)/2 + 1; n < N; n++) {
      sum += a[n]*cos(2*M_PI*(double)(n - N)*t[i]) - b[n]*sin(2*M_PI*(double)(n - N)*t[i]);
    }
    res[i] = sum;
  }
}

void loop() {
  for (int i = 0; i < SAMPLES_IN; i++) {
    uReal[i] = analogRead(posTerminal) - analogRead(negTerminal);
    delayMicroseconds(1000);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
  }

  // FFT.setArrays(uReal, uImag, SAMPLES_IN);

  // Perform FFT
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
  FFT.compute(FFTDirection::Forward);  // Perform FFT
  
  // FFT.complexToMagnitude();
  // Serial.println(FFT.majorPeak());
  // FFT.setArrays(vReal, vImag, SAMPLES_OUT);
  // FFT.compute(FFTDirection::Reverse); // Inverse FFT

  realResult(uReal, uImag, SAMPLES_IN, timeSpacing, SAMPLES_OUT, outResults);

  tft.fillScreen(0);
  for (int i = 0; i < 127; i++) {
    tft.drawLine(i, constrain(map(outResults[i], -1*MAXVAL, MAXVAL, 0, 159), 0, 159), i+1, 
                  constrain(map(outResults[i+1], -1*MAXVAL, MAXVAL, 0, 159), 0, 159), ST7735_BLUE);
  }
  delay(100);
}