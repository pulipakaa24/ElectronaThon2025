#include <arduinoFFT.h>
#include <math.h>
#include <TimerOne.h>
#include <Adafruit_ST7735.h>

#define MIN_SAMPLES_IN 8            // Number of minimum samples for FFT
#define SAMPLES_OUT 128
#define MAX_SAMPLING_FREQUENCY 10000  // Max. Sampling frequency in Hz (1 kHz)
#define SQUAREWAVE_FREQ 4000 // frequency of square wave output for testing
#define MAXVAL 2047

// Define the pins connected to your ST7735R display
#define TFT_CS    10  // Chip select pin (CS)
#define TFT_DC    9   // Data/Command pin (DC)
#define TFT_RST   8   // Reset pin (RST)
#define signalPin 5
#define posTerminal A0
#define negTerminal A5
#define vertKnob A1

unsigned int samplingFreq = 10000;
uint8_t samplesIn = 32;
double vertScale = 1.0;
unsigned int horScale = 1; // these scales have yet to be implemented fully

// Array to hold the real and imaginary parts of the FFT

double outResults[SAMPLES_OUT];
double timeSpacing[SAMPLES_OUT];

// Create FFT object
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

volatile bool signalState = LOW;

void toggleSignal() {
  signalState = !signalState;
  digitalWrite(signalPin, signalState);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST7735_BLACK);
  pinMode(signalPin, OUTPUT);

  Serial.println(1000000/(SQUAREWAVE_FREQ*2));
  Timer1.initialize(1000000/(SQUAREWAVE_FREQ*2));
  Timer1.attachInterrupt(toggleSignal);
}

void realResult(double a[], double b[], int N, double t[], int samplesOut, double res[]) {
  for (int i = 0; i < samplesOut; i++) {
    double sum = 0.0;
    for (int n = 0; n < (N+1)/2; n++) {
      sum += a[n]*cos(2*M_PI*(double)n*t[i]) - b[n]*sin(2*M_PI*(double)n*t[i]);
    }
    for (int n = (N+1)/2; n < N; n++) {
      sum += a[n]*cos(2*M_PI*(double)(n - N)*t[i]) - b[n]*sin(2*M_PI*(double)(n - N)*t[i]);
    }
    res[i] = sum;
  }
}

void loop() {
  double uReal[samplesIn];
  double uImag[samplesIn];
  ArduinoFFT<double> FFT = ArduinoFFT<double>(uReal, uImag, samplesIn, samplingFreq);
  double t_TOT = (double)samplesIn/(double)samplingFreq;

  for(int i = 0; i < SAMPLES_OUT; i++) {
    timeSpacing[i] = t_TOT*(double)i/(double)SAMPLES_OUT;
  }

  for (int i = 0; i < samplesIn; i++) {
    uReal[i] = (double)(analogRead(posTerminal) - analogRead(negTerminal));
    uImag[i] = 0;
    delayMicroseconds(1000000/samplingFreq);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
  }


  for (int i = 0; i < 31; i++) {
    tft.drawLine(i*4, map((int)(uReal[i] * vertScale), -1*MAXVAL, MAXVAL, 0, 159), (i+1)*4, 
                  map((int)(uReal[i+1] * vertScale), -1*MAXVAL, MAXVAL, 0, 159), ST7735_BLUE);
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
  FFT.compute(FFTDirection::Forward);  // Perform FFT

  realResult(uReal, uImag, samplesIn, timeSpacing, SAMPLES_OUT, outResults);

  // for (int i = 0; i < SAMPLES_OUT; i++) {
  //   Serial.println(outResults[i]);
  // }

  tft.fillScreen(0);
  // for (int i = 0; i < 127; i++) {
  //   tft.drawLine(i, map((int)(outResults[i] * vertScale), -1*MAXVAL, MAXVAL, 0, 159), i+1, 
  //                 map((int)(outResults[i+1] * vertScale), -1*MAXVAL, MAXVAL, 0, 159), ST7735_BLUE);
  // }

  vertScale = (double)analogRead(vertKnob) * 9.0 / 1023.0 + 1.0;
  Serial.println(analogRead(vertKnob));
  Serial.println(vertScale);
  delay(100);
}