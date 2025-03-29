#include <arduinoFFT.h>
#include <TimerOne.h>
#include <Adafruit_ST7735.h>

#define SAMPLES_OUT 128
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

const unsigned short debounceDelay = 500;
volatile unsigned long lastPressText = 0;
volatile unsigned long lastPressAxis = 0;

volatile unsigned char txt = 0;
void togText() {
  unsigned long currentTime = millis();
  if (currentTime - lastPressText > debounceDelay && txt < 2) txt++;
  else if (txt >= 2) txt = 0;
  lastPressText = currentTime;
}

volatile bool axis = false; // true for vertical (voltage), false for horizontal (time)
void selAxis() {
  unsigned long currentTime = millis();
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

void realResult(float a[], float b[], int N, float res[], int samplesOut, double totalTime) {
  for (int i = 0; i < samplesOut; i++) {
    float sum_real = 0.0;
    float sum_imag = 0.0;
    double timeVal = (double)i*totalTime/(double)samplesOut;
    for (int n = 0; n < N/2; n++) {
      sum_real += a[n]*cos(2.0*M_PI*(double)n*timeVal) - b[n]*sin(2.0*M_PI*(double)n*timeVal);
      sum_imag += a[n]*cos(2.0*M_PI*(double)n*timeVal) + b[n]*sin(2.0*M_PI*(double)n*timeVal);
    }
    res[i] = sum_real / (double)N;
  }
}

void loop() {
  unsigned short readHor = analogRead(horKnob);
  unsigned short samplesIn = constrain(map(readHor, 896, 1023, 0, 3), 0, 3);
  if (samplesIn == 0) samplesIn = 128;
  else samplesIn = 128 / pow(2, samplesIn);
  // if pot greater than 7/8, do 64, then 32, then 16 samples.
  vertScale = (float)analogRead(vertKnob) * 9.0 / 1023.0 + 1.0;

  float *uReal = malloc(samplesIn * sizeof(float));
  float *uImag = calloc(samplesIn, sizeof(float));

  unsigned short delayTime = 10*constrain(map(readHor, 896, 0, 0, 1023), 0, 1023);
  // sampling frequency increases as potentiometer reading increases, until 7/8 of total.
  // after this point, the sample # changes as noted above, and FFT interpolation is used.

  unsigned long interval;
  for (int i = 0; i < samplesIn; i++) {
    interval = micros();
    uReal[i] = (float)analogRead(posTerminal);
    if (delayTime >= 10) delayMicroseconds(delayTime);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
    // if delay is less than 10 us, there seem to be issues with the delay and how it shows up on LCD, so just remove delay.
    interval = micros() - interval;
  }

  unsigned long trueFreq = 1000000 / interval;
  double t_TOT = (double)samplesIn/(double)trueFreq;

  if (samplesIn == 128) {
    for (int i = 0; i < SAMPLES_OUT - 1; i++) {
      tft.drawLine(i, map((int)(uReal[i] * vertScale), 2047, 0, 0, 159), (i+1), 
                    map((int)(uReal[i+1] * vertScale), 2047, 0, 0, 159), ST7735_RED);
    }
  
    delay(100);
  }

  ArduinoFFT<float> FFT = ArduinoFFT<float>(uReal, uImag, samplesIn, trueFreq);

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);  // Apply Hamming window
  FFT.compute(FFTDirection::Forward);  // Perform FFT
  float* results;
  if (samplesIn != 128) {
    results = malloc(SAMPLES_OUT * sizeof(float));
    realResult(uReal, uImag, samplesIn, results, SAMPLES_OUT, t_TOT);
  }
  FFT.complexToMagnitude();
  float peak = FFT.majorPeak();
  free(uReal);
  free(uImag);

  tft.fillScreen(0);
  if (txt == 0) {
    if (axis) {
      tft.setCursor(0, 150);
      tft.print("0V");
      tft.setCursor(0, 0);
      tft.print(String(10.0/vertScale, 2));
      tft.print("V");
      for (unsigned short i = 10; i < 150; i+=16) {
        tft.setCursor(0, i);
        tft.print("-");
      }
    } else {
      tft.setRotation(3);
      tft.setCursor(0,0);
      tft.print("t0");
      tft.setCursor(0, 118);
      tft.print("+");
      tft.print(String(t_TOT*1000.0, 2));
      tft.print("ms");
      for (unsigned short i = 5; i < 118; i+=13) {
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

  if (samplesIn!=128){
    for (int i = 0; i < SAMPLES_OUT - 1; i++) {
      tft.drawLine(i, map((int)(results[i] * vertScale), 2047, 0, 0, 159), (i+1), 
                    map((int)(results[i+1] * vertScale), 2047, 0, 0, 159), ST7735_RED);
    }
    free(results);
    delay(100);
  }
}