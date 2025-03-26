  #include <arduinoFFT.h>
  #include <math.h>
  #include <TimerOne.h>
  #include <Adafruit_ST7735.h>

  #define MIN_SAMPLES_IN 8            // Number of minimum samples for FFT
  #define SAMPLES_OUT 128
  #define MAX_SAMPLING_FREQUENCY 100000  // Max. Sampling frequency in Hz (100kHz)
  #define MIN_SAMPLING_FREQUENCY 100
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
  #define horKnob A2

  unsigned long samplingFreq = 100000;
  uint8_t samplesIn = 128;
  double vertScale = 1.0;
  bool axisToggle = false; // false for vertical axis labels, true for horizontal

  // Array to hold the real and imaginary parts of the FFT

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
    vertScale = (double)analogRead(vertKnob) * 9.0 / 1023.0 + 1.0;
    samplingFreq = (unsigned long)((double)analogRead(horKnob) * (double)(MAX_SAMPLING_FREQUENCY - MIN_SAMPLING_FREQUENCY) 
                    / 1023.0 + (double)MIN_SAMPLING_FREQUENCY);
    Serial.println(samplingFreq);
    double uReal[samplesIn];
    unsigned long delayTime = (unsigned long)1000000 / samplingFreq;

    for (int i = 0; i < samplesIn; i++) {
      uReal[i] = (double)(analogRead(posTerminal) - analogRead(negTerminal));
      delayMicroseconds((unsigned int)delayTime);  // Wait for the next sample (based on SAMPLING_FREQUENCY)
    }

    tft.fillScreen(0);
    tft.setCursor(0, 80);
    tft.print("0V");
    tft.setCursor(0, 0);
    tft.print(String(10.0/vertScale, 2));
    tft.print("V");
    tft.setCursor(0, 150);
    tft.print(String(-10.0/vertScale, 2));
    tft.print("V");
    tft.setCursor(50, 0);
    tft.print("SFreq:");
    tft.print(String(samplingFreq));
    tft.print("Hz");


    for (int i = 0; i < samplesIn - 1; i++) {
      tft.drawLine(i * (128 / samplesIn), map((int)(uReal[i] * vertScale), MAXVAL, -1*MAXVAL, 0, 159), (i+1) * (128/samplesIn), 
                    map((int)(uReal[i+1] * vertScale), MAXVAL, -1*MAXVAL, 0, 159), ST7735_BLUE);
    }

    delay(100);
  }