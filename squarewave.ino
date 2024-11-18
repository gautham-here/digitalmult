// DAC Output Pins
#define DAC_PIN_0 2
#define DAC_PIN_1 3
#define DAC_PIN_2 4
#define DAC_PIN_3 5
#define DAC_PIN_4 6
#define DAC_PIN_5 7

// Analog Feedback Pin
#define ANALOG_FEEDBACK_PIN A0

// Waveform Settings
constexpr int SQUARE_WAVE_FREQ = 1000; // Frequency in Hz

// Error Handling Thresholds
constexpr int ERROR_THRESHOLD_LOW = 30;  // Increase/decrease as needed
constexpr int ERROR_THRESHOLD_HIGH = 970; // Increase/decrease as needed
constexpr int DEBOUNCE_THRESHOLD = 10; // Increase this value to reduce false errors
constexpr int RECOVERY_TIME_MS = 2000; // 2 seconds recovery window

// Smoothing Factor for Exponential Smoothing
constexpr float SMOOTHING_FACTOR = 0.2;

// Global Variables
volatile bool square_wave_state = false;
volatile int error_debounce_counter = 0;
volatile bool in_error_state = false;
float smoothed_feedback = 0;
unsigned long error_start_time = 0;

// Function Prototypes
void setupTimer();
void generateSquareWave();
void processFeedback();
void handleError();
void attemptRecovery();

void setup() {
  // Set DAC pins as outputs
  DDRD |= 0b11111100;

  Serial.begin(9600);

  // Set ADC prescaler to 16 for faster analog reads
  ADCSRA = (ADCSRA & 0xF8) | 0x04;

  // Setup Timer for square wave generation
  setupTimer();
}

// Timer1 Setup for Square Wave Generation
void setupTimer() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = (F_CPU / (2 * SQUARE_WAVE_FREQ * 64)) - 1;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS11) | (1 << CS10);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

// Interrupt Service Routine for Square Wave Generation
ISR(TIMER1_COMPA_vect) {
  generateSquareWave();
}

// Function to Generate Square Wave
void generateSquareWave() {
  square_wave_state = !square_wave_state;
  if (square_wave_state) {
    PORTD |= 0b11111100;
  } else {
    PORTD &= ~0b11111100;
  }
}

// Function to Read Feedback and Perform Error Detection
void processFeedback() {
  int currentReading = analogRead(ANALOG_FEEDBACK_PIN);

  // Apply Exponential Smoothing
  smoothed_feedback = SMOOTHING_FACTOR * currentReading + (1 - SMOOTHING_FACTOR) * smoothed_feedback;

  // Apply Moving Average Filter (optional)
  static float moving_average = 0;
  moving_average = (moving_average * 0.9) + (smoothed_feedback * 0.1);

  //Serial.print("Smoothed Feedback: ");
  Serial.println(smoothed_feedback);

  // Error Detection with Debounce Logic
  if (moving_average < ERROR_THRESHOLD_LOW || moving_average > ERROR_THRESHOLD_HIGH) {
    error_debounce_counter++;
    if (error_debounce_counter >= DEBOUNCE_THRESHOLD) {
      handleError();
    }
  } else {
    error_debounce_counter = 0;
  }
}

// Function to Handle Error State
void handleError() {
  if (!in_error_state) {
    Serial.println("Error detected! Entering error state...");
    in_error_state = true;
    error_start_time = millis();
  }
}

// Function to Attempt System Recovery
void attemptRecovery() {
  if (millis() - error_start_time > RECOVERY_TIME_MS) {
    Serial.println("Attempting recovery...");
    error_debounce_counter = 0;
    in_error_state = false;
  }
}

void loop() {
  if (!in_error_state) {
    processFeedback();
  } else {
    attemptRecovery();
  }
}