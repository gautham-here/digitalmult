#define DAC_PIN_0 2
#define DAC_PIN_1 3
#define DAC_PIN_2 4
#define DAC_PIN_3 5
#define DAC_PIN_4 6
#define DAC_PIN_5 7

#define ANALOG_FEEDBACK_PIN A0
#define NUM_SAMPLES 256
#define BUFFER_SIZE 10

volatile int index = 0;
bool error_detected = false;
bool in_error_state = false;
unsigned long error_start_time = 0;

float alpha = 0.05;  // Reduced smoothing factor
float smoothed_feedback = 0;

// Debounce settings
int error_debounce_counter = 0;
const int debounce_threshold = 5; // Number of consecutive errors needed to trigger error state

// Error thresholds
const int error_threshold_low = 100;   // Increased lower threshold
const int error_threshold_high = 900;  // Adjusted higher threshold

// Sine wave lookup table
const uint16_t sine_wave[NUM_SAMPLES] = {
  0, 804, 1608, 2410, 3209, 4005, 4797, 5585,
  6367, 7143, 7913, 8676, 9432, 10180, 10920, 11651,
  12372, 13084, 13785, 14476, 15155, 15822, 16477, 17119,
  17748, 18363, 18964, 19551, 20124, 20682, 21225, 21754,
  22267, 22765, 23247, 23714, 24164, 24598, 25015, 25415,
  25799, 26166, 26515, 26847, 27162, 27459, 27739, 28001,
  28247, 28475, 28685, 28877, 29052, 29209, 29349, 29471,
  29576, 29663, 29733, 29785, 29819, 29836, 29835, 29816
};

void setup() {
  DDRD |= 0b11111100; // Set DAC pins as outputs
  Serial.begin(9600);

  // Faster analogRead setup
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // Set ADC prescaler to 16

  // Timer setup for waveform generation
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 63;
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS11);  // Prescaler 8
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  generateWaveform();
}

void generateWaveform() {
  uint16_t value = sine_wave[index++];
  // Split the high and low bytes properly between the DAC pins
  PORTD = (PORTD & 0x03) | ((value >> 8) << 2);  // Set the higher bits
  PORTD = (PORTD & 0xFC) | (value & 0xFF);      // Set the lower bits
  if (index >= NUM_SAMPLES) index = 0;
}

void processFeedback() {
  int currentReading = analogRead(ANALOG_FEEDBACK_PIN);

  // Exponential smoothing for noise reduction
  smoothed_feedback = alpha * currentReading + (1 - alpha) * smoothed_feedback;

  Serial.println(smoothed_feedback);

  // Error checking with debounce logic
  if (smoothed_feedback < error_threshold_low || smoothed_feedback > error_threshold_high) {
    error_debounce_counter++;
    if (error_debounce_counter > debounce_threshold) {
      handleError();
    }
  } else {
    error_debounce_counter = 0; // Reset debounce counter if within range
  }
}

void handleError() {
  if (!in_error_state) {
    Serial.println("System in error state, attempting recovery...");
    in_error_state = true;
    error_start_time = millis();
  }
}

void attemptRecovery() {
  if (in_error_state && (millis() - error_start_time > 2000)) { // 2-second recovery window
    Serial.println("Attempting to recover...");
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