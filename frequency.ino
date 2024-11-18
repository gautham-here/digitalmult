#define DAC_PIN_0 2
#define DAC_PIN_1 3
#define DAC_PIN_2 4
#define DAC_PIN_3 5
#define DAC_PIN_4 6
#define DAC_PIN_5 7

#define ANALOG_FEEDBACK_PIN A0
#define NUM_SAMPLES 64

volatile int index = 0;
bool in_error_state = false;
unsigned long error_start_time = 0;

float alpha = 0.1;  // Smoothing factor for exponential smoothing
float smoothed_feedback = 0;

// Debounce settings
int error_debounce_counter = 0;
const int debounce_threshold = 5;

// Error thresholds
const int error_threshold_low = 50;
const int error_threshold_high = 950;

// Triangular wave lookup table
const uint8_t triangular_wave[NUM_SAMPLES] = {
  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
  63, 59, 55, 51, 47, 43, 39, 35, 31, 27, 23, 19, 15, 11, 7, 3,
  0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60,
  63, 59, 55, 51, 47, 43, 39, 35, 31, 27, 23, 19, 15, 11, 7, 3
};

volatile unsigned long last_time = 0;
volatile unsigned long current_time = 0;
volatile float frequency = 0.0;

float amplitude = 0.0;
int max_value = 0;
int min_value = 1023;

void setup() {
  DDRD |= 0b11111100; // Set DAC pins as outputs
  Serial.begin(9600);
 
  // Faster analogRead setup for improved performance
  ADCSRA = (ADCSRA & 0xF8) | 0x04; // Set ADC prescaler to 16

  // Timer1 setup for waveform generation
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 249; // Generates ~1kHz triangular wave
  TCCR1B |= (1 << WGM12); // CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); // Prescaler 64
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  // Attach interrupt to detect rising edge for frequency calculation
  attachInterrupt(digitalPinToInterrupt(DAC_PIN_0), calculateFrequency, RISING);
}

ISR(TIMER1_COMPA_vect) {
  generateWaveform();
}

void generateWaveform() {
  uint8_t value = triangular_wave[index++];
  PORTD = (PORTD & 0x03) | (value << 2);
  if (index >= NUM_SAMPLES) index = 0;
}

void calculateFrequency() {
  current_time = micros(); // Get current time in microseconds
  if (last_time > 0) {
    unsigned long time_diff = current_time - last_time; // Time between rising edges
    frequency = 1000000.0 / time_diff; // Frequency in Hz (1 million microseconds = 1 second)
  }
  last_time = current_time; // Update last time for next interrupt
}

void processFeedback() {
  int currentReading = analogRead(ANALOG_FEEDBACK_PIN);

  // Exponential smoothing for noise reduction
  smoothed_feedback = alpha * currentReading + (1 - alpha) * smoothed_feedback;

  // Track min and max values for amplitude calculation
  if (currentReading > max_value) max_value = currentReading;
  if (currentReading < min_value) min_value = currentReading;

  // Calculate peak-to-peak amplitude
  amplitude = max_value - min_value;

  // Reset max and min values periodically for next calculation
  if (index == 0) {
    max_value = 0;
    min_value = 1023;
  }

  // Debugging output for monitoring feedback
  Serial.print("Smoothed Feedback: ");
  Serial.println(smoothed_feedback);

  // Error checking with debounce logic
  if (smoothed_feedback < error_threshold_low || smoothed_feedback > error_threshold_high) {
    error_debounce_counter++;
    Serial.print("Debounce Counter: ");
    Serial.println(error_debounce_counter);

    if (error_debounce_counter >= debounce_threshold) {
      handleError();
    }
  } else {
    error_debounce_counter = 0; // Reset debounce counter if feedback is within range
  }
}

void handleError() {
  if (!in_error_state) {
    Serial.println("Error detected! System in error state, attempting recovery...");
    in_error_state = true;
    error_start_time = millis();
  }
}

void attemptRecovery() {
  if (in_error_state && (millis() - error_start_time > 2000)) { // 2-second recovery window
    Serial.println("Attempting to recover...");
    error_debounce_counter = 0;
    in_error_state = false;
    Serial.println("Recovery successful, system back to normal.");
  }
}

void loop() {
  if (!in_error_state) {
    processFeedback();
  } else {
    attemptRecovery();
  }

  // Output the frequency and amplitude periodically
  Serial.print("Frequency: ");
  Serial.print(frequency);
  Serial.print(" Hz, Amplitude: ");
  Serial.print(amplitude);
  Serial.println(" units");
  delay(1000); // Update every second
}