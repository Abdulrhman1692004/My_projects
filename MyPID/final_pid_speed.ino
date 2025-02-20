const int R_PWM = 5; 
const int L_PWM = 6; 
const int speed = 255; 
 
const int encoderPinA = 2; 
const int encoderPinB = 3; 
 
volatile int pulseCount = 0; 
volatile int initial_pulseCount = 0; 
int b; 
 
unsigned long prevTime = 0; 
volatile double rpm = 0; 
 
const int APR = 360; 
int samplingFrequency = 5000; // Default sampling frequency in Hz (5000 Hz) 
int cutoffFrequency = 500;    // Default cutoff frequency in Hz (500 Hz) 
int filterWindow = samplingFrequency / (2 * cutoffFrequency); // Filter window size 
 
const int FILTER_SIZE = 4;  // Reduced number of samples for the moving average filter 
 
double rpmReadings[FILTER_SIZE];  // Array to hold RPM readings for the filter 
int currentReadingIndex = 0;  // Index for the current RPM reading 
 
void setup() { 
  pinMode(R_PWM, OUTPUT); 
  pinMode(L_PWM, OUTPUT); 
  Serial.begin(9600); 
 
  // Set the encoders 
  pinMode(encoderPinA, INPUT_PULLUP); 
  pinMode(encoderPinB, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, RISING); 
 
  // Initialize all readings to 0 
  for (int i = 0; i < FILTER_SIZE; i++) { 
    rpmReadings[i] = 0.0; 
  } 
} 
 
void loop() { 
  right(); 
  measure_Speed(); 
} 
 
void right() { 
  analogWrite(R_PWM, speed); 
  analogWrite(L_PWM, 0); 
} 
 
void updateEncoder() { 
  b = digitalRead(encoderPinB); 
  if (b > 0) { 
    pulseCount++; 
  } else { 
    pulseCount--; 
  } 
} 
 
void measure_Speed() { 
  unsigned long currentTime = micros(); // Time in microseconds 
 
  if (currentTime - prevTime >= 1000000 / samplingFrequency) { // Update based on sampling frequency 
    noInterrupts(); // Disable interrupts to ensure an accurate count 
 
    double elapsedTime = (currentTime - prevTime) / 1000000.0; // Find the dt in seconds 
    long passed_angle = pulseCount - initial_pulseCount; // Find the d8 
 
    double currentRpm = ((double(passed_angle) / elapsedTime) / APR) * 60; // Find the d8/dt and convert to RPM 
 
    // Store the current RPM reading in the array 
    rpmReadings[currentReadingIndex] = currentRpm; 
    currentReadingIndex = (currentReadingIndex + 1) % FILTER_SIZE; 
 
    // Calculate the average RPM 
    double sum = 0.0; 
    for (int i = 0; i < FILTER_SIZE; i++) { 
      sum += rpmReadings[i]; 
    } 
    rpm = sum / FILTER_SIZE; 
    int real_rpm = 0.724307 * rpm - 1.21882; 
 
    // Debug prints 
    Serial.print("Passed angle = "); 
    Serial.print(passed_angle); 
    Serial.print(" | Elapsed time = "); 
    Serial.print(elapsedTime); 
    Serial.print(" | RPM = "); 
    Serial.print(real_rpm); 
    Serial.println(); 
 
    initial_pulseCount = pulseCount; // Refresh the initial pulse count 
    prevTime = currentTime; // Update the last measurement time 
    interrupts(); // Re-enable interrupts 
  } 
}