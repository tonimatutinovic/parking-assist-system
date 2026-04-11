#include <SoftwareSerial.h>

/*
  Pin configuration
  RX_PIN: Arduino receives data from sensor (connect sensor TX here)
  TX_PIN: Not used in this project (sensor transmits automatically)
  BUZZER_PIN: Passive buzzer output pin
  REVERSE PIN: Simulates vehicle reverse gear activation using a push button
  HC_TRIG_PIN: Trigger output pin used to start measurement
  HC_ECHO_PIN: Input pin used to measure echo pulse duration
*/

#define RX_PIN 11
#define TX_PIN 10
#define BUZZER_PIN 3
#define REVERSE_PIN 2
#define HC_TRIG_PIN 5
#define HC_ECHO_PIN 6

// Special value used to indicate that audio output should be disabled
#define AUDIO_OFF_INTERVAL 0xFFFFFFFFUL

// Distance zones for acoustic warning
typedef enum
{
    ZONE_SAFE,
    ZONE_SLOW,
    ZONE_MEDIUM,
    ZONE_FAST,
    ZONE_CRITICAL
} zone_t;

/*
  System states
*/
typedef enum
{
    STATE_IDLE,
    STATE_ACTIVE,
    STATE_FAULT
} system_state_t;

/*
  Fault types
*/
typedef enum
{
    FAULT_NONE,
    FAULT_TIMEOUT,
    FAULT_SENSOR_INVALID,
    FAULT_DIRTY_SENSOR
} fault_t;

// Current active zone
zone_t currentZone = ZONE_SAFE;

// Current and previous system state
system_state_t currentState = STATE_IDLE;
system_state_t previousState = STATE_IDLE;

// Current active fault
fault_t currentFault = FAULT_NONE;

// Fault flag derived from current fault type
bool faultDetected = false;

// Software UART for ultrasonic sensor communication
SoftwareSerial sensorSerial(RX_PIN, TX_PIN);

// Buffer for incoming UART packet (4 bytes)
byte buffer[4];
int indexBuffer = 0;

// Latest measured distance (in cm)
float distance = 0;

/*
  Audio control variables
  audioInterval:
    0                  -> continuous tone
    AUDIO_OFF_INTERVAL -> no sound
    any other value    -> beep interval in milliseconds
*/
unsigned long lastAudioToggle = 0;
unsigned long audioInterval = AUDIO_OFF_INTERVAL;
bool audioToneOn = false;
unsigned int audioFrequency = 2800;

// Timestamp marking the start of the fault audio pattern cycle
unsigned long faultCycleStartTime = 0;

// Number of consecutive invalid sensor packets
uint8_t invalidPacketCount = 0;

// Threshold for invalid packet fault activation
const uint8_t INVALID_PACKET_THRESHOLD = 5;

/*
  Timeout handling
  If no data is received from sensor within TIMEOUT,
  a timeout fault can be triggered while reverse is active
*/
const unsigned long TIMEOUT = 200; // milliseconds
unsigned long lastPacketTime = 0;
bool sensorDataValid = false;

/*
  Dirty sensor detection variables

  Dirty sensor is detected when valid packets continue to arrive,
  but filtered distance remains abnormally unstable over time.
*/
float previousFilteredDistance = 0.0;
bool previousDistanceValid = false;

uint8_t distanceJumpCount = 0;

// Dirty sensor detection thresholds
const float DIRTY_JUMP_THRESHOLD_CM = 12.0;
const uint8_t DIRTY_SENSOR_THRESHOLD = 8;
const uint8_t DIRTY_SENSOR_RECOVERY_THRESHOLD = 3;

// Moving average filter
#define FILTER_SIZE 5

float filterBuffer[FILTER_SIZE] = {0};
int filterIndex = 0;
bool filterFilled = false;

// Curb detection variables
float curbDistance = -1.0;                   // Latest filtered curb distance value (cm)
static float lastValidCurbDistance = -1.0;   // Stores last valid measurement to filter out invalid readings (-1)
unsigned long lastCurbReadTime = 0;          // Timestamp of last measurement
const unsigned long CURB_READ_INTERVAL = 50; // Minimum interval between measurements

// Curb detection configuration
const float CURB_CRITICAL_THRESHOLD = 30.0; // Distance below which curb is considered dangerous
bool curbCritical = false;                  // Flag indicating emergency curb proximity

// Curb detection stabilization
const float CURB_ON_THRESHOLD = 35.0;
const float CURB_OFF_THRESHOLD = 45.0;

unsigned long curbTriggerTime = 0;
const unsigned long CURB_MIN_ACTIVE_TIME = 150;

bool curbActive = false;

/*
  Applies moving average filter to incoming distance values
  Reduces noise and prevents rapid fluctuations in measurements

  Parameters:
    newValue - latest raw distance measurement (cm)

  Returns:
    filtered distance value (cm)
*/
float applyMovingAverage(float newValue)
{
    filterBuffer[filterIndex] = newValue;          // Store new measurement
    filterIndex = (filterIndex + 1) % FILTER_SIZE; // Move index forward using modulo

    // Mark buffer as filled after first full cycle
    if (filterIndex == 0)
        filterFilled = true;

    // Determine how many valid samples we currently have
    int count = filterFilled ? FILTER_SIZE : filterIndex;

    // Compute average of available samples
    float sum = 0;
    for (int i = 0; i < count; i++)
    {
        sum += filterBuffer[i];
    }

    return sum / count;
}

/*
  Reads distance from HC-SR04 ultrasonic sensor

  Returns:
    distance in cm
    -1.0 if no valid echo received (timeout)
*/
float readCurbDistanceCm()
{
    digitalWrite(HC_TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(HC_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(HC_TRIG_PIN, LOW);

    unsigned long duration = pulseIn(HC_ECHO_PIN, HIGH, 30000);

    if (duration == 0)
    {
        return -1.0;
    }

    return duration * 0.0343 / 2.0;
}

/*
  Updates zone based on distance using hysteresis
  Prevents rapid switching between zones near boundaries

  Parameters:
    d - distance (cm),
    current - current active zone

  Returns:
    updated zone
*/
zone_t updateZone(float d, zone_t current)
{
    switch (current)
    {
    case ZONE_SAFE:
        if (d < 95)
            return ZONE_SLOW;
        break;

    case ZONE_SLOW:
        if (d < 55)
            return ZONE_MEDIUM;
        if (d > 105)
            return ZONE_SAFE;
        break;

    case ZONE_MEDIUM:
        if (d < 35)
            return ZONE_FAST;
        if (d > 65)
            return ZONE_SLOW;
        break;

    case ZONE_FAST:
        if (d < 18)
            return ZONE_CRITICAL;
        if (d > 45)
            return ZONE_MEDIUM;
        break;

    case ZONE_CRITICAL:
        if (d > 22)
            return ZONE_FAST;
        break;
    }

    return current;
}

/*
  Starts buzzer tone only if it is currently OFF
*/
void startAudioTone()
{
    if (!audioToneOn)
    {
        tone(BUZZER_PIN, audioFrequency);
        audioToneOn = true;
    }
}

/*
  Stops buzzer tone only if it is currently ON
*/
void stopAudioTone()
{
    if (audioToneOn)
    {
        noTone(BUZZER_PIN);
        audioToneOn = false;
    }
}

/*
  Sets audio behavior based on current warning zone

  For now, all zones use the same frequency and only differ in beep interval.
  This function is intentionally separated so future versions can assign:
  - different tones to different sensors
  - different sound patterns to fault states
*/
void setAudioFromZone(zone_t zone)
{
    audioFrequency = 2800;

    switch (zone)
    {
    case ZONE_SAFE:
        audioInterval = AUDIO_OFF_INTERVAL;
        break;

    case ZONE_SLOW:
        audioInterval = 500;
        break;

    case ZONE_MEDIUM:
        audioInterval = 200;
        break;

    case ZONE_FAST:
        audioInterval = 80;
        break;

    case ZONE_CRITICAL:
        audioInterval = 0;
        break;
    }
}

/*
  Sets base audio configuration for fault conditions
  Fault patterns are played by a dedicated pattern handler.
  This function only selects the tone frequency.
*/
void setAudioFromFault(fault_t fault)
{
    switch (fault)
    {
    case FAULT_TIMEOUT:
        audioFrequency = 1500;
        break;

    case FAULT_SENSOR_INVALID:
        audioFrequency = 2000;
        break;

    case FAULT_DIRTY_SENSOR:
        audioFrequency = 1100;
        break;

    default:
        audioFrequency = 2800;
        break;
    }
}

/*
  Updates passive buzzer output using non-blocking timing

  Behavior:
  - outdated sensor data -> audio disabled
  - interval == 0        -> continuous tone
  - interval == OFF      -> no sound
  - otherwise            -> periodic beeping
*/
void updateAudio()
{
    if (audioInterval == 0)
    {
        startAudioTone();
        return;
    }

    if (audioInterval == AUDIO_OFF_INTERVAL)
    {
        stopAudioTone();
        return;
    }

    if (millis() - lastAudioToggle >= audioInterval)
    {
        lastAudioToggle = millis();

        if (audioToneOn)
        {
            stopAudioTone();
        }
        else
        {
            startAudioTone();
        }
    }
}

/*
  Plays fault-specific audio patterns using non-blocking timing

  Patterns:
  - FAULT_TIMEOUT        -> 3 long beeps
  - FAULT_SENSOR_INVALID -> 2 long beeps
  - FAULT_DIRTY_SENSOR   -> 1 very long beep + short pause
*/
void updateFaultAudio(fault_t fault)
{
    const unsigned long LONG_BEEP_ON = 400;
    const unsigned long LONG_BEEP_OFF = 250;
    const unsigned long PATTERN_PAUSE = 1000;

    const unsigned long DIRTY_BEEP_ON = 900;
    const unsigned long DIRTY_BEEP_OFF = 300;

    unsigned long now = millis();

    switch (fault)
    {
    case FAULT_TIMEOUT:
    {
        const uint8_t beepCount = 3;
        unsigned long cycleDuration =
            beepCount * (LONG_BEEP_ON + LONG_BEEP_OFF) + PATTERN_PAUSE;

        unsigned long cycleTime = (now - faultCycleStartTime) % cycleDuration;

        for (uint8_t i = 0; i < beepCount; i++)
        {
            unsigned long onStart = i * (LONG_BEEP_ON + LONG_BEEP_OFF);
            unsigned long onEnd = onStart + LONG_BEEP_ON;

            if (cycleTime >= onStart && cycleTime < onEnd)
            {
                startAudioTone();
                return;
            }
        }

        stopAudioTone();
        return;
    }

    case FAULT_SENSOR_INVALID:
    {
        const uint8_t beepCount = 2;
        unsigned long cycleDuration =
            beepCount * (LONG_BEEP_ON + LONG_BEEP_OFF) + PATTERN_PAUSE;

        unsigned long cycleTime = (now - faultCycleStartTime) % cycleDuration;

        for (uint8_t i = 0; i < beepCount; i++)
        {
            unsigned long onStart = i * (LONG_BEEP_ON + LONG_BEEP_OFF);
            unsigned long onEnd = onStart + LONG_BEEP_ON;

            if (cycleTime >= onStart && cycleTime < onEnd)
            {
                startAudioTone();
                return;
            }
        }

        stopAudioTone();
        return;
    }

    case FAULT_DIRTY_SENSOR:
    {
        unsigned long cycleDuration = DIRTY_BEEP_ON + DIRTY_BEEP_OFF;
        unsigned long cycleTime = (now - faultCycleStartTime) % cycleDuration;

        if (cycleTime < DIRTY_BEEP_ON)
        {
            startAudioTone();
        }
        else
        {
            stopAudioTone();
        }
        return;
    }

    default:
        stopAudioTone();
        return;
    }
}

/*
  Updates system state based on reverse input and fault condition
*/
system_state_t updateSystemState(bool reverseActive, bool faultDetected, system_state_t current)
{
    switch (current)
    {
    case STATE_IDLE:
        if (reverseActive)
        {
            if (faultDetected)
                return STATE_FAULT;
            else
                return STATE_ACTIVE;
        }
        break;

    case STATE_ACTIVE:
        if (!reverseActive)
            return STATE_IDLE;

        if (faultDetected)
            return STATE_FAULT;
        break;

    case STATE_FAULT:
        if (!reverseActive)
            return STATE_IDLE;

        if (!faultDetected)
            return STATE_ACTIVE;
        break;
    }

    return current;
}

/*
  Detects current system fault

  Implemented faults:
  - timeout fault
  - invalid sensor packet fault
*/
fault_t detectFault(bool reverseActive)
{
    if (reverseActive && sensorDataValid && ((millis() - lastPacketTime) > TIMEOUT))
    {
        return FAULT_TIMEOUT;
    }

    if (reverseActive && invalidPacketCount >= INVALID_PACKET_THRESHOLD)
    {
        return FAULT_SENSOR_INVALID;
    }

    if (reverseActive && sensorDataValid)
    {
        if (currentFault == FAULT_DIRTY_SENSOR)
        {
            if (distanceJumpCount > DIRTY_SENSOR_RECOVERY_THRESHOLD)
            {
                return FAULT_DIRTY_SENSOR;
            }
        }
        else
        {
            if (distanceJumpCount >= DIRTY_SENSOR_THRESHOLD)
            {
                return FAULT_DIRTY_SENSOR;
            }
        }
    }

    return FAULT_NONE;
}

/*
  Updates dirty sensor detection based on filtered distance stability

  If filtered distance changes too abruptly multiple times in a row,
  the sensor is considered degraded (e.g. dirty, wet, or partially blocked).
*/
void updateDirtySensorMonitor(float filteredDistance)
{
    if (!previousDistanceValid)
    {
        previousFilteredDistance = filteredDistance;
        previousDistanceValid = true;
        return;
    }

    float delta = abs(filteredDistance - previousFilteredDistance);

    if (delta >= DIRTY_JUMP_THRESHOLD_CM)
    {
        if (distanceJumpCount < 255)
        {
            distanceJumpCount++;
        }
    }
    else
    {
        if (distanceJumpCount >= 2)
        {
            distanceJumpCount -= 2;
        }
        else
        {
            distanceJumpCount = 0;
        }
    }

    previousFilteredDistance = filteredDistance;
}

/*
  Resets fault-related runtime data after fault recovery
*/
void resetFaultLogic()
{
    invalidPacketCount = 0;
    indexBuffer = 0;
    stopAudioTone();
    audioInterval = AUDIO_OFF_INTERVAL;

    previousFilteredDistance = 0.0;
    previousDistanceValid = false;
    distanceJumpCount = 0;
}

/*
  Resets runtime warning logic when system becomes inactive
*/
void resetSystemLogic()
{
    currentZone = ZONE_SAFE;
    audioInterval = AUDIO_OFF_INTERVAL;
    indexBuffer = 0;
    sensorDataValid = false;
    invalidPacketCount = 0;

    previousFilteredDistance = 0.0;
    previousDistanceValid = false;
    distanceJumpCount = 0;
}

void setup()
{
    Serial.begin(115200);     // Serial communication for debugging (PC)
    sensorSerial.begin(9600); // Sensor UART communication (fixed at 9600 baud)

    pinMode(BUZZER_PIN, OUTPUT);        // Configure buzzer pin as output
    pinMode(REVERSE_PIN, INPUT_PULLUP); // Configure reverse input with internal pull-up resistor
    pinMode(HC_TRIG_PIN, OUTPUT);       // TRIG is output (generating pulse)
    pinMode(HC_ECHO_PIN, INPUT);        // ECHO is input (measuring pulse width)
}

void loop()
{
    // Read reverse state (active LOW)
    bool reverseActive = (digitalRead(REVERSE_PIN) == LOW);

    // If reverse is OFF, system stays idle
    if (!reverseActive)
    {
        currentState = STATE_IDLE;
        currentFault = FAULT_NONE;
        faultDetected = false;
        stopAudioTone();
        resetSystemLogic();
        return;
    }

    /*
      Read incoming UART data from sensor
      Sensor continuously sends 4-byte packets:
      [0] = 0xFF (start byte)
      [1] = high byte of distance
      [2] = low byte of distance
      [3] = checksum
    */
    while (sensorSerial.available())
    {

        byte b = sensorSerial.read();

        lastPacketTime = millis(); // Update timestamp of last received data

        // Wait for start byte (0xFF) to synchronize packet
        if (indexBuffer == 0 && b != 0xFF)
            continue;

        buffer[indexBuffer++] = b; // Store byte in buffer

        // If full packet (4 bytes) is received
        if (indexBuffer == 4)
        {
            // Calculate checksum (sum of first 3 bytes, lower 8 bits)
            int sum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;

            // Validate packet integrity
            if (sum == buffer[3])
            {
                sensorDataValid = true;
                invalidPacketCount = 0;

                // Combine high and low bytes into 16-bit distance value then convert to centimeters
                float rawDistance = ((buffer[1] << 8) + buffer[2]) / 10.0;

                distance = applyMovingAverage(rawDistance); // Apply filtering to stabilize measurement
                updateDirtySensorMonitor(distance);

                currentZone = updateZone(distance, currentZone); // Update zone with hysteresis
                setAudioFromZone(currentZone);                   // Set buzzer behavior based on zone

                // Debug output to Serial Monitor
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.print(" cm -> Zone: ");
                Serial.println(currentZone);
                Serial.print("Dirty count: ");
                Serial.println(distanceJumpCount);
            }
            else
            {
                if (invalidPacketCount < 255)
                {
                    invalidPacketCount++;
                }

                Serial.print("Invalid packet count: ");
                Serial.println(invalidPacketCount);
            }

            // Reset buffer for next packet
            indexBuffer = 0;
        }
    }

    /*
      Periodic curb sensor measurement

      - Measurement is performed at a fixed interval
      - Invalid readings (-1) are ignored
      - Last valid value is preserved for stability
    */
    if (millis() - lastCurbReadTime >= CURB_READ_INTERVAL)
    {
        lastCurbReadTime = millis();

        float rawCurb = readCurbDistanceCm();

        if (rawCurb > 0)
        {
            lastValidCurbDistance = rawCurb;
        }

        curbDistance = lastValidCurbDistance;

        // Curb detection with hysteresis and minimum activation time
        if (curbDistance > 0)
        {
            if (!curbActive)
            {
                // Activate only if below ON threshold
                if (curbDistance < CURB_ON_THRESHOLD)
                {
                    curbActive = true;
                    curbTriggerTime = millis();
                }
            }
            else
            {
                // Stay active for minimum time
                if (millis() - curbTriggerTime < CURB_MIN_ACTIVE_TIME)
                {
                    // keep active
                }
                else
                {
                    // Deactivate only if above OFF threshold
                    if (curbDistance > CURB_OFF_THRESHOLD)
                    {
                        curbActive = false;
                    }
                }
            }
        }

        Serial.print("Curb distance: ");
        Serial.println(curbDistance);
    }

    // Timeout is currently used as the first fault condition.
    currentFault = detectFault(reverseActive);
    faultDetected = (currentFault != FAULT_NONE);

    // Update system state
    currentState = updateSystemState(reverseActive, faultDetected, currentState);

    if (currentState != previousState)
    {
        if (currentState == STATE_FAULT)
        {
            faultCycleStartTime = millis();
        }
        if (previousState == STATE_FAULT && currentState == STATE_ACTIVE)
        {
            resetFaultLogic();
        }

        previousState = currentState;
    }

    // Handle output according to current state
    switch (currentState)
    {
    case STATE_IDLE:
        stopAudioTone();
        resetSystemLogic();
        return;

    case STATE_FAULT:
        Serial.print("FAULT ACTIVE: ");
        Serial.println(currentFault);

        if (currentFault != FAULT_DIRTY_SENSOR)
        {
            indexBuffer = 0;
            audioInterval = AUDIO_OFF_INTERVAL;
            setAudioFromFault(currentFault);
            updateFaultAudio(currentFault);
        }
        else
        {
            stopAudioTone();
        }
        return;

    case STATE_ACTIVE:
        if (curbActive)
        {
            audioFrequency = 1800;
            audioInterval = 40;

            updateAudio();
            return;
        }
        updateAudio();
        return;
    }
}