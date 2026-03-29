#include <SoftwareSerial.h>

/*
  Pin configuration
  RX_PIN: Arduino receives data from sensor (connect sensor TX here)
  TX_PIN: Not used in this project (sensor transmits automatically)
  BUZZER_PIN: Passive buzzer output pin
  REVERSE PIN: Simulates vehicle reverse gear activation using a push button
*/

#define RX_PIN 11
#define TX_PIN 10
#define BUZZER_PIN 3
#define REVERSE_PIN 2

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

// Current active zone
zone_t currentZone = ZONE_SAFE;

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

/*
  Timeout handling
  If no data is received from sensor within TIMEOUT, buzzer will be disabled for safety
*/
const unsigned long TIMEOUT = 200; // milliseconds
unsigned long lastPacketTime = 0;

// Moving average filter
#define FILTER_SIZE 5

float filterBuffer[FILTER_SIZE] = {0};
int filterIndex = 0;
bool filterFilled = false;

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
  Updates passive buzzer output using non-blocking timing

  Behavior:
  - outdated sensor data -> audio disabled
  - interval == 0        -> continuous tone
  - interval == OFF      -> no sound
  - otherwise            -> periodic beeping
*/
void updateAudio()
{
    if (millis() - lastPacketTime > TIMEOUT)
    {
        stopAudioTone();
        return;
    }

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

void setup()
{
    Serial.begin(115200);     // Serial communication for debugging (PC)
    sensorSerial.begin(9600); // Sensor UART communication (fixed at 9600 baud)

    pinMode(BUZZER_PIN, OUTPUT);        // Configure buzzer pin as output
    pinMode(REVERSE_PIN, INPUT_PULLUP); // Configure reverse input with internal pull-up resistor
}

void loop()
{
    // Read reverse state (active LOW)
    bool reverseActive = (digitalRead(REVERSE_PIN) == LOW);

    // If reverse is OFF, disable system
    if (!reverseActive)
    {
        stopAudioTone();
        currentZone = ZONE_SAFE;
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
                // Combine high and low bytes into 16-bit distance value then convert to centimeters
                float rawDistance = ((buffer[1] << 8) + buffer[2]) / 10.0;

                distance = applyMovingAverage(rawDistance);      // Apply filtering to stabilize measurement
                currentZone = updateZone(distance, currentZone); // Update zone with hysteresis
                setAudioFromZone(currentZone);                   // Set buzzer behavior based on zone

                // Debug output to Serial Monitor
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.print(" cm -> Zone: ");
                Serial.println(currentZone);
            }
            // Reset buffer for next packet
            indexBuffer = 0;
        }
    }
    // Update buzzer state continuously
    updateAudio();
}