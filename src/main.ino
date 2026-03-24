#include <SoftwareSerial.h>

/*
  Pin configuration
  RX_PIN: Arduino receives data from sensor (connect sensor TX here)
  TX_PIN: Not used in this project (sensor transmits automatically)
  BUZZER_PIN: Output pin for buzzer
*/
#define RX_PIN 11
#define TX_PIN 10
#define BUZZER_PIN 3

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

// Buzzer control variables (non-blocking timing using millis)
unsigned long lastToggle = 0;     // Last time buzzer state changed
unsigned long buzzerInterval = 0; // Interval between ON/OFF toggles
bool buzzerState = false;         // Current buzzer state

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
  Parameters: newValue: latest raw distance measurement (cm)
  Returns: filtered distance value (cm)
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
    d: distance (cm),
    current: current active zone
  Returns: updated zone
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
  Sets buzzer interval based on current warning zone
*/
void setBuzzerFromZone(zone_t zone)
{
    switch (zone)
    {
    case ZONE_SAFE:
        buzzerInterval = 999999;
        break;
    case ZONE_SLOW:
        buzzerInterval = 500;
        break;
    case ZONE_MEDIUM:
        buzzerInterval = 200;
        break;
    case ZONE_FAST:
        buzzerInterval = 80;
        break;
    case ZONE_CRITICAL:
        buzzerInterval = 0;
        break;
    }
}

/*
  Updates buzzer state using non-blocking timing
  Uses millis() instead of delay() to keep system responsive
*/
void updateBuzzer()
{
    // Disable buzzer if sensor data is outdated
    if (millis() - lastPacketTime > TIMEOUT)
    {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        return;
    }

    // Continuous tone for critical distance
    if (buzzerInterval == 0)
    {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        return;
    }

    // No sound zone
    if (buzzerInterval >= 999999)
    {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        return;
    }

    // Toggle buzzer ON/OFF based on interval
    if (millis() - lastToggle >= buzzerInterval)
    {
        lastToggle = millis();
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
}

void setup()
{
    Serial.begin(115200);        // Serial communication for debugging (PC)
    sensorSerial.begin(9600);    // Sensor UART communication (fixed at 9600 baud)
    pinMode(BUZZER_PIN, OUTPUT); // Configure buzzer pin as output
}

void loop()
{
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
                setBuzzerFromZone(currentZone);                  // Set buzzer behavior based on zone

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
    updateBuzzer();
}