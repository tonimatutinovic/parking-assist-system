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
bool toneOn = false;              // Current buzzer state

/*
  Timeout handling
  If no data is received from sensor within TIMEOUT, buzzer will be disabled for safety
*/
const unsigned long TIMEOUT = 200; // milliseconds
unsigned long lastPacketTime = 0;

/*
  Maps distance (cm) to buzzer behavior
  Smaller distance -> faster beeping
*/
void setBuzzerInterval(float d)
{
    if (d < 20)
        buzzerInterval = 0; // Continuous tone (very close)
    else if (d < 40)
        buzzerInterval = 80; // Fast beeping
    else if (d < 60)
        buzzerInterval = 200; // Medium speed
    else if (d < 100)
        buzzerInterval = 500; // Slow beeping
    else
        buzzerInterval = 999999; // No sound (safe distance)
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
        noTone(BUZZER_PIN);
        return;
    }

    // Continuous tone for critical distance
    if (buzzerInterval == 0)
    {
        tone(BUZZER_PIN, 2000); // 2 kHz tone
        return;
    }

    // No sound zone
    if (buzzerInterval >= 999999)
    {
        noTone(BUZZER_PIN);
        return;
    }

    // Toggle buzzer ON/OFF based on interval
    if (millis() - lastToggle >= buzzerInterval)
    {
        lastToggle = millis();
        toneOn = !toneOn;

        if (toneOn)
        {
            tone(BUZZER_PIN, 2000); // Turn sound ON
        }
        else
        {
            noTone(BUZZER_PIN); // Turn sound OFF
        }
    }
}

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
    // Store new measurement
    filterBuffer[filterIndex] = newValue;

    // Move index forward using modulo
    filterIndex = (filterIndex + 1) % FILTER_SIZE;

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

void setup()
{
    // Serial communication for debugging (PC)
    Serial.begin(115200);

    // Sensor UART communication (fixed at 9600 baud)
    sensorSerial.begin(9600);

    // Configure buzzer pin as output
    pinMode(BUZZER_PIN, OUTPUT);
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

        // Update timestamp of last received data
        lastPacketTime = millis();

        // Wait for start byte (0xFF) to synchronize packet
        if (indexBuffer == 0 && b != 0xFF)
            continue;

        // Store byte in buffer
        buffer[indexBuffer++] = b;

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

                // Apply filtering to stabilize measurement
                distance = applyMovingAverage(rawDistance);

                // Debug output to Serial Monitor
                Serial.print("Distance: ");
                Serial.print(distance);
                Serial.println(" cm");

                // Update zone with hysteresis
                currentZone = updateZone(distance, currentZone);

                // Set buzzer behavior based on zone
                switch (currentZone)
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

            // Reset buffer for next packet
            indexBuffer = 0;
        }
    }

    // Update buzzer state continuously
    updateBuzzer();
}