#include <Arduino.h>
#include <BluetoothSerial.h>

BluetoothSerial bt;
int ledState = 0;
unsigned long previousMillis = 0;
bool isConnected = false;

const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;

const int MOTORS_LEFT_IN1 = 16;
const int MOTORS_LEFT_IN2 = 17;
const int MOTORS_LEFT_EN = 4;

const int MOTORS_RIGHT_IN1 = 5;
const int MOTORS_RIGHT_IN2 = 18;
const int MOTORS_RIGHT_EN = 15;

double steerAngle = 0;
double speed = 0;

void setAngle(int angle)
{
    steerAngle = (angle - 127) / 127.0;
}

void setSpeed(int speedVal)
{
    speed = (speedVal - 127) / 127.0;
}

const int DRIVE_CONTROL_SIZE = 2;
uint8_t *driveControl;

void setupMotorDrivers();
void runMotors();

void setup()
{
    Serial.begin(115200);
    bt.begin("ESP32-Bluetooth");

    pinMode(BUILTIN_LED, OUTPUT);
    digitalWrite(BUILTIN_LED, LOW);

    setupMotorDrivers();

    Serial.println("ESP32 Bluetooth Control");

    Serial.println("Bluetooth Device is Ready to Pair");
    previousMillis = millis();

    driveControl = (uint8_t *)malloc(DRIVE_CONTROL_SIZE * sizeof(uint8_t));
    if (driveControl == NULL)
    {
        Serial.println("Memory allocation failed");
        return;
    }
    driveControl[0] = 127;
    driveControl[1] = 127;
}

void loop()
{
    if (!bt.connected())
    {
        if (isConnected)
        {
            isConnected = false;
            digitalWrite(BUILTIN_LED, LOW);
        }
        unsigned long current = millis();
        if (current - previousMillis >= 1000)
        {
            previousMillis = current;
            ledState = !ledState;
            digitalWrite(BUILTIN_LED, ledState);
        }
    }
    if (bt.available())
    {
        if (!isConnected)
        {
            isConnected = true;
            digitalWrite(BUILTIN_LED, HIGH);
            Serial.println("Bluetooth Connected");
        }
        bt.readBytes(driveControl, DRIVE_CONTROL_SIZE);

        Serial.print("Drive Control: ");
        Serial.print(driveControl[0]);
        Serial.print(", ");
        Serial.println(driveControl[1]);

        int speedRaw = driveControl[0];
        int steerRaw = driveControl[1];

        setSpeed(speedRaw);
        setAngle(steerRaw);

        Serial.print("Speed: ");
        Serial.print(speed);
        Serial.print(", Steer Angle: ");
        Serial.println(steerAngle);

        runMotors();
    }
}

void setupMotorDrivers()
{
    ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
    ledcSetup(pwmChannel2, pwmFreq, pwmResolution);

    ledcAttachPin(MOTORS_LEFT_EN, pwmChannel1);
    ledcAttachPin(MOTORS_RIGHT_EN, pwmChannel2);

    pinMode(MOTORS_LEFT_IN1, OUTPUT);
    pinMode(MOTORS_LEFT_IN2, OUTPUT);
    pinMode(MOTORS_RIGHT_IN1, OUTPUT);
    pinMode(MOTORS_RIGHT_IN2, OUTPUT);
}

void runMotors()
{
    double leftMotorSpeed = speed;
    double rightMotorSpeed = speed;

    if (steerAngle > 0)
    {
        rightMotorSpeed = rightMotorSpeed * (1.0 - abs(steerAngle));
    }
    else if (steerAngle < 0)
    {
        leftMotorSpeed = leftMotorSpeed * (1.0 - abs(steerAngle));
    }

    if (leftMotorSpeed > 0)
    {
        digitalWrite(MOTORS_LEFT_IN1, HIGH);
        digitalWrite(MOTORS_LEFT_IN2, LOW);
    }
    else if (leftMotorSpeed < 0)
    {
        digitalWrite(MOTORS_LEFT_IN1, LOW);
        digitalWrite(MOTORS_LEFT_IN2, HIGH);
    }
    else
    {
        digitalWrite(MOTORS_LEFT_IN1, LOW);
        digitalWrite(MOTORS_LEFT_IN2, LOW);
    }

    if (rightMotorSpeed > 0)
    {
        digitalWrite(MOTORS_RIGHT_IN1, HIGH);
        digitalWrite(MOTORS_RIGHT_IN2, LOW);
    }
    else if (rightMotorSpeed < 0)
    {
        digitalWrite(MOTORS_RIGHT_IN1, LOW);
        digitalWrite(MOTORS_RIGHT_IN2, HIGH);
    }
    else
    {
        digitalWrite(MOTORS_RIGHT_IN1, LOW);
        digitalWrite(MOTORS_RIGHT_IN2, LOW);
    }

    int leftPWM = min(255, (int)(abs(leftMotorSpeed) * 255));
    int rightPWM = min(255, (int)(abs(rightMotorSpeed) * 255));

    ledcWrite(pwmChannel1, leftPWM);
    ledcWrite(pwmChannel2, rightPWM);

    Serial.print("Left Motor PWM: ");
    Serial.print(leftPWM);
    Serial.print(", Right Motor PWM: ");
    Serial.println(rightPWM);
}