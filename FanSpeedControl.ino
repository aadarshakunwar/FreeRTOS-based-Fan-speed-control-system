/*
 * Motor Speed Control using FreeRTOS
 */

#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <PID_v1.h>
#include <LiquidCrystal.h>

// Motor Pin Definitions
#define PIN_PWMA    9
#define PIN_AIN1    7
#define PIN_AIN2    8
#define PIN_STBY    6
#define PIN_IR      2

// LCD Pin Definitions (16x2 Parallel LCD)
#define LCD_RS      12
#define LCD_EN      11
#define LCD_D4      5
#define LCD_D5      4
#define LCD_D6      3
#define LCD_D7      10

// Configuration - Adjust these values as needed
#define SLOTS_PER_REVOLUTION  15
#define MIN_PWM               15
#define MAX_RPM               10000

// PID Tuning
double Kp = 0.5;
double Ki = 0.3;

double Kd = 0.05;

// Shared Variables
volatile unsigned long pulseCount = 0;
double measuredRPM = 0;
double desiredRPM = 0;
double pwmOutput = 0;
double smoothedRPM = 0;

SemaphoreHandle_t xDataMutex;
PID motorPID(&measuredRPM, &pwmOutput, &desiredRPM, Kp, Ki, Kd, DIRECT);

// LCD Object
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Task Declarations
void TaskSpeedMeasure(void *pvParameters);
void TaskSpeedControl(void *pvParameters);
void TaskUserInput(void *pvParameters);
void TaskDisplay(void *pvParameters);

void countPulse() {
    pulseCount++;
}

void setup() {
    Serial.begin(9600);
    while (!Serial) { ; }

    pinMode(PIN_PWMA, OUTPUT);
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);

    digitalWrite(PIN_STBY, HIGH);
    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);

    pinMode(PIN_IR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_IR), countPulse, FALLING);

    motorPID.SetMode(AUTOMATIC);
    motorPID.SetOutputLimits(0, 255);
    motorPID.SetSampleTime(100);

    xDataMutex = xSemaphoreCreateMutex();

    // Initialize LCD
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor Control");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");

    Serial.println(F("================================"));
    Serial.println(F("  Motor Speed Control - FreeRTOS"));
    Serial.println(F("================================"));
    Serial.println(F("Enter desired RPM (0-1000):"));
    Serial.println();

    xTaskCreate(TaskSpeedMeasure, "Measure",  128, NULL, 2, NULL);
    xTaskCreate(TaskSpeedControl, "Control",  128, NULL, 3, NULL);
    xTaskCreate(TaskUserInput,    "Input",    128, NULL, 1, NULL);
    xTaskCreate(TaskDisplay,      "Display",  192, NULL, 1, NULL);
}

void loop() {}

// Measures RPM from IR sensor with smoothing filter
void TaskSpeedMeasure(void *pvParameters) {
    (void) pvParameters;

    unsigned long lastPulseCount = 0;
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t measureInterval = pdMS_TO_TICKS(100);
    const double filterAlpha = 0.3;

    for (;;) {
        noInterrupts();
        unsigned long currentPulses = pulseCount;
        interrupts();

        unsigned long deltaPulses = currentPulses - lastPulseCount;
        lastPulseCount = currentPulses;

        double rawRPM = (deltaPulses * 60000.0) / (SLOTS_PER_REVOLUTION * 100.0);
        smoothedRPM = (filterAlpha * rawRPM) + ((1.0 - filterAlpha) * smoothedRPM);

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            measuredRPM = smoothedRPM;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&lastWakeTime, measureInterval);
    }
}

// PID control with rate limiting for smooth motor response
void TaskSpeedControl(void *pvParameters) {
    (void) pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t controlInterval = pdMS_TO_TICKS(50);
    double lastPWM = 0;

    for (;;) {
        // SAFETY: Prevent motor spin if sensor has not detected pulses yet
        if (pulseCount < 3) {
            analogWrite(PIN_PWMA, 0);   // Force motor off

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            pwmOutput = 0;          // Report PWM as 0
            measuredRPM = 0;        // Report speed as 0
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&lastWakeTime, controlInterval);
        continue;  // Skip the PID until pulses start coming
        }

        double currentRPM, targetRPM;
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            currentRPM = measuredRPM;
            targetRPM = desiredRPM;
            xSemaphoreGive(xDataMutex);
        }

        measuredRPM = currentRPM;
        desiredRPM = targetRPM;
        motorPID.Compute();

        double finalPWM;
        if (targetRPM == 0) {
            finalPWM = 0;
        } else if (pwmOutput < MIN_PWM) {
            finalPWM = MIN_PWM;
        } else {
            finalPWM = pwmOutput;
        }

        // Rate limit PWM changes
        double maxChange = 5.0;
        if (finalPWM > lastPWM + maxChange) {
            finalPWM = lastPWM + maxChange;
        } else if (finalPWM < lastPWM - maxChange) {
            finalPWM = lastPWM - maxChange;
        }
        lastPWM = finalPWM;

        analogWrite(PIN_PWMA, (int)finalPWM);

        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            pwmOutput = finalPWM;
            xSemaphoreGive(xDataMutex);
        }

        vTaskDelayUntil(&lastWakeTime, controlInterval);
    }
}

// Reads desired speed from Serial input
void TaskUserInput(void *pvParameters) {
    (void) pvParameters;

    for (;;) {
        if (Serial.available() > 0) {
            double newSpeed = Serial.parseFloat();

            while (Serial.available()) {
                Serial.read();
            }

            if (newSpeed >= 0 && newSpeed <= MAX_RPM) {
                if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
                    desiredRPM = newSpeed;
                    xSemaphoreGive(xDataMutex);
                }
                Serial.print(F(">> New target: "));
                Serial.print(newSpeed, 0);
                Serial.println(F(" RPM"));
            } else {
                Serial.println(F(">> Invalid! Enter 0-1000"));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Displays current speed status on LCD and Serial
void TaskDisplay(void *pvParameters) {
    (void) pvParameters;

    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t displayInterval = pdMS_TO_TICKS(500);

    for (;;) {
        double currentRPM, targetRPM, currentPWM;
        if (xSemaphoreTake(xDataMutex, portMAX_DELAY) == pdTRUE) {
            currentRPM = measuredRPM;
            targetRPM = desiredRPM;
            currentPWM = pwmOutput;
            xSemaphoreGive(xDataMutex);
        }

        // Update LCD display
        // Line 1: Desired speed
        lcd.setCursor(0, 0);
        lcd.print("Desired:  ");
        lcd.print((int)targetRPM);
        lcd.print(" RPM    ");  // Extra spaces to clear old digits

        // Line 2: Measured speed
        lcd.setCursor(0, 1);
        lcd.print("Measured: ");
        lcd.print((int)currentRPM);
        lcd.print(" RPM    ");  // Extra spaces to clear old digits

        // Also output to Serial for debugging
        Serial.print(F("Desired: "));
        Serial.print(targetRPM, 0);
        Serial.print(F(" RPM | Measured: "));
        Serial.print(currentRPM, 0);
        Serial.print(F(" RPM | PWM: "));
        Serial.println(currentPWM, 0);

        vTaskDelayUntil(&lastWakeTime, displayInterval);
    }
}