#include <Arduino.h>  // remove this line if you are using arduino

#include "SSD1306Wire.h"  // library for SSD1306

class Motor {
public:
    int pwm_channel{};
    int duty_cycle = 200;
    int pin_1{};
    int pin_2{};

    void Init(int MotorPin1, int MotorPin2, int enable1Pin, int freq,
              int MotorPWMChannel, int resolution, int MotorDutyCycle) {
        Serial.println("Initializing motor");
        pwm_channel = MotorPWMChannel;
        duty_cycle = MotorDutyCycle;
        pin_1 = MotorPin1;
        pin_2 = MotorPin2;

        // sets the pins as outputs:
        pinMode(pin_1, OUTPUT);
        pinMode(pin_2, OUTPUT);
        pinMode(enable1Pin, OUTPUT);

        // configure PWM
        ledcSetup(pwm_channel, freq, resolution);

        // attach the channel to the GPIO to be controlled
        ledcAttachPin(enable1Pin, pwm_channel);

        ledcWrite(pwm_channel, duty_cycle);
        Serial.print("Motor initialized at pwm channel ");
        Serial.println(pwm_channel);
    }

    void Stop() const {
        Serial.print("Stop motor at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
    }

    void Backwards() const {
        Serial.print("Moving motor Backwards at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, HIGH);
        digitalWrite(pin_2, LOW);
    }

    void Forward() const {
        Serial.print("Moving motor Forward at pwm channel ");
        Serial.println(pwm_channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
    }

    void UpdateDutyCycle() const {
        Serial.print("Update duty cycle: duty cycle \"");
        Serial.print(duty_cycle);
        Serial.print("\" at pwm channel \"");
        Serial.print(pwm_channel);
        Serial.println("\"");

        ledcWrite(pwm_channel, duty_cycle);
    }
};

class LED {
public:
    int pin;
    int pwm_channel;
    int pwm_frequency = 500;
    int duty_cycle = 0;

    const int PWM_RESOLUTION = 8;

    void Init(int LedPin, int LedPWMChannel) {
        // Save parameters in public variables
        pwm_channel = LedPWMChannel;
        pin = LedPin;

        // Sets the pin as output
        pinMode(pin, OUTPUT);

        // Configure PWM
        ledcSetup(pwm_channel, pwm_frequency, PWM_RESOLUTION);

        // Attach the channel to the GPIO to be controlled
        ledcAttachPin(pin, pwm_channel);
        UpdateDutyCycle();

        Serial.print("Led initialized at pwm channel ");
        Serial.println(pwm_channel);
    }

    void Off() {
        duty_cycle = 0;
        UpdateDutyCycle();
    }

    void On() {
        duty_cycle = 255;
        UpdateDutyCycle();
    }

    void FadeOn(int step_time) {
        Serial.println("Start of led fading on");
        duty_cycle = 0;

        for( ; duty_cycle <= 255; duty_cycle++){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void FadeOff(int step_time) {
        Serial.println("Start of led fading off");
        duty_cycle = 255;

        for( ; duty_cycle >= 0; duty_cycle--){
            UpdateDutyCycle();
            delay(step_time);
        }
    }

    void UpdateDutyCycle() const {
        Serial.print("Update duty cycle: duty cycle \"");
        Serial.print(duty_cycle);
        Serial.print("\" at pwm channel \"");
        Serial.print(pwm_channel);
        Serial.println("\"");

        ledcWrite(pwm_channel, duty_cycle);
    }
};

// INIT AND SET VARIABLES
// Init and set black and white sensor objects
int IR_sensor_pin = 7;  // Pin of the black and white sensor
int circumference = 33; // diameter of the sensor location from center of wheel in millimeters

// Init display
SSD1306Wire display(0x3c, 5, 17);

// INIT VARIABLES
// Init motor objects
Motor left_motor;
Motor right_motor;

// Init LED objects
LED red_led1;
LED red_led2;

// Init black and white sensor objects
int previous_output_of_IR_sensor;  // output of sensor from previous loop
long sensor_output_time;  // moment in time when the sensor changes output
float car_speed;  // cm per milliseconds traveling calculated by radius of wheel and RPS

void read_car_speed () {
    if (digitalRead(IR_sensor_pin) != previous_output_of_IR_sensor) {
        previous_output_of_IR_sensor = digitalRead(IR_sensor_pin);
        try {
            car_speed = circumference/(((millis()-sensor_output_time)*2));  // calculating the speed in cm/milliseconds
            Serial.println("Speed updated current speed is " + String(car_speed));
        } catch (...) {
            Serial.println("Failed updating speed");
        }
        sensor_output_time = millis();
    }
}

void setup() {
    // Begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // Init LED's
    red_led1.Init(4, 2);
    red_led1.On();
    red_led2.Init(16, 3);
    red_led2.On();

    // Init display
    display.init();
    display.clear();
    display.flipScreenVertically();
    display.display();

    // Draw on Display
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Setup");
    display.display();

    // Init sensor and sensor objects
    pinMode(IR_sensor_pin, INPUT);
    previous_output_of_IR_sensor = digitalRead(IR_sensor_pin);
    sensor_output_time = millis();

    // Init motors
    left_motor.Init(27, 26, 14, 30000, 0, 8, 200);
    right_motor.Init(25, 33, 32, 30000, 1, 8, 200);

    // Debug info
    Serial.print("Setup() running on core ");
    Serial.println(xPortGetCoreID());
    Serial.println("End of setup");
}

void loop() {
    read_car_speed();
}