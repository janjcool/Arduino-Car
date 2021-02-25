#include <Arduino.h>  // remove this line if you are using arduino

class Motor {
public:
    int pwmChannel;
    int dutyCycle = 200;
    int Pin1;
    int Pin2;

    void init(int MotorPin1, int MotorPin2, int enable1Pin, int freq,
              int MotorPWMChannel, int resolution, int MotorDutyCycle) {
        Serial.println("Initializing motor");
        pwmChannel = MotorPWMChannel;
        dutyCycle = MotorDutyCycle;
        Pin1 = MotorPin1;
        Pin2 = MotorPin2;

        // sets the pins as outputs:
        pinMode(Pin1, OUTPUT);
        pinMode(Pin2, OUTPUT);
        pinMode(enable1Pin, OUTPUT);

        // configure PWM
        ledcSetup(pwmChannel, freq, resolution);

        // attach the channel to the GPIO to be controlled
        ledcAttachPin(enable1Pin, pwmChannel);

        ledcWrite(pwmChannel, dutyCycle);
        Serial.print("Motor initialized at pwm channel ");
        Serial.println(pwmChannel);
    }

    void stop() const {
        Serial.print("Stop motor at pwm channel ");
        Serial.println(pwmChannel);

        digitalWrite(Pin1, LOW);
        digitalWrite(Pin2, LOW);
    }

    void backwards() const {
        Serial.print("Moving motor backwards at pwm channel ");
        Serial.println(pwmChannel);

        digitalWrite(Pin1, HIGH);
        digitalWrite(Pin2, LOW);
    }

    void forward() const {
        Serial.print("Moving motor forward at pwm channel ");
        Serial.println(pwmChannel);

        digitalWrite(Pin1, LOW);
        digitalWrite(Pin2, HIGH);
    }

    void update_dutyCycle() const {
        Serial.print("Forward with duty cycle ");
        Serial.print(dutyCycle);
        Serial.print(" at pwm channel ");
        Serial.println(pwmChannel);

        ledcWrite(pwmChannel, dutyCycle);
    }
};

Motor leftMotor;
Motor rightMotor;

void setup() {
    // begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // init motors
    leftMotor.init(27, 26, 14, 30000, 0, 8, 200);
    rightMotor.init(25, 33, 32, 30000, 1, 8, 200);

    Serial.println("End of setup");
}

void loop() {
    leftMotor.forward();
    rightMotor.forward();
    delay(2000);

    leftMotor.stop();
    rightMotor.stop();
    delay(1000);

    leftMotor.dutyCycle = 150;
    leftMotor.update_dutyCycle();
}