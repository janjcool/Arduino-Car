#include <Arduino.h>  // remove this line if you are using arduino

class Motor {
public:
    int pwm_Channel{};
    int duty_cycle = 200;
    int pin_1{};
    int pin_2{};

    void Init(int MotorPin1, int MotorPin2, int enable1Pin, int freq,
              int MotorPWMChannel, int resolution, int MotorDutyCycle) {
        Serial.println("Initializing motor");
        pwm_Channel = MotorPWMChannel;
        duty_cycle = MotorDutyCycle;
        pin_1 = MotorPin1;
        pin_2 = MotorPin2;

        // sets the pins as outputs:
        pinMode(pin_1, OUTPUT);
        pinMode(pin_2, OUTPUT);
        pinMode(enable1Pin, OUTPUT);

        // configure PWM
        ledcSetup(pwm_Channel, freq, resolution);

        // attach the channel to the GPIO to be controlled
        ledcAttachPin(enable1Pin, pwm_Channel);

        ledcWrite(pwm_Channel, duty_cycle);
        Serial.print("Motor initialized at pwm channel ");
        Serial.println(pwm_Channel);
    }

    void Stop() const {
        Serial.print("Stop motor at pwm channel ");
        Serial.println(pwm_Channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, LOW);
    }

    void Backwards() const {
        Serial.print("Moving motor Backwards at pwm channel ");
        Serial.println(pwm_Channel);

        digitalWrite(pin_1, HIGH);
        digitalWrite(pin_2, LOW);
    }

    void Forward() const {
        Serial.print("Moving motor Forward at pwm channel ");
        Serial.println(pwm_Channel);

        digitalWrite(pin_1, LOW);
        digitalWrite(pin_2, HIGH);
    }

    void UpdateDutyCycle() const {
        Serial.print("Forward with duty cycle ");
        Serial.print(duty_cycle);
        Serial.print(" at pwm channel ");
        Serial.println(pwm_Channel);

        ledcWrite(pwm_Channel, duty_cycle);
    }
};

Motor left_motor;
Motor right_motor;

void setup() {
    // begin serial connect
    Serial.begin(115200);
    Serial.println("Begin of setup");

    // Init motors
    left_motor.Init(27, 26, 14, 30000, 0, 8, 200);
    right_motor.Init(25, 33, 32, 30000, 1, 8, 200);

    Serial.println("End of setup");
}

void loop() {
    left_motor.Forward();
    right_motor.Forward();
    delay(2000);

    left_motor.Stop();
    right_motor.Stop();
    delay(1000);

    left_motor.duty_cycle = 150;
    left_motor.UpdateDutyCycle();
}