#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>
#include <ArduinoJson-v6.18.5.h>
#include <DifferentialSteering.h>

const int MotorLFront = 7;
const int MotorLBack = 3;
const int MotorRFront = 2;
const int MotorRBack = 4;
const int MotorLeftPWM = 6;
const int MotorRightPWM = 5;

const int leftOpticalSensorPin = 9;
const int rightOpticalSensorPin = 10;

const int servoPin = 8;
Servo UltrasonicServoMotor;     // define UltrasonicServoMotor
const int degreesForward = 110; //nr degrees to look forward
const int degreesLeft = 30;     //nr degrees to look left
const int degreesRight = 180;   //nr degrees to look right
const int delay_time = 250;     // servo motor delay
const int UltrasonicReceivePin = 12;         // ultrasonic receive=echo pin
const int UltrasonicSendPin = 13;      // ultrasonic send=trigger pin

int mode = 0;

const int goStraight = 8;       // go straight
const int goRight = 6;          // turn right
const int goLeft = 4;           // turn left
const int goBack = 2;           // go back
int motorsSpeed = 255;          //speed, range 0 to 255

const int BT_STOP = 10;
const int BT_FWD = 1;
const int BT_BCK = 2;
const int BT_RIGHT = 3;
const int BT_LEFT = 4;
const int BT_MODE = 9;

void setup() {
    Serial.begin(115200);

    pinMode(MotorLFront, OUTPUT);
    pinMode(MotorLBack, OUTPUT);
    pinMode(MotorRFront, OUTPUT);
    pinMode(MotorRBack, OUTPUT);
    pinMode(MotorLeftPWM, OUTPUT);
    pinMode(MotorRightPWM, OUTPUT);

    pinMode(UltrasonicReceivePin, INPUT);
    pinMode(UltrasonicSendPin, OUTPUT);
    UltrasonicServoMotor.write(degreesForward);
    UltrasonicServoMotor.attach(servoPin);

    pinMode(leftOpticalSensorPin, INPUT);
    pinMode(rightOpticalSensorPin, INPUT);
}

void handleLeftSideMotors(bool forward, int velocity) {
    digitalWrite(MotorLFront, forward ? HIGH : LOW);
    digitalWrite(MotorLBack, forward ? LOW : HIGH);
    analogWrite(MotorLeftPWM, velocity);
}

void handleRightSideMotors(bool forward, int velocity) {
    digitalWrite(MotorRFront, forward ? HIGH : LOW);
    digitalWrite(MotorRBack, forward ? LOW : HIGH);
    analogWrite(MotorRightPWM, velocity);
}

void advance(int d) {
    digitalWrite(MotorLFront, HIGH);
    digitalWrite(MotorLBack, LOW);
    digitalWrite(MotorRFront, HIGH);
    digitalWrite(MotorRBack, LOW);
    analogWrite(MotorLeftPWM, motorsSpeed);
    analogWrite(MotorRightPWM, motorsSpeed);
    delay(d * 10);
}

void turnRight(int d) {
    digitalWrite(MotorLFront, HIGH);
    digitalWrite(MotorLBack, LOW);
    digitalWrite(MotorRFront, LOW);
    digitalWrite(MotorRBack, HIGH);
    analogWrite(MotorLeftPWM, motorsSpeed);
    analogWrite(MotorRightPWM, motorsSpeed);
    delay(d * 10);
}

void turnLeft(int d) {
    digitalWrite(MotorLFront, LOW);
    digitalWrite(MotorLBack, HIGH);
    digitalWrite(MotorRFront, HIGH);
    digitalWrite(MotorRBack, LOW);
    analogWrite(MotorLeftPWM, motorsSpeed);
    analogWrite(MotorRightPWM, motorsSpeed);
    delay(d * 10);
}

void stop(int d) {
    digitalWrite(MotorLFront, LOW);
    digitalWrite(MotorLBack, LOW);
    digitalWrite(MotorRFront, LOW);
    digitalWrite(MotorRBack, LOW);
    analogWrite(MotorLeftPWM, motorsSpeed);
    analogWrite(MotorRightPWM, motorsSpeed);
    delay(d * 10);
}

void back(int d) { //go back
    digitalWrite(MotorLFront, LOW);
    digitalWrite(MotorLBack, HIGH);
    digitalWrite(MotorRFront, LOW);
    digitalWrite(MotorRBack, HIGH);
    analogWrite(MotorLeftPWM, motorsSpeed);
    analogWrite(MotorRightPWM, motorsSpeed);
    delay(d * 10);
}

float getDistance(int degrees) {
    UltrasonicServoMotor.write(degrees);
    digitalWrite(UltrasonicSendPin, LOW);                       // ultrasonic echo low level in 2us
    delayMicroseconds(2);
    digitalWrite(UltrasonicSendPin, HIGH);                      // ultrasonic echo high level in 10us, at least 10us
    delayMicroseconds(10);
    digitalWrite(UltrasonicSendPin, LOW);                       // ultgrasonic echo low level
    float distance = pulseIn(UltrasonicReceivePin, HIGH);       // read time
    distance = distance * 0.034 / 2;                                    // turn time to distance
    return distance;
}

int chooseDirection() {
    int forwardDistance = 0;
    int rightSideDistance = 0;
    int leftSideDistance = 0;
    int direction = 0;

    forwardDistance = getDistance(degreesForward);

    if (forwardDistance == 0.00) {
        stop(1);
        direction = goBack;
        return direction;
    }
    if (forwardDistance < 10) {
        stop(1);
        direction = goBack;
    } else if (forwardDistance > 10 && forwardDistance < 25) {
        stop(1);
        leftSideDistance = getDistance(degreesLeft);
        delay(delay_time);

        rightSideDistance = getDistance(degreesRight);
        delay(delay_time);

        if (leftSideDistance == rightSideDistance) {
            direction = goBack;
        }
        if (leftSideDistance > rightSideDistance) {
            direction = goLeft;
        }

        if (leftSideDistance < rightSideDistance) {
            direction = goRight;
        }

        if (leftSideDistance < 15 && rightSideDistance < 15) {
            direction = goBack;
        }
    } else {
        direction = goStraight;
    }
    const int leftDistanceSensorValue = digitalRead(leftOpticalSensorPin);
    const int rightDistanceSensorValue = digitalRead(rightOpticalSensorPin);
    if (leftDistanceSensorValue == 0 && rightDistanceSensorValue == 0) {
        return goBack;
    }
    if (leftDistanceSensorValue == 0) {
        return goRight;
    }
    if (rightDistanceSensorValue == 0) {
        return goLeft;
    }
    return direction;
}

void autoRunUsingUltraSonic() {
    int directionn = 0;
    UltrasonicServoMotor.write(80);
    directionn = chooseDirection();

    if (directionn == goStraight) {
        advance(5);
    } else if (directionn == goBack) {
        back(8);
        turnLeft(60);
    } else if (directionn == goRight) {
        back(1);
        turnRight(30);
    } else if (directionn == goLeft) {
        back(1);
        turnLeft(30);
    }
}

void analog(int x, int y) {
    DifferentialDriver result = differentialSteering(x, 0 - y);
    handleLeftSideMotors(result.leftMotor > 0, map(abs(result.leftMotor), 0, 127, 0, 255));
    handleRightSideMotors(result.rightMotor > 0, map(abs(result.rightMotor), 0, 127, 0, 255));
}

void bluetoothControls(StaticJsonDocument<300> data) {
    int x = data["x"].as<int>();
    int y = data["y"].as<int>();
    int button = data["button"].as<int>();
    if (button != 0) {
        motorsSpeed = 255;
    }

    if (button == BT_MODE) {
        if (mode == 1) {
            mode = 0;
            Serial.println("manual");
            return;
        }
        if (mode == 0) {
            mode = 1;
            Serial.println("automatic");
            return;
        }
    }
    if (button == BT_STOP || ((x < 20 && x > -20) && (y < 20 && y > -20))) {
        stop(1);
    }
    if (button == BT_FWD) {
        advance(1);
    }
    if (button == BT_BCK) {
        back(1);
    }
    if (button == BT_LEFT) {
        turnLeft(1);
    }
    if (button == BT_RIGHT) {
        turnRight(1);
    }
    if (button == 0) {
        analog(x, y);
    }
}

void loop() {
    if (mode == 1) {
        autoRunUsingUltraSonic();
        UltrasonicServoMotor.write(degreesForward);
    }


    while (Serial.available()) {
        StaticJsonDocument<300> doc;
        DeserializationError err = deserializeJson(doc, Serial);

        if (err == DeserializationError::Ok) {
            Serial.println(doc.as<String>());
            bluetoothControls(doc);
        } else {
            Serial.print("deserializeJson() returned ");
            Serial.println(err.c_str());
            Serial.println(err.f_str());

            // Flush all bytes in the "link" serial port buffer
            while (Serial.available() > 0)
                Serial.read();
        }
    }
}