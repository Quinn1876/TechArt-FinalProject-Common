#ifndef MIRROR_SERVO_HPP
#define MIRROR_SERVO_HPP
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>

int operator ""_s(long double seconds) {
  return seconds*1000;
}

int operator ""_ms(long double milli_seconds) {
  return milli_seconds;
}

namespace TechArt {

  struct Lazer {
    int pinNumber_;
    Lazer(int pinNumber): pinNumber_(pinNumber) {}

    void initialize() {
      pinMode(pinNumber_, OUTPUT);
      digitalWrite(pinNumber_, LOW);
    }

    void setIntensity(int intensity) const {
      digitalWrite(pinNumber_, intensity > 0 ? HIGH : LOW);
    } 
  };

  class MirrorServo {
      Servo servo_;
      int centerPoint_;
      int pinNumber_;

    public:
      MirrorServo(int pinNumber, int centerPoint)
      : servo_(), centerPoint_(centerPoint), pinNumber_(pinNumber) {
      }

      void initialize() {
          servo_.attach(pinNumber_);
      }

      void setAngle(int angle) {
          servo_.write(centerPoint_ + angle);
      }
  };

  struct Pattern {
    int angles_[10];
    int lightLevels_[5];
    int patternPostDelay_;

    Pattern(int patternPostDelay, int angles[10], int lightLevels[5])
    {
      patternPostDelay_ = patternPostDelay;
      for (int i = 0; i < 10; i++) {
        angles_[i] = angles[i];
      }
      for (int i = 0; i < 5; i++) {
        lightLevels_[i] = lightLevels[i];
      }
    }

    void applyPatternLazers(Lazer lazers[5]) {
      for (int i = 0; i < 5; i++) {
        lazers[i].setIntensity(lightLevels_[i]);
      }
    }

    void applyPatternServos(MirrorServo servos[10] ) {
      for (int i = 0; i < 10; i++) {
        servos[i].setAngle(angles_[i]);
      }
    }

    void applyPatternDelay() {
      delay(patternPostDelay_);
    }
  };

};

namespace I2C {
  static constexpr char PERIPHERAL_ID = 1;
  static constexpr char DOOR_ID = 2;

  static constexpr char OK = 0;
  static constexpr char DATA_TOO_LONG = 1;
  static constexpr char NACK_ON_ADR = 2;
  static constexpr char NACK_ON_MSG = 3;
  static constexpr char UNKNOWN_ERR = 4;
  static constexpr char TIMEOUT = 5;

  enum DoorStateCommand : uint8_t {
    OPEN = 0,
    CLOSE
  };
  
  class Controller {
    void handleResult(const char& result) {
      switch (result) {
        case DATA_TOO_LONG:
          Serial.println("CONTROLLER ERROR: DATA TOO LONG");
          break;
        case NACK_ON_ADR:
          Serial.println("CONTROLLER ERROR: NACK ON Address");
          break;
        case NACK_ON_MSG:
          Serial.println("CONTROLLER ERROR: NACK ON Message");
          break;
        case UNKNOWN_ERR:
          Serial.println("CONTROLLER ERROR: Unknown Error");
          break;
        case TIMEOUT:
          Serial.println("CONTROLLER ERROR: TIMEOUT");
          break;
        
      }      
    }

    void sendByte(const char& id, const char& data) {
      Wire.beginTransmission(id);
      Wire.write(data);
      const char result = Wire.endTransmission();
      handleResult(result);
    }
  public:
    Controller() {
    }

    void initialize() {
      Wire.begin();
    }

    void sendPattern(uint8_t patternId) {
      sendByte(PERIPHERAL_ID, patternId);
    }

    void openDoor() {
      sendByte(DOOR_ID, 0);
    }

  };

  template <uint8_t GenericPeripheralId>
  class GenericPeripheral {
  public:
    GenericPeripheral(void (*handler)(int numBytes)) {
      Wire.begin(GenericPeripheralId);
      Wire.onReceive(handler);
    }
  };

  typedef GenericPeripheral<PERIPHERAL_ID> Peripheral;
  typedef GenericPeripheral<DOOR_ID> Door;

};

namespace Hardware {
  class Motor {
    public:
      enum Direction {
          Forward,
          Reverse,
          Stop
      };
    private:
      const int enablePin;
      const int forwardPin;
      const int reversePin;

      int currentSpeed;
      Direction currentDirection;
    public:
      Motor(int enable, int forward, int reverse, bool invert=false) 
        : enablePin(enable),
          forwardPin( invert ? reverse : forward),
          reversePin( invert ? forward : reverse) 
        {}

      void Setup() {
          pinMode(enablePin, OUTPUT);
          pinMode(forwardPin, OUTPUT);
          pinMode(reversePin, OUTPUT);
      }

      void SetDirection(Direction dir) {
          digitalWrite(forwardPin, LOW);
          digitalWrite(reversePin, LOW);
          switch (dir) {
          case Forward:
              digitalWrite(forwardPin, HIGH);
              break;
          case Reverse:
              digitalWrite(reversePin, HIGH);
              break;
          case Stop: break;
          }
      }

      void SetSpeed(int speed) {
          if (speed > 100)
              speed = 100;
          else if (speed < 0)
              speed = 0;

          int writeVal = map(speed, 0, 100, 0, 255);
          analogWrite(enablePin, writeVal);
          currentSpeed = speed;
      }

      int GetSpeed() const { return currentSpeed; }
      Direction GetDirection() const { return currentDirection; }
  };

  class ProximitySensor {
    const int pinNumber;
    const int threshold;

    public:
      ProximitySensor(int analogPinNumber, int threshold)
        : pinNumber(analogPinNumber),
          threshold(threshold){}

      void initialize() {
        pinMode(pinNumber, INPUT);

        ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bits
        // Set the Prescaller to 32. Default is 128 which gives an unusable sampling frequency
        // of 104us. With a prescaller of 32, we can get a sampling frequency of 26us 
        // which gives more then enough time to sample for the metal strip.
        ADCSRA |= bit (ADPS0) | bit (ADPS2);                 //  32
      }

      bool read() {
        int val = analogRead(pinNumber);
        return val > threshold;
      }
  };

  class Button {
    const int pinNumber;

    public:
      Button(int punNumber)
      : pinNumber(pinNumber) 
      {}

      void initialize() {
          pinMode(pinNumber, INPUT_PULLUP);
      }

      bool read() {
        int val = digitalRead(pinNumber);
        return val == LOW; // Digital pullup means that the button is pressed when val is low;
      }

      void registerOnPressed(void (*onPressedHandler)()) {
        attachInterrupt(digitalPinToInterrupt(pinNumber), onPressedHandler, FALLING);
      }
  };

  class Relay {
    int pinNumber;
    int state;

    public:
      Relay(int pin) 
        : pinNumber(pin) {}

      void initialize() {
        pinMode(pinNumber, OUTPUT);
      }

      void on() {
        digitalWrite(pinNumber, HIGH);
        state = HIGH;
      }

      void off() {
        digitalWrite(pinNumber, LOW);
        state = LOW;
      }

      void toggle() {
        state = state == HIGH ? LOW : HIGH;
        digitalWrite(pinNumber, state);
      }
  };
};
#endif
