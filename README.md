#include <Bluepad32.h>

int beruhrungskabel = 5;
int leben = 3;
long letzteberuhrung = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
const int joystickThreshold = 50;
const int l2Max = 1020;
const int r2Max = 520;
const int r2Threshold = 10;
const int l2Threshold = 10;
const int maxMotorSpeed = 255;
const int leftJoystickMax = 520;

// Gleichstrommotor 1 (rechts)
int GSM1 = 27;
int in1 = 12;
int in2 = 14;
// Gleichstrommotor 2 (links)
int GSM2 = 33;
int in3 = 26;
int in4 = 25;

//Piezo Pieper
int beep = 4;

//Lebensanzeige - LEDs
int Led1 = 18;
int Led2 = 19;
int Led3 = 21;

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }
    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void moveMotors(bool linksVorwaerts, bool rechtsVorwaerts, int currentSpeedLeft, int currentSpeedRight) {
    // Set motor speeds
    analogWrite(GSM1, currentSpeedLeft);
    analogWrite(GSM2, currentSpeedRight);

    // Motor direction control
    if( leben > 0) {
      digitalWrite(in1, linksVorwaerts ? HIGH : LOW);
      digitalWrite(in2, linksVorwaerts ? LOW : HIGH);
      digitalWrite(in3, rechtsVorwaerts ? HIGH : LOW);
      digitalWrite(in4, rechtsVorwaerts ? LOW : HIGH);
      }
}


void processGamepad(ControllerPtr ctl) {


//== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
      //alle 3 leds anmachen ( der reihe nach am besten) 
      digitalWrite(Led1, HIGH);
      delay(1000);
      digitalWrite(Led2, HIGH);
      delay(1000);
      digitalWrite(Led3, HIGH);
      delay(1000);
      leben = 3;   
  }

    int aufteilungsgrenzelinks = (-511 + joystickThreshold) / 2;
    int aufteilungsgrenzerechts = (512 - joystickThreshold) / 2;

    auto calculateSpeed = [&](int brakeThrottle, int axisX, int threshold, int maxVal, bool reverse) {
        int speedLeft = maxMotorSpeed * ((double)brakeThrottle / maxVal);
        int speedRight = speedLeft;
        bool leftMotorReverse = reverse;
        bool rightMotorReverse = reverse;

        if (axisX <= -joystickThreshold) {
            if (axisX > aufteilungsgrenzelinks) {
                speedLeft *= (1 + ((double)axisX / leftJoystickMax));
            } else {
                leftMotorReverse = !reverse;
            }
        } else if (axisX >= joystickThreshold) {
            if (axisX < aufteilungsgrenzerechts) {
                speedRight *= (1 + ((double)axisX / leftJoystickMax));
            } else {
                rightMotorReverse = !reverse;
            }
        }

        return std::make_tuple(speedLeft, speedRight, leftMotorReverse, rightMotorReverse);
    };

    if (ctl->brake() >= r2Threshold) {
        auto [currentSpeedLeft, currentSpeedRight, leftMotor, rightMotor] = calculateSpeed(ctl->brake(), ctl->axisX(), joystickThreshold, r2Max, false);
        moveMotors(leftMotor, rightMotor, currentSpeedLeft, currentSpeedRight);
        Serial.println(currentSpeedLeft);
        Serial.println(currentSpeedRight);
    } else if (ctl->throttle() >= l2Threshold) {
        auto [currentSpeedLeft, currentSpeedRight, leftMotor, rightMotor] = calculateSpeed(ctl->throttle(), ctl->axisX(), joystickThreshold, l2Max, true);
        moveMotors(leftMotor, rightMotor, currentSpeedLeft, currentSpeedRight);
        Serial.println(currentSpeedLeft);
        Serial.println(currentSpeedRight);
    } else {
        Serial.println("Still stehen");
        moveMotors(true, true, 0, 0);
    }
}


void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected() && myController->hasData()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void setup() {
    
    pinMode(Led1, OUTPUT);
    pinMode(Led2, OUTPUT);
    pinMode(Led3, OUTPUT);

    digitalWrite(Led1, HIGH);
    digitalWrite(Led2, HIGH);
    digitalWrite(Led3, HIGH);

    pinMode(beruhrungskabel, INPUT);

    Serial.begin(115200);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
    pinMode(GSM1, OUTPUT);
    pinMode(GSM2, OUTPUT);
    pinMode(beep, OUTPUT);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
}

void loop() {
    Serial.printf("BeruehrungsKabel: %d \n", digitalRead(beruhrungskabel));
    bool istfahrzeuggetroffen = digitalRead(beruhrungskabel) == 1 && millis() - letzteberuhrung > 3000;
    static bool doppelPiepen = false;
    if (istfahrzeuggetroffen) {
        letzteberuhrung = millis();
        leben -= 1;
        Serial.printf("Leben: %d \n", leben);

        doppelPiepen = true;
        digitalWrite(beep, HIGH);

      
      switch (leben) {
    case 2:
        // led3 ausmachen
        digitalWrite(Led3, LOW);
        break;
    case 1:
        // led2 ausmachen
        digitalWrite(Led2, LOW);
        break;
    case 0:
        // led1 ausmachen
        digitalWrite(Led1, LOW);
        break;
    }

    }

    if (millis() > letzteberuhrung+100) {
      digitalWrite(beep, LOW);
    }
    if (millis() > letzteberuhrung+3000 && doppelPiepen) {
      digitalWrite(beep, HIGH);
      delay(100);
      digitalWrite(beep,LOW);
      delay(100);
      digitalWrite(beep, HIGH);
      delay(100);
      digitalWrite(beep,LOW);
      doppelPiepen = false;
    }
    

    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        Serial.println("dataUpdated wird true weil das hier angezeigt wird");
        processControllers();
    }
    delay(150);
}
