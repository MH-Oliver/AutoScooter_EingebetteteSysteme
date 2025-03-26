#include <Bluepad32.h>

const int beruhrungskabel = 5;
int leben = 3;
long letzteberuhrung = 0;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

/*
const int joystickThreshold = 50;
const int l2Max = 1020;
const int r2Max = 520;
const int r2Threshold = 10;
const int l2Threshold = 10;
const int maxMotorSpeed = 255;
const int leftJoystickMax = 520;
*/

// Gleichstrommotor 1 (rechts)
const int GSM1 = 27;
const int in1 = 12;
const int in2 = 14;
// Gleichstrommotor 2 (links)
const int GSM2 = 33;
const int in3 = 26;
const int in4 = 25;

//Piezo Pieper
const int beep = 4;

//Lebensanzeige - LEDs
const int Led1 = 18;
const int Led2 = 19;
const int Led3 = 21;

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

void moveMotors(bool linksVorwaerts, bool rechtsVorwaerts) {
    
    if( leben < 1) {
        analogWrite(GSM1, 0);
        analogWrite(GSM2, 0);
    } else {
        analogWrite(GSM1, 255);
        analogWrite(GSM2, 255);
    }
    /*digitalWrite(in1, linksVorwaerts ? HIGH : LOW);
      digitalWrite(in2, linksVorwaerts ? LOW : HIGH);
      digitalWrite(in3, rechtsVorwaerts ? HIGH : LOW);
      digitalWrite(in4, rechtsVorwaerts ? LOW : HIGH);
    */
      if (linksVorwaerts && rechtsVorwaerts) {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
      }

      if (!linksVorwaerts && !rechtsVorwaerts) {
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
      }

      if (!linksVorwaerts && rechtsVorwaerts) {
          digitalWrite(in1, LOW);
          digitalWrite(in2, HIGH);
          digitalWrite(in3, HIGH);
          digitalWrite(in4, LOW);
      }

      if (linksVorwaerts && !rechtsVorwaerts) {
          digitalWrite(in1, HIGH);
          digitalWrite(in2, LOW);
          digitalWrite(in3, LOW);
          digitalWrite(in4, HIGH);
      }
}


void processGamepad(ControllerPtr ctl) {


//== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001 && leben < 1) {
      //alle 3 leds anmachen ( der riehe nach am besten) 
      digitalWrite(Led1, HIGH);
      delay(1000);
      digitalWrite(Led2, HIGH);
      delay(1000);
      digitalWrite(Led3, HIGH);
      delay(1000);
      leben = 3;   
  }

  if (ctl->throttle() >= 10) { //R2

        if (ctl->axisX() <= 50 && ctl-> joystick() >= -50) { //gerade
          moveMotors(true,true);
        }
      
       if (ctl->axisX() > 50) { //rechts
         moveMotors(true, false);
       }

       if (ctl->axisX() < -50) { //links
         moveMotors(false, true);
       }    

  } 
    
  if (ctl->brake() >= 10) { // L2

        if (ctl->axisX() <= 50 && ctl-> joystick() >= -50) { //gerade
          moveMotors(false,false);
        }
      
       if (ctl->axisX() > 50) { //rechts
         moveMotors(false, true);
       }

       if (ctl->axisX() < -50) { //links
         moveMotors(true, false);
       }    
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
    bool istfahrzeuggetroffen = (digitalRead(beruhrungskabel) == 1) && ( (millis() - letzteberuhrung) > 3000);
    static bool doppelPiepen = false;
    if (istfahrzeuggetroffen && leben >= 1) {
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
