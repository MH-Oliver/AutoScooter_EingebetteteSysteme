#include <Bluepad32.h>

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



// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
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

// ========= SEE CONTROLLER VALUES IN SERIAL MONITOR ========= //

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
  "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
  "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
  ctl->index(),        // Controller Index
  ctl->dpad(),         // D-pad
  ctl->buttons(),      // bitmask of pressed buttons
  ctl->axisX(),        // (-511 - 512) left X Axis
  ctl->axisY(),        // (-511 - 512) left Y axis
  ctl->axisRX(),       // (-511 - 512) right X axis
  ctl->axisRY(),       // (-511 - 512) right Y axis
  ctl->brake(),        // (0 - 1023): brake button
  ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
  ctl->miscButtons(),  // bitmask of pressed "misc" buttons
  ctl->gyroX(),        // Gyro X
  ctl->gyroY(),        // Gyro Y
  ctl->gyroZ(),        // Gyro Z
  ctl->accelX(),       // Accelerometer X
  ctl->accelY(),       // Accelerometer Y
  ctl->accelZ()        // Accelerometer Z
  );
}

void moveMotors(bool linksVorwaerts, bool rechtsVorwaerts, int currentSpeedLeft, int currentSpeedRight) {
    analogWrite(GSM1,currentSpeedLeft);
    analogWrite(GSM2,currentSpeedRight);

    if (linksVorwaerts && rechtsVorwaerts) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }

    else if (linksVorwaerts && !rechtsVorwaerts) {
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }

    else if (!linksVorwaerts && rechtsVorwaerts) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
    }

    else if (!linksVorwaerts && !rechtsVorwaerts) {
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
    }

}

// ========= GAME CONTROLLER ACTIONS SECTION ========= //

void processGamepad(ControllerPtr ctl) {
  // There are different ways to query whether a button is pressed.
  // By query each button individually:
  //  a(), b(), x(), y(), l1(), etc...
 
  //== PS4 X button = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
    // code for when X button is pushed
  }
  if (ctl->buttons() != 0x0001) {
    // code for when X button is released
  }

  //== PS4 Square button = 0x0004 ==//
  if (ctl->buttons() == 0x0004) {
    // code for when square button is pushed
  }
  if (ctl->buttons() != 0x0004) {
  // code for when square button is released
  }

  //== PS4 Triangle button = 0x0008 ==//
  if (ctl->buttons() == 0x0008) {
    // code for when triangle button is pushed
  }
  if (ctl->buttons() != 0x0008) {
    // code for when triangle button is released
  }

  //== PS4 Circle button = 0x0002 ==//
  if (ctl->buttons() == 0x0002) {
    // code for when circle button is pushed
  }
  if (ctl->buttons() != 0x0002) {
    // code for when circle button is released
  }

  //== PS4 Dpad UP button = 0x01 ==//
  if (ctl->buttons() == 0x01) {
    // code for when dpad up button is pushed
  }
  if (ctl->buttons() != 0x01) {
    // code for when dpad up button is released
  }

  //==PS4 Dpad DOWN button = 0x02==//
  if (ctl->buttons() == 0x02) {
    // code for when dpad down button is pushed
  }
  if (ctl->buttons() != 0x02) {
    // code for when dpad down button is released
  }

  //== PS4 Dpad LEFT button = 0x08 ==//
  if (ctl->buttons() == 0x08) {
    // code for when dpad left button is pushed
  }
  if (ctl->buttons() != 0x08) {
    // code for when dpad left button is released
  }

  //== PS4 Dpad RIGHT button = 0x04 ==//
  if (ctl->buttons() == 0x04) {
    // code for when dpad right button is pushed
  }
  if (ctl->buttons() != 0x04) {
    // code for when dpad right button is released
  }

  //== PS4 R1 trigger button = 0x0020 ==//
  if (ctl->buttons() == 0x0020) {
    // code for when R1 button is pushed
  }
  if (ctl->buttons() != 0x0020) {
    // code for when R1 button is released
  }

  //== PS4 R2 trigger button = 0x0080 ==//
  if (ctl->buttons() == 0x0080) {
    // code for when R2 button is pushed
  }
  if (ctl->buttons() != 0x0080) {
    // code for when R2 button is released
  }

  //== PS4 L1 trigger button = 0x0010 ==//
  if (ctl->buttons() == 0x0010) {
    // code for when L1 button is pushed
  }
  if (ctl->buttons() != 0x0010) {
    // code for when L1 button is released
  }

  //== PS4 L2 trigger button = 0x0040 ==//
  if (ctl->buttons() == 0x0040) {
    // code for when L2 button is pushed
  }
  if (ctl->buttons() != 0x0040) {
    // code for when L2 button is released
  }
/*
//backwards driving (l2)
//noch kein scharfes lenken implementiert
//alter code, ich lasse den trotzdem noch hier falls ich was versaue
if (ctl->l2() >= l2Threshold) {
    
    int currentSpeedLeft;
    int currentSpeedRight;
    if (ctl->axisX() <= -joystickThreshold) {
    // code for when joystick is pushed left
    currentSpeedLeft = maxMotorSpeed*((double)ctl->throttle()/l2Max) * (1+((double)ctl->axisX()/leftJoystickMax));
    currentSpeedRight = 255*((double)ctl->throttle()/l2Max);
    }

    else if (ctl->axisX() >= joystickThreshold) {
    // code for when left joystick is pushed right
    //Serial.println(1+((double)ctl->axisX()/leftJoystickMax));
    currentSpeedLeft = maxMotorSpeed*((double)ctl->throttle()/l2Max);
    currentSpeedRight = maxMotorSpeed*((double)ctl->throttle()/l2Max) * (1-((double)ctl->axisX()/leftJoystickMax));
    }

    else {
      currentSpeedLeft = maxMotorSpeed*((double)ctl->throttle()/l2Max);
      currentSpeedRight = maxMotorSpeed*((double)ctl->throttle()/l2Max);
    }

    Serial.println(currentSpeedLeft);
    Serial.println(currentSpeedRight);
    
    moveMotors( true,  true,  currentSpeedLeft,  currentSpeedRight);
  }
*/


    //axisX kann werte von bis haben  // (-511 bis 512) left X Axis
    //wir teilen in leichtes lenken und scharfes lenken (scharf = ausserer bereich)
    int aufteilungsgrenzelinks = (-511 + joystickThreshold) / 2; //die grenze ist genau die mitte der moeglichen werte des joysticks (dann koennen wir fallunterscheidung machen bei starkem und lechten lenken)
    int aufteilungsgrenzerechts = (512 - joystickThreshold) / 2;



//forwards driving (r2)
// mit brake = r2
if (ctl->brake() >= r2Threshold) {
    
    int currentSpeedLeft;
    int currentSpeedRight;
    bool leftmotor;
    bool rightmotor;
    
    if (ctl->axisX() <= -joystickThreshold) {
    // code for when joystick is pushed left

  
    if (ctl->axisX() <= -joystickThreshold && aufteilungsgrenzelinks < ctl->axisX()) { //leichtes lenken (unverandert zu vorherigem lenken)
      currentSpeedLeft = maxMotorSpeed*((double)ctl->brake()/r2Max) * (1+((double)ctl->axisX()/leftJoystickMax));
      currentSpeedRight = 255*((double)ctl->brake()/r2Max);
      leftmotor = false; //beide motoren auf vorarts eingestellt
      rightmotor = false;
    
    } else { //scharfes lenken
      currentSpeedLeft = 255*((double)ctl->brake()/r2Max); //der motor, der vorher verlangsamt wurde, soll jetzt einfach ruckwarts fahren statt verlangsamt (eventuell kann man noch was an der zahl andern aber das muss getestet werden)
      currentSpeedRight = 255*((double)ctl->brake()/r2Max);
      leftmotor = true; //linker motor ruckwarts
      rightmotor = false;

    }

    }

    else if (ctl->axisX() >= joystickThreshold) {
    // code for when left joystick is pushed right

    if (ctl->axisX() >= joystickThreshold && aufteilungsgrenzerechts > ctl->axisX()) { //leichtes lenken (unverandert zu vorherigem lenken)
      currentSpeedLeft = 255*((double)ctl->brake()/r2Max);
      currentSpeedRight = maxMotorSpeed*((double)ctl->brake()/r2Max) * (1+((double)ctl->axisX()/leftJoystickMax));
      leftmotor = false; //beide motoren auf vorarts eingestellt
      rightmotor = false;
    


    } else { //scharfes lenken
      currentSpeedLeft = 255*((double)ctl->brake()/r2Max); //der motor, der vorher verlangsamt wurde, soll jetzt einfach ruckwarts fahren statt verlangsamt (eventuell kann man noch was an der zahl andern aber das muss getestet werden)
      currentSpeedRight = 255*((double)ctl->brake()/r2Max);
      leftmotor = false; 
      rightmotor = true; //rechter motor ruckwarts

    }


    }

    else {
      currentSpeedLeft = maxMotorSpeed*((double)ctl->brake()/r2Max);
      currentSpeedRight = maxMotorSpeed*((double)ctl->brake()/r2Max);
      leftmotor = false; //beide vorwarts
      rightmotor = false;
    }

    Serial.println(currentSpeedLeft);
    Serial.println(currentSpeedRight);
    
    moveMotors( leftmotor,  rightmotor,  currentSpeedLeft,  currentSpeedRight);
  }







//backwards driving (l2)
//mit throttle = l2
else if (ctl->throttle() >= l2Threshold) {
    
    int currentSpeedLeft;
    int currentSpeedRight;
    bool leftmotor;
    bool rightmotor;

    if (ctl->axisX() <= -joystickThreshold) {
    // code for when joystick is pushed left

    if (ctl->axisX() <= -joystickThreshold && aufteilungsgrenzelinks < ctl->axisX()) { //leichtes lenken (unverandert zu vorherigem lenken)
      currentSpeedLeft = maxMotorSpeed*((double)ctl->throttle()/l2Max) * (1+((double)ctl->axisX()/leftJoystickMax));
      currentSpeedRight = 255*((double)ctl->throttle()/l2Max);
      leftmotor = true; //beide motoren auf ruckwarts eingestellt
      rightmotor = true;
    
    } else { //scharfes lenken (ruckwarts scharf links)
      currentSpeedLeft = 255*((double)ctl->throttle()/l2Max); //der motor, der vorher verlangsamt wurde, soll jetzt einfach ruckwarts fahren statt verlangsamt (eventuell kann man noch was an der zahl andern aber das muss getestet werden)
      currentSpeedRight = 255*((double)ctl->throttle()/l2Max);
      leftmotor = false; //linker motor vorwarts
      rightmotor = true; //rechter ruckwarts (wenn man ruckwarts fahrt und scharf links einlenkt, dann muss der linke reifen vorwarts drehen und der rechte ruckwarts)

    }

    }

    else if (ctl->axisX() >= joystickThreshold) {
    // code for when left joystick is pushed right

    if (ctl->axisX() >= joystickThreshold && aufteilungsgrenzerechts > ctl->axisX()) { //leichtes lenken (unverandert zu vorherigem lenken)
      currentSpeedLeft = 255*((double)ctl->throttle()/l2Max);
      currentSpeedRight = maxMotorSpeed*((double)ctl->throttle()/l2Max) * (1+((double)ctl->axisX()/leftJoystickMax));

      leftmotor = true; //beide motoren auf ruckwarts eingestellt
      rightmotor = true;
    
    } else { //scharfes lenken
      currentSpeedLeft = 255*((double)ctl->throttle()/l2Max); //der motor, der vorher verlangsamt wurde, soll jetzt einfach ruckwarts fahren statt verlangsamt (eventuell kann man noch was an der zahl andern aber das muss getestet werden)
      currentSpeedRight = 255*((double)ctl->throttle()/l2Max);
      leftmotor = true; 
      rightmotor = false; //rechter motor forwarts

    }


    }

    else {
      currentSpeedLeft = maxMotorSpeed*((double)ctl->throttle()/l2Max);
      currentSpeedRight = maxMotorSpeed*((double)ctl->throttle()/l2Max);
      leftmotor = true;
      rightmotor = true;
    }

    Serial.println(currentSpeedLeft);
    Serial.println(currentSpeedRight);
    
    moveMotors( leftmotor,  rightmotor,  currentSpeedLeft,  currentSpeedRight);
  }

  else {
    Serial.println("Still stehen");
    moveMotors( true,  true,  0,  0);
  }







































  if (ctl->r2() <= r2Threshold) {

  }

  //== LEFT JOYSTICK - UP ==//
  if (ctl->axisY() <= -joystickThreshold) {
    // code for when left joystick is pushed up
    }

  //== LEFT JOYSTICK - DOWN ==//
  if (ctl->axisY() >= joystickThreshold) {
    // code for when left joystick is pushed down
  }

  //== LEFT JOYSTICK - LEFT ==//
  if (ctl->axisX() <= -joystickThreshold) {
    // code for when left joystick is pushed left
  }

  //== LEFT JOYSTICK - RIGHT ==//
  if (ctl->axisX() >= joystickThreshold) {
    // code for when left joystick is pushed right
  }

  //== LEFT JOYSTICK DEADZONE ==//
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisX() > -25 && ctl->axisX() < 25) {
    // code for when left joystick is at idle
  }

  //== RIGHT JOYSTICK - X AXIS ==//
  if (ctl->axisRX()) {
    // code for when right joystick moves along x-axis
  }

  //== RIGHT JOYSTICK - Y AXIS ==//
  if (ctl->axisRY()) {
  // code for when right joystick moves along y-axis
  }
  dumpGamepad(ctl);
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        /*for (int i = 0; i < 6; i++) {
          	Serial.println(myController->getProperties().btaddr[i]);
        }
        Serial.println("-------------");*/
        if (myController->getProperties().btaddr[0] == 28 &&
            myController->getProperties().btaddr[1] == 160 && 
            myController->getProperties().btaddr[2] == 184 &&
            myController->getProperties().btaddr[3] == 14 &&
            myController->getProperties().btaddr[4] == 242 &&
            myController->getProperties().btaddr[5] == 104
            ) {
              processGamepad(myController);
            }
            else {
              myController->disconnect();
            }
         
         
      }
      else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  Serial.begin(115200);

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  pinMode(GSM1, OUTPUT);
  pinMode(GSM2, OUTPUT);

  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  //BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
}

// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated)
    processControllers();

    /*Serial.println("test");
    analogWrite(GSM2,200);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);*/
  

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    // vTaskDelay(1);
  delay(150);
}
