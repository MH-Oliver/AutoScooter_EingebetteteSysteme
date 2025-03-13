#include <Bluepad32.h>

//  Zur Verwaltung aller verbundenen Contoller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

//  Diese Methode wird immer aufgerufen, wenn ein neuer Controller angeschlossen wird.
//  Hierbei wird ein Controller in Form eines ControllerPtr-Objektes als Parameter übergeben, welcher im Anschluss der Liste an 
//  verbundenen Controllern angehängt wird.
//
//  Es können bis zu 4 Controller gleichzeitig angeschlossen werden.
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

//  Dient dem Trennen der Verbindung mit einem konkreten Controller
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

//  In diese Methode findet die tatsächliche Verabeitung der aktuellen Daten des Controllers statt.
//  Das übergebene "ControllerPtr"-Objekt beinhaltet alle verfügbaren Informationen über den Controller.
//
//  In unserem Fall können wir hier auf sich ändernde Daten, wie das Drücken der Taste "X", oder das bewegen eines Joysticks reagieren.
void processGamepad(ControllerPtr ctl) {
//== PS4 X Taste = 0x0001 ==//
  if (ctl->buttons() == 0x0001) {
      // Code, wenn X Taste gedrückt wird. 
      Serial.println("X funktioniert!");
  }

  //== LINKER JOYSTICK - LINKS ==//
  if (ctl->axisX() <= -25) {
    // Code, wenn der linke Joystick nach links gedrückt wird.
    // HINWEIS: Ein perfekter Controller sollte, sobald der Joystick nicht mehr gedrückt ist,
    //          Den Wert 0 aufweisen.
    //          Da die meisten Controller aber eine leichte Abweichung von diesem Wert haben, 
    //          wird hier erst bei einem Wert ab 25 reagiert.
    //          Ist der Controller Maximal nach Links geneigt, wird der Wert -508 erreicht.
  }

  //== LINKS JOYSTICK - RECHTS ==//
  if (ctl->axisX() >= 25) {
    // Code, wenn der linke Joystick nach rechts gedrückt wird
  }
    
}

//  Hier werden alle Controller duchgegangen, und für jeden die "processGamepad(ControllerPtr ctl)" Methode aufgerufen.
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
    Serial.begin(115200);

    //  Hier wird die Bluetooth-Verbindung gestartet, und die Bluetooth-Addresse des ESPs dargestellt.
    //  Im Weiteren Verlauf des Programms findet die eigentliche Verbindung mit einem PS4-Controller im Hintergrund statt.
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.enableVirtualDevice(false);
}

void loop() {

  	//  Hier wird überprüft, ob neue Daten vom Controller empfangen wurden. Wenn dem so ist wird die Verarbeitung gestartet ("processControllers()").
    //  Sobalt eine Verbindung besteht werden permanent neue Daten gesendet, auch wenn sich nichts am Contoller verändert hat.
    bool dataUpdated = BP32.update();
    if (dataUpdated) {
        processControllers();
    }
    delay(150);
}