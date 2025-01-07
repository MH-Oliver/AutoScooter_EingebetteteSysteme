# include "PlayRtttl.hpp"
# include "LiquidCrystal_I2C.h"

// Name des LCDs (lcd) festlegen
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Pin des Kautsprechers
# define Lautsprecher  4

// Variable für die gedrückte Taste des Tastenpads
int GedrueckteTaste;
String TitelSong;

char TeDeum[] =
"Charpentier/Te_Deum:d=4,o=5,b=63:8c,8f,16f,16g,8a,8f,c6,8a,8a,8a#,16c6,16a#,16a,16a#,8c6,16g,16f,16g,16a,8g,8c,8f,16f,16g,8a,8f,c6,8a,8a,16a#,16c6,16a,16a#,g,16f,2f";

char FuerElise[] =
"Beethoven/Für_Elise:d=8,o=5,b=125:32p,e6,d#6,e6,d#6,e6,b,d6,c6,4a.,32p,c,e,a,4b.,32p,e,g#,b,4c.6,32p,e,e6,d#6,e6,d#6,e6,b,d6,c6,4a.,32p,c,e,a,4b.,32p,d,c6,b,2a";

char OdeandieFreude[] =
"Beethoven/Ode_an_die_Freude:d=4,o=6,b=100:a5,a5,a_5,c,c,a_5,a5,g5,f5,f5,g5,a5,a.5,8g5,2g5,";

char Bolero[] =
"Ravel/Bolero:d=4,o=5,b=80:c6,8c6,16b,16c6,16d6,16c6,16b,16a,8c6,16c6,16a,c6,8c6,16b,16c6,16a,16g,16e,16f,2g,16g,16f,16e,16d,16e,16f,16g,16a,g,g,";

void setup()
{
  // LCD starten
  lcd.init();

  // Hintergrundbeleuchtung des LCDs einschalten
  lcd.backlight();
}

void loop()
{
  playRtttlBlocking(Lautsprecher, OdeandieFreude);
}

void ZeigeTitel()
{
  // Teil des Strings bis zum ersten Doppelpunkt kürzen, enthält die Informationen zu Interpret und Titel
  TitelSong = TitelSong.substring(0, TitelSong.indexOf(":"));

  // _ durch leerzeichen ersetzen
  TitelSong.replace("_", " ");

  // erster Teil des Strings bis zum / -> Name des Interpreten
  String Interpret = TitelSong.substring(0, TitelSong.indexOf("/"));

  // zweiter Teil des Strings vom / + 1 bis zum Ende des Strings -> Name des Titels
  String Titel = TitelSong.substring(TitelSong.indexOf("/") + 1, TitelSong.length());

   // Titel auf dem LCD anzeigen
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Spiele:");
  lcd.setCursor(0, 1);
  lcd.print(Interpret);
  lcd.setCursor(0, 2);
  lcd.print(Titel);
}