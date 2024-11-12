// LED an Pin 7, Taster an Pin 6
int TasterGelb = 13;   

void setup()  
{
  
  // der Taster wird als INPUT (Eingang) deklariert
  pinMode(TasterGelb, INPUT);  
  Serial.begin(115200);
}

void loop() 
{  
  // Taster abfragen
  Serial.println(digitalRead(TasterGelb)); 
             
}