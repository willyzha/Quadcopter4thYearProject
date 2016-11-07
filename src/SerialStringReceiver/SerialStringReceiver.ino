// Open a serial connection and flash LED when input is received

int ledRed = 4;      // LED connected to digital pin 4

void setup(){
 // Open serial connection.
 Serial.begin(9600);
 Serial.setTimeout(1000);
 pinMode(ledRed, OUTPUT);     
}

void loop(){
 if(Serial.available() > 0){      // if data present, blink
       String text = Serial.readStringUntil('\n');
       Serial.println(text);
   }
}
