#include <Servo.h>

Servo s0, s1, s2;
String buffer = "";

void setup() {
  Serial.begin(115200);
  
  s0.attach(9);   // База
  s1.attach(10);  // Плечо
  s2.attach(11);  // Локоть
  
  // При старте Arduino ставим руку в положение "I"
  s0.write(90);
  s1.write(0);
  s2.write(90);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (buffer.length() > 0) {
        int c1 = buffer.indexOf(',');
        int c2 = buffer.indexOf(',', c1 + 1);
        
        if (c1 > 0 && c2 > 0) {
          int a0 = buffer.substring(0, c1).toInt();
          int a1 = buffer.substring(c1 + 1, c2).toInt();
          int a2 = buffer.substring(c2 + 1).toInt();
          
          // Отправляем напрямую, без constrain
          s0.write(a0);
          s1.write(a1);
          s2.write(a2);
        }
        buffer = "";
      }
    } else {
      buffer += c;
    }
  }
}