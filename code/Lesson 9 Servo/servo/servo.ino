// www.elegoo.com
// 2018.12.19
#include <Servo.h>
Servo myservo;

void setup() {
    myservo.attach(9);
}

void loop() {
    for (int a = 0; a <= 180; a++) {
        myservo.write(a);
        delay(10);
    }
    for (int a = 180; a >= 0; a--) {
        myservo.write(a);
        delay(10);
    }
}
