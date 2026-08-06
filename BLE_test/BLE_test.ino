 #include <BleKeyboard.h>

 // Name the device whatever you want it to show up as in Bluetooth settings
 BleKeyboard bleKeyboard("ESP32_Dialer", "MyCompany", 100);

 void setup() {
   Serial.begin(115200);
   Serial.println("Starting BLE work!");
   bleKeyboard.begin();
 }

 void loop() {
   if(bleKeyboard.isConnected()) {
     // This is just a test.
     // Once you're connected, pressing the 'BOOT' button on the ESP32
     // will send these characters to your phone.
     if(digitalRead(0) == LOW) {
       bleKeyboard.print("*123#");
       delay(1000);
     }
   }
 }