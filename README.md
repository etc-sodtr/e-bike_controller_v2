# e-bike Electronic Control Unit v2
În acest repo se va regăsi implementarea software pentru proiectul *e-Bike ECU v2*.<br>
Sistemul de operare ce va gestiona sistemul este **FreeRTOS v9.0.0.**<br>
Versiunea beta (pentru teste) va fi implementată pe un microcontroller AVR de 8 biți ATmega128A (folosit pe v1),<br>
urmând ca după verificarea funcționării algoritmilor de comandă tot programul să fie portat pe un microcontroller ARM (MSP432P401R).<br><br>
Funcționalități ce vor fi implementate:
* controlul unui motor BLDC (Brushless Direct Current) folosind diferite scheme de comutație:
  * dreptunghiulară;
  * sinusoidală (opțional pentru faza beta).
* control variabil al turației motorului
* controlul unui bloc de lumini (far, stop)
* monitorizarea de parametri relevanți aplicației:
  * tensiunea bateriei;
  * viteza instantanee;
  * temperatura internă a modulului;
  * consumul mediu al motorului.
* transmiterea de date importante prin interfață Bluetooth către un telefon
* recepționarea de comenzi prin intefața Bluetooth
* măsuri de protecție:
  * OCP (supracurent motor);
  * UVP (subtensiune baterie);
  * OTP (supratemperatură modul electronic).