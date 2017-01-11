# e-bike Electronic Control Unit v2
În acest repo se regăsește implementarea software pentru proiectul *e-Bike ECU v2*.<br>
<br>
Sistemul de operare ce va gestiona sistemul este **FreeRTOS v9.0.0.**<br>
Software-ul este implementat pe un microcontroller AVR de 8 biți, ATmega128A (HW v1).<br>
<br>
Continut foldere:<br>
**test_FreeRTOS_blink-a-LED** - primul test pentru FreeRTOS portat pe ATmega128A<br>
**test_FreeRTOS_UART** - implementarea funcțiilor de transmisie de date prin interfața UART<br>
**ebike_controller_RTOS** - proiectul final pentru e-Bike v2<br>
<br>
Funcționalități implementate în programul final:
* controlul unui motor BLDC (Brushless Direct Current) folosind o schemă de comutație dreptunghiulară
* control variabil al turației motorului
* controlul unui bloc de lumini (far, semnalizari)
* monitorizarea de parametri relevanți:
  * tensiunea bateriei;
  * viteza instantanee;
  * temperatura internă a modulului;
  * consumul mediu al motorului.
* transmiterea de date importante prin interfață UART (Bluetooth) către un telefon
* recepționarea de comenzi prin intefața UART (Bluetooth)
* măsuri de protecție:
  * OCP (supracurent motor);
  * UVP (subtensiune baterie);
  * OTP (supratemperatură modul electronic)

Video test:
[https://youtu.be/RDsVpeOm2Dg](https://youtu.be/RDsVpeOm2Dg)<br>
După cum se poate observa și în videoclipul de mai sus, momentan există mici probleme legate de transmisia de date.<br>
(se pare că uneori transmisia mesajelor este întreruptă).
