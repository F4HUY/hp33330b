# hp33330b
This is a adaptation of POWER-METER-HP33330B-M5STACK from F1CJN, a instrument to mesure RF power, using HP33330B diode detector, without M5stack use, ESP32DEVKIT + 128x64 LCD I2C instead in this version.

Describe here https://www.f4huy.fr/2024/02/02/mesurer-la-puissance-rf-en-utilisant-une-sonde-hp33330b/
Original project from F1CJN is available here https://github.com/F1CJN/POWER-METER-HP33330B-M5STACK
You need u8g2 librarie as well https://github.com/olikraus/u8g2 , µC used is ESP32DEVKIT , LCD used is 128x64, I2C ST7567S chip , attenuator selection button is connected to D 14 ESP pin, SDA/SCK i2c LCD lines are connected to D21/D22 pins, lcd is powered on 3.3v from ESP.

REV2: 23/3/2024 - RF_Power_Meter_rev6.ino include a frequency meter, code sourced and merged from this topic https://www.esp32.com/viewtopic.php?t=17018

NOTE: Frequency Meter from 1 Hz to 40 MHz , so you can add a prescaler to measure more, but need code modification as well.
There is also included an Oscillator to test Frequency Meter. Connect the GPIO 33 to GPIO 34 to test it.

NOTE2: Pulses are counted both as the pulse rising and falling, to improve the counting average.
The sampling time is defined by the high resolution esp-timer, and it is defined in 1 second, in the sample-time variable.
If the count is greater than 20000 pulses during the counting time, overflow occurs and for with each overflow that occurs
the multPulses variable is incremented. Then pulse counter is cleared and proceed to counting.
Unfortunately the Pulse Counter has only 16 bits that may be used. 
THAT WORK FINE FOR SQUARE/TRIANGLE signal, but less fore SINE signal, FOR EXPERIMENTATION ONLY
