/*
 * ESP32 Wattmetre RF utilisant une sonde Hewlett Packard HP33330B
 * RF Wattmeter using an HP33330B RF negative detector
 * By Ham radio F1CJN HP33330B from F1GE f1ge.mg at gmail.com
 * alain.fort.f1cjn at gmail.com
 * F4HUY - 12/2023 - ESP32 on 128x64LCD ONLY
 * F4huy.ham [at] gmail.com
 
 * Version V8 avec choix attenuateur d'entrée
 * Version V2.1  Grosse amélioration dans le calcul avec une interpolation logarithmique entre -10 et -30 dBm 
 * Version V2.2  avec  "Alerte niveau" sur écran  si P>20dBm   13/03/2023
 * Version V2.3 sur ESP32 + LCD 128x64
 * Version V3 rajout d'une lecture de frequence basé sur https://www.esp32.com/viewtopic.php?t=17018
 * Voltage = voltage read by the ADS1115 from the output of the OP192 == (HP33330B voltage x -2)
*/


#include <ADS1115_WE.h>
#include <Wire.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

#include "stdio.h"                                                        // Library STDIO
#include "driver/ledc.h"                                                  // Library ESP32 LEDC
#include "driver/pcnt.h"                                                  // Library ESP32 PCNT
#include "soc/pcnt_struct.h"

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Set Pulse Counter Unit - 0 
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Set Pulse Counter channel - 0 

#define PCNT_INPUT_SIG_IO     GPIO_NUM_34                                 // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_33                                 // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_35                                 // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_32                                 // Timer output control port - GPIO_32
#define PCNT_H_LIM_VAL        overflow                                    // Overflow of Pulse Counter 

#define IN_BOARD_LED          GPIO_NUM_2                                  // ESP32 native LED - GPIO 2

bool            flag2          = true;                                     // Flag to enable print frequency reading
uint32_t        overflow      = 20000;                                    // Max Pulse Counter value
int16_t         pulses        = 0;                                        // Pulse Counter value
uint32_t        multPulses    = 0;                                        // Quantidade de overflows do contador PCNT
uint32_t        sample_time   = 977745;                                   // sample time of 1 second to count pulses                                                          
uint32_t        osc_freq      = 12000;                                    // Oscillator frequency - initial 12543 Hz (may be 1 Hz to 40 MHz)
uint32_t        mDuty         = 0;                                        // Duty value
uint32_t        resolution    = 0;                                        // Resolution value
float           frequency     = 0;                                        // frequency value
char            buf[32];                                                  // Buffer

esp_timer_create_args_t create_args;                                      // Create an esp_timer instance
esp_timer_handle_t timer_handle;                                          // Create an single timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // portMUX_TYPE to do synchronism

//U8G2_ST7567_ENH_DG128064I_F_SW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);  //dont use it, thats SLOWING code as nightmarre
U8G2_ST7567_ENH_DG128064I_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 4, 5);

ADS1115_WE adc = ADS1115_WE(0x48);
U8G2LOG u8g2log;
uint32_t updateTime = 0;                          // time for next update
float voltage = 0;
float puissance_uW = 0;
float puissance_dBm = 0;
float v_m;

int interval = 10;      // Update interval 10 milliseconds
int old_analog = 9999;  // Value last displayed
int value = 2, att = 0;
int d = 0;
int flag = 0;
//int diode = 1;
int gamme = 1, oldgamme = 3;
int moins = true;
bool toggle = true;

//**********    Variables pour la mesure des sondes 
//**********( Entrez les valeurs mesurées en fin de programme à partir des lignes 415)  ****
//**********                   Ne pas modifier les 9 lignes suivantes              *********                                   
 float   v_m30 =0.13;  // Tension mesurée en mV par le M5stack avec -30dBm au géné
 float   v_m20 =2.90;  // Tension mesurée en mV par le M5stack avec -20dBm au géné
 float   v_m10 = 29.0; // Tension mesurée en mV par le M5stack avec -10dBm au géné
 float   v_m5 = 76;    // Tension mesurée en mV par le M5stack avec -5dBm au géné
 float   v_0 = 175;    // Tension mesurée en mV par le M5stack avec  0dBm au géné
 float   v_5 = 372;    // Tension mesurée en mV par le M5stack avec +5dBm au géné
 float   v_10 = 729;   // Tension mesurée en mV par le M5stack avec +10dBm au géné
 float   v_15 = 1421;  // Tension mesurée en mV par le M5stack avec +15dBm au géné
 float   v_20 = 2047;  // Tension mesurée en mV par le M5stack avec +20dBm au géné
//******************************************************************************************

#define BUTTON_PIN1  14 // ESP32 pin GPIO23, which connected to button

// variables will change:
int button_state1;       // the current state of button
int last_button_state1;  // the previous state of button

void setup(void) 
{
  Wire.begin();
  Serial.begin(115200);

  u8g2.setI2CAddress(0x3F * 2);
  u8g2.begin();

  init_frequencyMeter (); //init freq meter

  pinMode(BUTTON_PIN1, INPUT_PULLUP); // set ESP32 pin to input pull-up mode
  button_state1 = digitalRead(BUTTON_PIN1);

  adc.setVoltageRange_mV(ADS1115_RANGE_2048);  //ADC range max= 2,047V  avec 32768 valeurs
  adc.setMeasureMode(ADS1115_CONTINUOUS); //comment line/change parameter to change mode
  adc.setCompareChannels(ADS1115_COMP_0_GND);  // mesure channel 0 par rapport à la masse
  
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(35, 20, "HP33330B"); // write something to the internal memory
  u8g2.drawStr(12, 40, "RF POWER METER");
  u8g2.sendBuffer(); // transfer internal memory to the display

  delay(1000);
  u8g2.clearBuffer(); // clear the internal memory


  if (!adc.init()) {
    Serial.println("ADS1115 not connected");
    adc.setMeasureMode(ADS1115_CONTINUOUS);
    u8g2.drawStr(0, 50, "ASD1115 not connected");
  }
  delay(50);
  updateTime = millis();  // Save update time
}

void loop()
{
    if (flag2 == true)                                                     // If count has ended
  {
    flag2 = false;                                                       // change flag to disable print
    frequency = (pulses + (multPulses * overflow)) / 2  ;               // Calculation of frequency
    printf("Frequency : %s", (ltos(frequency, buf, 10)));               // Print frequency with commas
    printf(" Hz \n");                                                   // Print unity Hz
    u8g2.clearBuffer(); // clear the internal memory
    u8g2.drawStr(40, 48, (ltos(frequency, buf, 10)));
    //u8g2.sendBuffer(); // transfer internal memory to the display

    multPulses = 0;                                                     // Clear overflow counter
    // Put your function here, if you want
    //delay (100);                                                        // Delay 100 ms
    // Put your function here, if you want

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Clear Pulse Counter
    esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             // Set enable PCNT count
  }

  String inputString = "";                                               // clear temporary string
  osc_freq = 0;                                                          // Clear oscillator frequency
  while (Serial.available())
  {
    char inChar = (char)Serial.read();                                   // Reads a byte on the console
    inputString += inChar;                                               // Add char to string
    if (inChar == '\n')                                                  // If new line (enter)
    {
      osc_freq = inputString.toInt();                                    // Converts String into integer value
      inputString = "";                                                  // Clear string
    }
  }
  if (osc_freq != 0)                                                     // If some value inputted to oscillator frequency
  {
    init_osc_freq ();                                                    // reconfigure ledc function - oscillator 
  }
  

  delay(80);
  buttons();
  Serial.println("Starting loop ...");
  u8g2.setCursor(40, 60);
  u8g2.print("Att");
  u8g2.setCursor(60, 60);
  u8g2.print(att);

  u8g2.setCursor(82, 48);
  u8g2.print("Hz");

  last_button_state1 = button_state1;      // save the last state
  button_state1 = digitalRead(BUTTON_PIN1); // read new state
 
    Serial.print("Voltage: ");
    Serial.println(voltage);
    updateTime = millis() + interval;  // Update interval

    v_m = 0;
    voltage = 0;
    for (int i = 1; i <= 10; i++)
    {
      voltage = readChannel(ADS1115_COMP_0_GND);  // mesure ADC
      delay(10);
      v_m = v_m + voltage;
    }  //tension en mv

    u8g2.setCursor(40, 10);
    u8g2.print(voltage);
    u8g2.drawStr(80, 10, " mV");

    voltage = v_m / 10;  // moyenne de 10 mesures  // Vmax = 4V avec ampli G=2
   
    //if (voltage <= 0) {(voltage = v_m30);}  // si offset negatif ) la mise sous tension
    //Serial.print("Voltage mV: ");
    //Serial.println(voltage);

    // Square law de -30 à -10dBm
    //{puissance_dBm= -pow(10,(log10(Y0)+log10(Y1/Y0) * log10(X/X0) /log10(X1/X0)));// formule interpolation log } 

    if ((voltage > v_m30) && (voltage <= v_m20))                            // -30 à -20 dBm
    {Serial.print("OK30");puissance_dBm= -pow(10,(log10(20)+log10(1.5) * log10(voltage/v_m20) /(log10(v_m30/v_m20))));}// interpolation log  

    if ((voltage > v_m20) && (voltage <= v_m10))                          // -20 à -10 dBm
    {Serial.print("OK20");puissance_dBm= -pow(10,(log10(10)+log10(2) * log10(voltage/v_m10) /log10(v_m20/v_m10)));} // interpolation quadratique

    if ((voltage > v_m10) && (voltage <= v_m5)){ puissance_dBm = ((5 / (v_m5 - v_m10)) * (voltage - v_m10)) - 10; }  // -10 à -5dBm
    if ((voltage > v_m5) && (voltage <= v_0)) { puissance_dBm = ((5 / (v_0 - v_m5)) * (voltage - v_m5)) - 5; }        // -5 à 0dBm
    if ((voltage > v_0) && (voltage <= v_5)) { puissance_dBm = ((5 / (v_5 - v_0)) * (voltage - v_0)); }               // 0 à +5dBm
    if ((voltage > v_5) && (voltage <= v_10)) { puissance_dBm = ((5 / (v_10 - v_5)) * (voltage - v_5)) + 5; }         // +5 à +10dBm
    if ((voltage > v_10) && (voltage <= v_15)) { puissance_dBm = ((5 / (v_15 - v_10)) * (voltage - v_10)) + 10; }     // +10 à +15dBm
    if ((voltage > v_15) && (voltage <= v_20)) { puissance_dBm = ((5 / (v_20 - v_15)) * (voltage - v_15)) + 15; }     // +15 à +20dBm
    if (voltage > v_20) { puissance_dBm = 20 ; } // limite 20dBm                                                               
    if (voltage <= v_m30){puissance_dBm=-30;}  // si puissance <= -30dBm

    //  Butée si P >+20dBm
    if (voltage >= v_20 + 10) 
    {
      u8g2.drawStr(25, 12, "Danger Niveau"); 
    }
    Serial.print("Puissance dBm: "); 
    Serial.println(puissance_dBm,DEC);
  
    float z = pow(10, ((puissance_dBm + 30) / 10));
    Serial.print("z=");
    Serial.println(z);  // power with microwatt
    puissance_uW = z;

  if ((puissance_dBm > 10) && (puissance_dBm <= 20))  // gamme 10 à 20dBm
  {
    value = (puissance_dBm - 10) * 10;
    gamme = att + 30;

  } else if ((puissance_dBm >= 0) && (puissance_dBm <= 10))  // gamme 0 à 10dBm
  {
    value = puissance_dBm * 10;
    gamme = att + 20;

  } else if ((puissance_dBm < 0) && (puissance_dBm >= -10))  // gamme -10 à 0dBm
  {
    value = (puissance_dBm + 10) * 10;
    gamme = att + 10;

  } else if ((puissance_dBm < -10) && (puissance_dBm >= -20))  //gamme -20 à -10dBm
  {
    value = (puissance_dBm + 20) * 10;
    gamme = att;

  } else if ((puissance_dBm < -20) && (puissance_dBm >= -30))  //gamme -30 à -20dBm
  {
    value = (puissance_dBm + 30) * 10;
    gamme = att - 10;

  } else if (puissance_dBm <= -30) {
    value = 1; // valeur pour aiguille
    gamme = att - 10;
  }  //gamme -30 à -20dBm {value=-2;}

  if (flag == 0) {
    gamme = 0;

    flag = 1;
  }  // mise sous tension
  if (gamme <= oldgamme) {
    moins = true;
  } else {
    moins = false;
  }
  if ((gamme != oldgamme) || (flag == 0)) {
    oldgamme = gamme;

    flag = 1;
  }  // changement de gamme

  Serial.print("Gamme: ");
  Serial.println(gamme);
 
  if (toggle == false) 
  {
    u8g2.setCursor(0, 18);
    u8g2.print("           ");
    u8g2.setCursor(40, 10);
    u8g2.print(voltage);
    u8g2.drawStr(80, 10, "mV ");

  }

  else if (toggle == true) 
  {
    u8g2.setCursor(15, 18);
    u8g2.print("            ");
    u8g2.setCursor(40, 22);
    u8g2.print(puissance_dBm + att);
    u8g2.drawStr(80, 22, " dBm "); 
    
    if ((puissance_uW <= 1000) && (att == 0))  // affichage puissance avec attenuateur = 0 dB
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW);
      u8g2.drawStr(80, 35, "    "); 
      u8g2.drawStr(80, 35, " uW "); 
      }

    if ((puissance_uW > 1000) && (att == 0)) 
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 1000);
      u8g2.drawStr(80, 35, "    "); 
      u8g2.drawStr(80, 35, " mW "); 
      }
  
    if (att == 10)  // affichage puissance avec attenuateur = 10 dB
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 100);
      u8g2.drawStr(80, 35, "    "); 
      u8g2.drawStr(80, 35, " mW "); 
       }

    if ((puissance_uW <= 1000) && (att == 20))  // affichage puissance avec attenuateur = 20 dB
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 10);
      u8g2.drawStr(80, 35, "    "); 
      u8g2.drawStr(80, 35, " mW "); 
     }
    if ((puissance_uW > 1000) && (att == 20)) 
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 10000);
      u8g2.drawStr(70, 35, "   "); 
      u8g2.drawStr(70, 35, " W "); 
     }
    if (att == 30)  // affichage puissance avec attenuateur = 30 dB
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 1000);
      u8g2.drawStr(80, 35, "   "); 
      u8g2.drawStr(80, 35, " W "); 
      }

    if (att == 40)  // affichage puissance avec attenuateur = 40 dB
    { 
      u8g2.setCursor(15, 32);
      u8g2.print("           ");
      u8g2.setCursor(40, 35);
      u8g2.print(puissance_uW / 100);
      u8g2.drawStr(80, 35, "   "); 
      u8g2.drawStr(80, 35, " W "); 
      }
  }
      u8g2.sendBuffer(); // transfer internal memory to the display

}
  
  void buttons()
  {
       if (last_button_state1 == HIGH && button_state1 == LOW) 
  {
    Serial.println("ATT button pressed pin 23");
    att = att + 10;
    if (att == 50) att = -10;
    //delay(10);
  }

  }

float readChannel(ADS1115_MUX channel) 
{
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  //while (adc.isBusy()) {}
  voltage = adc.getResult_mV();  // alternative: getResult_mV for Millivolt
  return voltage;
}


//----------------------------FREQ READ SECTION-----------------------------
void init_osc_freq ()                                                     // Initialize Oscillator to test Freq Meter
{
  resolution = (log (80000000 / osc_freq)  / log(2)) / 2 ;                // Calc of resolution of Oscillator
  if (resolution < 1) resolution = 1;                                     // set min resolution 
  // Serial.println(resolution);                                          // Print
  mDuty = (pow(2, resolution)) / 2;                                       // Calc of Duty Cycle 50% of the pulse
  // Serial.println(mDuty);                                               // Print

  ledc_timer_config_t ledc_timer = {};                                    // LEDC timer config instance

  ledc_timer.duty_resolution =  ledc_timer_bit_t(resolution);             // Set resolution
  ledc_timer.freq_hz    = osc_freq;                                       // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_0;                                    // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                                         // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};                                // LEDC Channel config instance

  ledc_channel.channel    = LEDC_CHANNEL_0;                               // Set HS Channel - 0
  ledc_channel.duty       = mDuty;                                        // Set Duty Cycle 50%
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // LEDC Oscillator output GPIO 33
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Set LEDC high speed mode
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);                                     // Config LEDC channel
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // disabling the interrupts
  multPulses++;                                                           // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timerMux);                                       // enabling the interrupts
}

//----------------------------------------------------------------------------------
void init_PCNT(void)                                                      // Initialize and run PCNT unit
{
  pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Clear PCNT unit

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
void read_PCNT(void *p)                                                   // Read Pulse Counter
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Stop counter - output control LOW
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Read Pulse Counter value
  flag2 = true;                                                            // Change flag to enable print
}

//---------------------------------------------------------------------------------
void init_frequencyMeter ()
{
  init_osc_freq();                                                        // Initialize Oscillator
  init_PCNT();                                                            // Initialize and run PCNT unit

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Set GPIO 32 as output

  create_args.callback = read_PCNT;                                       // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);                          // Create esp-timer instance

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Set GPIO matrin IN - Freq Meter input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Set GPIO matrix OUT - to inboard LED
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos) // Format an unsigned long (32 bits) into a string
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = ',';
  return s;
}
//----------------------------------------------------------------------------------------
char *ltos(long val, char *s, int radix)                                  // Format an long (32 bits) into a string
{
  if (radix < 2 || radix > 36) {
    s[0] = 0;
  } else {
    char *p = s;
    if (radix == 10 && val < 0) {
      val = -val;
      *p++ = '-';
    }
    p = ultos_recursive(val, p, radix, 0) - 1;
    *p = 0;
  }
  return s;
}