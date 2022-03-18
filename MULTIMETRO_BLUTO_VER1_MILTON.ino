
#include <Arduino.h>
#include <max6675.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <stdio.h>
#include "ADS1X15.h"
#include "RTClib.h"
#include <Adafruit_MCP4725.h>

RTC_DS3231 rtc;

Adafruit_MCP4725 MCP4725;

ADS1115 ads0(0x48); /* Use this for the 16-bit version */
ADS1115 ads1(0x49); /* Use this for the 16-bit version */

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 16 chars and 2 line display

int thermoDO = 16;
int thermoCS = 17;
int thermoCLK = 18;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

/////////////////////////////////////////////////////////////////////
// TM1637
//  Module 1 connection pins (Digital Pins)
//#define CLK1 10
//#define DIO1 9

/////////////////////////////////////////////////////////////////////

//#define SSR 2    //3
//#define tiempoCiclo 5000

volatile int MIL_1 = 0;
volatile int CENTENA_1 = 0;
volatile int DECENA_1 = 0;
volatile int UNIDAD_1 = 0;

// const uint8_t DEGREES[] = {
//     0x0,0x0,0x0,

//   SEG_A | SEG_F | SEG_E | SEG_D,    // C
//   };

/////////////////////////////////////////////////////////////////////

// TM1637Display display1(CLK1, DIO1);// define dispaly 1 object
// uint8_t data[] = { 0x0, 0x0, 0x0, 0x0 };  // all segments clear

/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
// ENTRADAS

int B_BLU_TU = 19;

int B_SELEC_MEDICION = 42;
int B_SCALA_MEDICION = 43;
int B_GUARDAR_MEDICION = 47; // 45;//44;

int B_SELEC_GENERAR = 44; // 45;
int B_DATOS_GUARDADOS = 46;
int B_GUARDAR_GENERAR = 45; // 47;

// BORNERA   4
// int ADC_2_ELECTRO = 60   ;
const int OUT_PIN = 60;
/*
int ADC_0_ELECTRO = A5   ;
int D11_ELECTRO = A3   ;  //salida  220
int D13_ELECTRO = A4   ; // salida  1000
*/
#define cap_in_analogPin 59 // 0
#define chargePin 58        // 13
#define dischargePin 57     // 11

/*
//BORNERA 3
int D6_ELECTRO = A0   ;  //      2K
int D7_ELECTRO = A1   ;  //      20K
int D8_ELECTRO = A2   ;  //      470K
*/
int res_2k = 54;
int res_20k = 55;
int res_470k = 56;

// Modes variables
int mode = 0;
int res_scale = 0;
int cap_scale = 0;
bool mid_but_state = true;
bool top_but_state = true;

// Variables for voltage mode
volatile float Voltage = 0.0;
volatile float resistance_voltage = 0.0;
volatile float battery_voltage = 0.0;
volatile float measured_resistance = 0.0;

volatile float CAPACITOR = 0.0;
volatile float y = 0.0;

volatile float TEMPERATURAX = 0.0;

// Variables for resistance mode
float Res_2k_value = 2.010;  // 2K resistor          //CHANGE THIS VALUES. MEASURE YOUT 2k, 20k and 470K and put the real values here
float Res_20k_value = 19.62; // 20K resistor
float Res_470k_value = 470;  // 470K resistor

// Variables for big capacitance mode
#define resistorValue 9900.0F // Remember, we've used a 10K resistor to charge the capacitor MEASURE YOUR VALUE!!!
unsigned long startTime;
unsigned long elapsedTime;
float microFarads;
float nanoFarads;
const float IN_STRAY_CAP_TO_GND = 47.48;
const float IN_CAP_TO_GND = IN_STRAY_CAP_TO_GND;
const float R_PULLUP = 34.8;
const int MAX_ADC_VALUE = 1023;

// Variables for inductance mode
// D5 is the input to the circuit (connects to 150ohm resistor), 11 is the comparator/op-amp output.
double pulse, frequency, capacit, inductance;
float C_cap_value = 1E6; // The capacitor value used for the LC tank. See schematic. For me that is 2uF

// Variables for current mode with the ACS712 of 5A range
float Current_sensor_Resolution = 0.185;

/////////////////////////////////////////////////////////////////////
// SALIDAS

int LED_1 = 23;
int LED_2 = 24;
int LED_3 = 25;
int LED_4 = 26;
int LED_5 = 27;
int LED_6 = 28;
int LED_7 = 29;
int LED_8 = 30;
int LED_9 = 31;

// int KEY  = 2;
// int TX_X = 0;
// int RX_X  = 1;
// int VCC_BLU_TU = 3;

/////////////////////////////////////////////////////////////////////
// MEDICIONES

// 0-100V   ADC0_0           BORNERA 1
// 0-30V   ADC0_1            BORNERA 2
// RESISTENCIAS    ADC1_0    BORNERA 3
// CAPACITANCIA              BORNERA 4
// 0-10V   ADC0_2    0-20ma  BORNERA 5

// GENERACION
//  0-20mv   ADC0_3   0-20ma  BORNERA 6
//  micro voltaje TERMOPAR    BORNERA 7
//  VOLTAJE 0-30    ADC1_1    BORNERA 8

/////////////VARIABLES
int BANDERA_LIBRE_AUTOMATIC;
int jojo;

volatile float TEMPERATURA_TEMPORAL;
volatile int SETPOINT_TEMPORAL;

volatile int BANDERA_AUTO_MANUAL;

volatile int VERTICAL_MENU = 1;
volatile int MENU_PRINCIPAL = 1;

volatile int PRODUCTO;
volatile int SELLADO;
volatile int PASOS;
volatile int T_CORTE;

volatile unsigned long Tiempo_MOTOR_ON = 0;

int PULSO_MOTOR_TEMPORAL = 0;

volatile unsigned long RETARDO_MOTOR = 1;
volatile unsigned long TIEMPO_X = 0;

volatile int BANDERA_DE_VISUALIZACION = 0;

volatile int PASOS_HORIZONTE_MENU = 1;
volatile int PASOS_VERTICAL_MENU = 1;

volatile int TEMP_1 = 0;
volatile int VALOR = 0;

volatile int CONTADOR_MIL = 0;
volatile int CONTADOR_CENTENA = 0;
volatile int CONTADOR_DECENA = 0;
volatile int CONTADOR_UNIDAD = 0;

volatile int PRIMERA_VEZ_AUTO = 0;
volatile int PRIMERA_VEZ_MANUAL = 1;

volatile unsigned long TIEMPO_1T = 1;

volatile unsigned long PASOS_TEMPORAL = 1;

volatile int BANDERA_VISUALIZAR_TRANKI = 1;
volatile unsigned long v = 0;
volatile unsigned long T = 0;
volatile int MENU_GENERAR = 1;
volatile int MENU_MEDICION = 1;

volatile int MENU_INDICE_REGISTROS = 1;
volatile int V_DIA = 1;
volatile int V_MEZ = 1;
volatile int V_ANO = 1;

volatile int V_HORA = 1;
volatile int V_MINUTO = 1;
volatile int V_SEGUNDO = 1;

volatile int V_TIPO_0 = 1;
volatile int V_TIPO_1 = 1;

String V_TIPO_A;
String V_TIPO_B;

volatile int FRAME_REGISTROS = 0;

volatile float V_VALOR_MEDIDO = 0.0;

byte V_UNIDAD_MEDIDA[2];

volatile int VALORES_A_BUSCAR = 1;

volatile int OK = 0;

volatile int CERO_DIA;
volatile int CERO_MEZ;
volatile int CERO_ANO;

volatile int CERO_HORA;
volatile int CERO_MINUTO;
volatile int CERO_SEGUNDO;

volatile float VALOR_CERO = 35.465;

byte UNIDAD_MEDIDAX[2];
volatile int res_scale_X;

volatile int TIPO_UNO = 0;
volatile int TIPO_DOS = 0;

String FECHA;
String V_TIPO_UNIDAD;

String inputString = "";

int BANDERA_PRIMERA_VEZ_BLUTU = 1;
int BANDERA_PRIMERA_VEZ_PROGRAMA = 0;

char cadena[255];   // Creamos un array de caracteres de 256 cposiciones
volatile int i = 0; // Tamaño actual del array

volatile int res_scale_BLU = 0;

String ENVIAR_DATO;

uint32_t MCP4725_value;
int adcValueRead = 0;
float voltageRead = 0;

int MCP4725_expected_output;

int MUESTREO;
/////////////////////////////////////////////////////////////////////

// void serialEvent();
/////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////
void setup()
{

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    abort();
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  //  DateTime FECHA = rtc.now();

  lcd.init(); // initialize the lcd
  lcd.backlight();
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("                    ");
  lcd.setCursor(0, 1);
  lcd.print("UNI. EVANGELICA BOL ");
  lcd.setCursor(0, 2);
  lcd.print(" AUTOMATIC  # 1     ");
  lcd.setCursor(0, 3);
  lcd.print("                    ");

  delay(2000);
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print("MEDICION            ");
  lcd.setCursor(0, 1);
  lcd.print("     000.000 mv     ");
  lcd.setCursor(0, 2);
  lcd.print("GENERAR             ");
  lcd.setCursor(0, 3);
  lcd.print("     000.000 mv     ");

  pinMode(B_BLU_TU, INPUT); // A como entrada

  pinMode(B_SELEC_MEDICION, INPUT);   // B como entrada
  pinMode(B_SCALA_MEDICION, INPUT);   // A como entrada
  pinMode(B_GUARDAR_MEDICION, INPUT); // A como entrada

  pinMode(B_SELEC_GENERAR, INPUT);   // A como entrada
  pinMode(B_DATOS_GUARDADOS, INPUT); // A como entrada
  pinMode(B_GUARDAR_GENERAR, INPUT); // A como entrada

  // pinMode(dischargePin, OUTPUT);    // A como entrada
  pinMode(OUT_PIN, INPUT);          // A2 = A0 EN ELECTRONOOBS
  pinMode(chargePin, INPUT);        // A como entrada
  pinMode(cap_in_analogPin, INPUT); // A como entrada
  pinMode(dischargePin, INPUT);     // A como entrada

  pinMode(res_2k, INPUT);   // A como entrada
  pinMode(res_20k, INPUT);  // A como entrada
  pinMode(res_470k, INPUT); // A como entrada

  pinMode(LED_1, OUTPUT); // A como entrada
  pinMode(LED_2, OUTPUT); // A como entrada
  pinMode(LED_3, OUTPUT); // A como entrada
  pinMode(LED_4, OUTPUT); // A como entrada
  pinMode(LED_5, OUTPUT); // A como entrada
  pinMode(LED_6, OUTPUT); // A como entrada

  pinMode(LED_7, OUTPUT); // A como entrada
  pinMode(LED_8, OUTPUT); // A como entrada
  pinMode(LED_9, OUTPUT); // A como entrada

  digitalWrite(LED_1, LOW); // turn pullup resistor on
  digitalWrite(LED_2, LOW); // turn pullup resistor on
  digitalWrite(LED_3, LOW); // turn pullup resistor on
  digitalWrite(LED_4, LOW); // turn pullup resistor on
  digitalWrite(LED_5, LOW); // turn pullup resistor on
  digitalWrite(LED_6, LOW); // turn pullup resistor on

  digitalWrite(LED_7, LOW); // turn pullup resistor on
  digitalWrite(LED_8, LOW); // turn pullup resistor on
  digitalWrite(LED_9, LOW); // turn pullup resistor on

  digitalWrite(B_BLU_TU, HIGH); // turn pullup resistor on

  digitalWrite(B_SELEC_MEDICION, HIGH);   // turn pullup resistor on
  digitalWrite(B_SCALA_MEDICION, HIGH);   // turn pullup resistor on
  digitalWrite(B_GUARDAR_MEDICION, HIGH); // turn pullup resistor on

  digitalWrite(B_SELEC_GENERAR, HIGH);   // turn pullup resistor on
  digitalWrite(B_DATOS_GUARDADOS, HIGH); // turn pullup resistor on
  digitalWrite(B_GUARDAR_GENERAR, HIGH); // turn pullup resistor on

  ////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////

  ////  BANDERAS

  VERTICAL_MENU = 1;

  PASOS_HORIZONTE_MENU = 1;
  PASOS_VERTICAL_MENU = 1;

  BANDERA_AUTO_MANUAL = 1;

  MENU_PRINCIPAL = 1;

  BANDERA_VISUALIZAR_TRANKI = 1;

  // PONER A CERO LOS BYTE DE LA EEPROM

  /*
  for (int REGISTROS_G = 0; REGISTROS_G < 50 ; REGISTROS_G++){
    EEPROM.put(OK,CERO_DIA);
    EEPROM.put(OK+2,CERO_MEZ);
    EEPROM.put(OK+4,CERO_ANO);

    EEPROM.put(OK+6,CERO_HORA);
    EEPROM.put(OK+8,CERO_MINUTO);
    EEPROM.put(OK+10,CERO_SEGUNDO);

    EEPROM.put(OK+14,VALOR_CERO);
    EEPROM.put(OK+18,UNIDAD_MEDIDAX[0]);
    EEPROM.put(OK+19,UNIDAD_MEDIDAX[1]);


    EEPROM.put(OK+20,TIPO_UNO);
    EEPROM.put(OK+22,TIPO_DOS);

    OK= OK + 24;

  }
  */

  ads0.setGain(0); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  ads1.setGain(0); // 2/3x gain +/- 6.144V  1 bit = 3mV      0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V  1 bit = 2mV      0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V  1 bit = 0.5mV    0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V  1 bit = 0.25mV   0.015625mV
  // ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  ads0.begin();
  ads1.begin();

  inputString.reserve(200);

  MENU_GENERAR = 1;
  MENU_MEDICION = 1;

  res_scale = 0;

  MENU_INDICE_REGISTROS = 1;
  FRAME_REGISTROS = 0;

  BANDERA_PRIMERA_VEZ_BLUTU = 1;
  BANDERA_PRIMERA_VEZ_PROGRAMA = 0;

  MCP4725.begin(0x60);

  MCP4725_value = 0;
  MUESTREO = 0;
  MCP4725.setVoltage(MCP4725_value, false);
}
/////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

  ////////////////////////////////////////////////////////////////////////////////////
  if (digitalRead(B_BLU_TU) == 0)
  {

    PRIMERA_VEZ_BLUTU();

    goto PROGRAMA;
  }

  else
  {
    PRIMERA_VEZ_PROGRAMA();
  }

  ////////////////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////////////////

  if (digitalRead(B_SELEC_MEDICION) == 0)
  {
    INCREMENTAR_MENU_MEDICION();
  }

  if (digitalRead(B_SCALA_MEDICION) == 0)
  {
    SCALA_MEDICION();
  }

  if (digitalRead(B_GUARDAR_MEDICION) == 0)
  {
    GUARDAR_MEDICION();
  }

  if (digitalRead(B_SELEC_GENERAR) == 0)
  {
    INCREMENTAR_MENU_GENERAR();
  }

  if (digitalRead(B_DATOS_GUARDADOS) == 0)
  {
    VER_DATOS_GUARDADOS();
  }

  if (digitalRead(B_GUARDAR_GENERAR) == 0)
  {
    GUARDAR_GENERAR();
  }

  if (FRAME_REGISTROS == 0)
  {
    MEDIR();

    GENERAR();
  }

  delay(500);

PROGRAMA:

  int ss = 35;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void serialEvent()
{
  if (Serial.available())
  { //////
    Serial3.write(Serial.read());
  }
}
/*
void serialEvent3(){
  if (Serial3.available()) {//////
    Serial.write(Serial3.read());
  }
}
*/

void serialEvent3()
{
  if (Serial3.available())
  {

    char dato = Serial3.read(); // Guarda los datos carácter a carácter en la variable "dato"

    cadena[i++] = dato; // Vamos colocando cada carácter recibido en el array "cadena"

    // Cuando reciba una nueva línea (al pulsar enter en la app) entra en la función
    if (dato == 47)
    {

      lcd.clear();

      lcd.setCursor(0, 3);
      lcd.print("BLU_TU");

      Serial.print(cadena); // Visualizamos el comando recibido en el Monitor Serial
      Serial.write(13);     // Enviamos un retorno de carro de la app. La app ya crea una línea nueva
      Serial.write(10);     // Enviamos un retorno de carro de la app. La app ya crea una línea nueva

      ///////////////////////////////////////////////////////////////////////////////////////////
      // MEDIR
      if (strstr(cadena, "M_1") != 0)
      {

        // MEDIR VOLTAJE 0-100V

        digitalWrite(LED_1, HIGH); // turn pullup resistor on
        digitalWrite(LED_2, LOW);  // turn pullup resistor on
        digitalWrite(LED_3, LOW);  // turn pullup resistor on
        digitalWrite(LED_4, LOW);  // turn pullup resistor on
        digitalWrite(LED_5, LOW);  // turn pullup resistor on
        digitalWrite(LED_6, LOW);  // turn pullup resistor on

        int16_t adc0_0; // Leemos el ADC, con 16 bits
        adc0_0 = ads0.readADC(0);
        Voltage = (adc0_0 * 0.1875);

        Voltage = Voltage / 40.548022;
        Voltage = Voltage + 0.7;

        if (Voltage > 0.75)
        {
          Serial3.print(Voltage, 3);
          Serial3.print("_Voltaje/");
        }
        else
        {
          Voltage = 000.000;
          Serial3.print(Voltage, 3);
          Serial3.print("_Voltaje/");
        }
      }

      if (strstr(cadena, "M_2") != 0)
      {

        digitalWrite(LED_1, LOW);  // turn pullup resistor on
        digitalWrite(LED_2, HIGH); // turn pullup resistor on
        digitalWrite(LED_3, LOW);  // turn pullup resistor on
        digitalWrite(LED_4, LOW);  // turn pullup resistor on
        digitalWrite(LED_5, LOW);  // turn pullup resistor on
        digitalWrite(LED_6, LOW);  // turn pullup resistor on

        int16_t adc0_1; // Leemos el ADC, con 16 bits
        adc0_1 = ads0.readADC(1);
        Voltage = (adc0_1 * 0.1875);

        Voltage = Voltage / 127.55705;
        Voltage = Voltage + 0.7;

        if (Voltage > 0.75)
        {

          Serial3.print(Voltage, 3);
          Serial3.print("_Voltaje/");
        }
        else
        {

          Voltage = 0.000;
          Serial3.print(Voltage, 3);
          Serial3.print("_Voltaje/");
        }
      }

      if (strstr(cadena, "M_3") != 0)
      {

        digitalWrite(LED_1, LOW);  // turn pullup resistor on
        digitalWrite(LED_2, LOW);  // turn pullup resistor on
        digitalWrite(LED_3, HIGH); // turn pullup resistor on
        digitalWrite(LED_4, LOW);  // turn pullup resistor on
        digitalWrite(LED_5, LOW);  // turn pullup resistor on
        digitalWrite(LED_6, LOW);  // turn pullup resistor on

        if (res_scale_BLU == 0)
        {

          pinMode(res_2k, OUTPUT);
          pinMode(res_20k, INPUT);
          pinMode(res_470k, INPUT);
          digitalWrite(res_2k, LOW);

          int32_t adc1_0; // Leemos el ADC, con 16 bits
          adc1_0 = ads1.readADC(0);
          resistance_voltage = (adc1_0 * 0.1875) / 1000;

          int32_t adc1_3; // Leemos el ADC, con 16 bits
          adc1_3 = ads1.readADC(3);
          battery_voltage = ((adc1_3 * 0.1875) / 1000);

          measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_2k_value);

          measured_resistance = measured_resistance * 1000;

          if (measured_resistance < 4000)
          {

            Serial3.print(measured_resistance, 3);
            Serial3.print("_Ohms/");
            res_scale_X = res_scale_BLU;
          }
          else
          {

            measured_resistance = 0.000;
            Serial3.print(measured_resistance, 3);
            Serial3.print("_Ohms/");
            res_scale_X = res_scale_BLU;
          }
        }

        if (res_scale_BLU == 1)
        {
          pinMode(res_2k, INPUT);
          pinMode(res_20k, OUTPUT);
          pinMode(res_470k, INPUT);
          digitalWrite(res_20k, LOW);

          int16_t adc1_0; // Leemos el ADC, con 16 bits
          adc1_0 = ads1.readADC(0);
          resistance_voltage = (adc1_0 * 0.1875) / 1000;

          int16_t adc1_3; // Leemos el ADC, con 16 bits
          adc1_3 = ads1.readADC(3);
          battery_voltage = ((adc1_3 * 0.1875) / 1000);

          measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_20k_value);
          measured_resistance = measured_resistance;
          if (measured_resistance < 200)
          {

            Serial3.print(measured_resistance, 3);
            Serial3.print("_Kohms/");
            res_scale_X = res_scale_BLU;
          }
          else
          {

            measured_resistance = 0.000;
            Serial3.print(measured_resistance, 3);
            Serial3.print("_Kohms/");
            res_scale_X = res_scale_BLU;
          }
        }

        if (res_scale_BLU == 2)
        {
          pinMode(res_2k, INPUT);
          pinMode(res_20k, INPUT);
          pinMode(res_470k, OUTPUT);
          digitalWrite(res_470k, LOW);
          delay(100);

          int16_t adc1_0; // Leemos el ADC, con 16 bits
          adc1_0 = ads1.readADC(0);
          resistance_voltage = (adc1_0 * 0.1875) / 1000;

          int16_t adc1_3; // Leemos el ADC, con 16 bits
          adc1_3 = ads1.readADC(3);
          battery_voltage = ((adc1_3 * 0.1875) / 1000);

          measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_470k_value);
          measured_resistance = measured_resistance / 1000;

          if (measured_resistance < 2 && measured_resistance > 0)
          {
            Serial3.print(measured_resistance, 3);
            Serial3.print("_Mohms/");
            res_scale_X = res_scale_BLU;
          }

          else
          {
            measured_resistance = 0.000;
            Serial3.print(measured_resistance, 3);
            Serial3.print("_Mohms/");
            res_scale_X = res_scale_BLU;
          }
        }
      }

      if (strstr(cadena, "M_4") != 0)
      {

        digitalWrite(LED_1, LOW);  // turn pullup resistor on
        digitalWrite(LED_2, LOW);  // turn pullup resistor on
        digitalWrite(LED_3, LOW);  // turn pullup resistor on
        digitalWrite(LED_4, HIGH); // turn pullup resistor on
        digitalWrite(LED_5, LOW);  // turn pullup resistor on
        digitalWrite(LED_6, LOW);  // turn pullup resistor on

        int16_t adc1_3; // Leemos el ADC, con 16 bits
        adc1_3 = ads1.readADC(3);
        battery_voltage = ((adc1_3 * 0.1875) / 1000);

        if (!cap_scale)
        {

          pinMode(cap_in_analogPin, INPUT);

          pinMode(OUT_PIN, OUTPUT);
          digitalWrite(OUT_PIN, LOW);

          pinMode(chargePin, OUTPUT);
          digitalWrite(chargePin, HIGH); // apply 5 Volts

          pinMode(dischargePin, INPUT); // dischargePin

          startTime = micros();
          while (analogRead(cap_in_analogPin) < 648)
          {
          } // end while

          elapsedTime = micros() - startTime;
          microFarads = ((float)elapsedTime / resistorValue) - 0.01097;

          if (microFarads > 1)
          {

            Serial3.print(microFarads);
            Serial3.print("_uF/");
            CAPACITOR = microFarads;
            res_scale_X = 3;
          }

          else
          {

            cap_scale = true; // We change the scale to MIN - 1uF
          }

          digitalWrite(chargePin, LOW);
          pinMode(dischargePin, OUTPUT);
          digitalWrite(dischargePin, LOW); // discharging the capacitor
          while (analogRead(cap_in_analogPin) > 0)
          {
          }                             // This while waits till the capaccitor is discharged
          pinMode(dischargePin, INPUT); // this sets the pin to high impedance

        } // end of upper scale

        if (cap_scale)
        {
          pinMode(dischargePin, INPUT);
          pinMode(chargePin, INPUT);
          pinMode(OUT_PIN, OUTPUT);
          pinMode(cap_in_analogPin, INPUT);

          digitalWrite(OUT_PIN, HIGH);
          int val = analogRead(cap_in_analogPin);
          digitalWrite(OUT_PIN, LOW);

          if (val < 1000)
          {
            pinMode(cap_in_analogPin, OUTPUT);
            float capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);

            Serial3.print(capacitance);
            Serial3.print("_pF/");
            CAPACITOR = capacitance;
            res_scale_X = 4;
          }

          else
          {
            pinMode(cap_in_analogPin, OUTPUT);
            delay(1);
            pinMode(OUT_PIN, INPUT_PULLUP);
            unsigned long u1 = micros();
            unsigned long t;
            int digVal;
            do
            {
              digVal = digitalRead(OUT_PIN);
              unsigned long u2 = micros();
              t = u2 > u1 ? u2 - u1 : u1 - u2;
            } while ((digVal < 1) && (t < 400000L));

            pinMode(OUT_PIN, INPUT);
            val = analogRead(OUT_PIN);
            digitalWrite(cap_in_analogPin, HIGH);
            int dischargeTime = (int)(t / 1000L) * 5;
            delay(dischargeTime);
            pinMode(OUT_PIN, OUTPUT);
            digitalWrite(OUT_PIN, LOW);
            digitalWrite(cap_in_analogPin, LOW);
            float capacitance = -(float)t / R_PULLUP / log(1.0 - (float)val / (float)MAX_ADC_VALUE);

            if (capacitance > 1000.0)
            {

              cap_scale = false; // We change the scale to 1uF - max
            }
            else
            {
              Serial3.print(capacitance);
              Serial3.print("_nF/");
              CAPACITOR = capacitance;
              res_scale_X = 5;
            }
          }
          while (micros() % 1000 != 0)
            ;
        } ////end of lower scalee
      }

      if (strstr(cadena, "M_5") != 0)
      {

        digitalWrite(LED_1, LOW);  // turn pullup resistor on
        digitalWrite(LED_2, LOW);  // turn pullup resistor on
        digitalWrite(LED_3, LOW);  // turn pullup resistor on
        digitalWrite(LED_4, LOW);  // turn pullup resistor on
        digitalWrite(LED_5, HIGH); // turn pullup resistor on
        digitalWrite(LED_6, LOW);  // turn pullup resistor on

        int16_t adc0_2; // Leemos el ADC, con 16 bits
        adc0_2 = ads0.readADC(2);
        resistance_voltage = (adc0_2 * 0.1875);

        int16_t adc1_3; // Leemos el ADC, con 16 bits
        adc1_3 = ads1.readADC(3);
        battery_voltage = ((adc1_3 * 0.1875) / 1000);

        y = map(resistance_voltage, 0, 4340, 0, 20000);

        y = y / 1000;

        Serial3.print(y, 3);
        Serial3.print("_mA/");
      }

      if (strstr(cadena, "M_6") != 0)
      {

        digitalWrite(LED_1, LOW);  // turn pullup resistor on
        digitalWrite(LED_2, LOW);  // turn pullup resistor on
        digitalWrite(LED_3, LOW);  // turn pullup resistor on
        digitalWrite(LED_4, LOW);  // turn pullup resistor on
        digitalWrite(LED_5, LOW);  // turn pullup resistor on
        digitalWrite(LED_6, HIGH); // turn pullup resistor on

        TEMPERATURAX = thermocouple.readCelsius();
        Serial3.print(TEMPERATURAX, 3);
        Serial3.print("_C/");
      }

      ///////////////////////////////////////////////////////////////////////////////////////////
      // GENERAR
      if (strstr(cadena, "G_1") != 0)
      {

        digitalWrite(LED_7, HIGH); // turn pullup resistor on
        digitalWrite(LED_8, LOW);  // turn pullup resistor on
        digitalWrite(LED_9, LOW);  // turn pullup resistor on

        int16_t adc0_3; // Leemos el ADC, con 16 bits
        adc0_3 = ads0.readADC(3);
        resistance_voltage = ((adc0_3 * 0.1875));

        int16_t adc1_3; // Leemos el ADC, con 16 bits
        adc1_3 = ads1.readADC(3);
        battery_voltage = ((adc1_3 * 0.1875) / 1000);

        Serial3.print(resistance_voltage, 3);
        Serial3.print("_mA");
      }

      if (strstr(cadena, "G_2") != 0)
      {

        digitalWrite(LED_7, LOW);  // turn pullup resistor on
        digitalWrite(LED_8, HIGH); // turn pullup resistor on
        digitalWrite(LED_9, LOW);  // turn pullup resistor on

        Serial3.print(0.000, 3);
        Serial3.print("_--");
      }

      if (strstr(cadena, "G_3") != 0)
      {

        digitalWrite(LED_7, LOW);  // turn pullup resistor on
        digitalWrite(LED_8, LOW);  // turn pullup resistor on
        digitalWrite(LED_9, HIGH); // turn pullup resistor on

        int16_t adc1_1; // Leemos el ADC, con 16 bits
        adc1_1 = ads1.readADC(1);
        Voltage = (adc1_1 * 0.1875);

        Voltage = Voltage / 126.05705;

        if (Voltage > 0)
        {
          Serial3.print(Voltage, 3);
          Serial3.print("_V");
        }
        else
        {
          Voltage = 0.000;
          Serial3.print(Voltage, 3);
          Serial3.print("_V");
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////////////
      // GUARDAR_MEDIR
      if (strstr(cadena, "MEDIR_G") != 0)
      {

        OK = 50 * 24;

        for (int REGISTROS_G = 1; REGISTROS_G < 50; REGISTROS_G++)
        {

          OK = OK - 24;

          EEPROM.get(OK, CERO_DIA);
          EEPROM.get(OK + 2, CERO_MEZ);
          EEPROM.get(OK + 4, CERO_ANO);

          EEPROM.get(OK + 6, CERO_HORA);
          EEPROM.get(OK + 8, CERO_MINUTO);
          EEPROM.get(OK + 10, CERO_SEGUNDO);

          EEPROM.get(OK + 14, VALOR_CERO);
          EEPROM.get(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.get(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.get(OK + 20, TIPO_UNO);
          EEPROM.get(OK + 22, TIPO_DOS);
          ///////////////////////////////////////////////////////////////////////

          EEPROM.put(OK + 24, CERO_DIA);
          EEPROM.put(OK + 24 + 2, CERO_MEZ);
          EEPROM.put(OK + 24 + 4, CERO_ANO);

          EEPROM.put(OK + 24 + 6, CERO_HORA);
          EEPROM.put(OK + 24 + 8, CERO_MINUTO);
          EEPROM.put(OK + 24 + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 24 + 14, VALOR_CERO);
          EEPROM.put(OK + 24 + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 24 + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 24 + 20, TIPO_UNO);
          EEPROM.put(OK + 24 + 22, TIPO_DOS);

          //                            OK= OK - 24;
        }

        DateTime FECHA = rtc.now();

        CERO_ANO = FECHA.year();
        CERO_MEZ = FECHA.month();
        CERO_DIA = FECHA.day();

        CERO_HORA = FECHA.hour();
        CERO_MINUTO = FECHA.minute();
        CERO_SEGUNDO = FECHA.second();

        if (digitalRead(LED_1) == HIGH)
        {

          VALOR_CERO = Voltage;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 1;
          TIPO_UNO = 1;
          TIPO_DOS = 1;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_2) == HIGH)
        {

          VALOR_CERO = Voltage;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 1;
          TIPO_UNO = 1;
          TIPO_DOS = 2;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_3) == HIGH)
        {

          VALOR_CERO = measured_resistance;
          UNIDAD_MEDIDAX[0] = 0;

          if (res_scale_X == 0)
          {
            UNIDAD_MEDIDAX[1] = 2;
          }

          if (res_scale_X == 1)
          {
            UNIDAD_MEDIDAX[1] = 3;
          }

          if (res_scale_X == 2)
          {
            UNIDAD_MEDIDAX[1] = 4;
          }

          TIPO_UNO = 1;
          TIPO_DOS = 3;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_4) == HIGH)
        {

          VALOR_CERO = CAPACITOR;
          UNIDAD_MEDIDAX[0] = 0;

          if (res_scale_X == 3)
          {
            UNIDAD_MEDIDAX[1] = 5;
          }

          if (res_scale_X == 4)
          {
            UNIDAD_MEDIDAX[1] = 7;
          }

          if (res_scale_X == 5)
          {
            UNIDAD_MEDIDAX[1] = 6;
          }

          TIPO_UNO = 1;
          TIPO_DOS = 4;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_5) == HIGH)
        {

          VALOR_CERO = y;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 8;
          TIPO_UNO = 1;
          TIPO_DOS = 5;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_6) == HIGH)
        {

          VALOR_CERO = TEMPERATURAX;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 9;
          TIPO_UNO = 1;
          TIPO_DOS = 6;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        /*
        Voltage
        Voltage
        measured_resistance
        CAPACITOR
        y  // CORRIENTE IN
        TEMPERATURAX

        resistance_voltage // CORRIENTE OUT
        -----------
        Voltage
        */

        Serial3.print("#GUARDADO...  ");
      }

      // GUARDAR_GENERAR
      if (strstr(cadena, "GENERAR_G") != 0)
      {

        OK = 50 * 24;

        for (int REGISTROS_G = 1; REGISTROS_G < 50; REGISTROS_G++)
        {
          OK = OK - 24;
          EEPROM.get(OK, CERO_DIA);
          EEPROM.get((OK + 2), CERO_MEZ);
          EEPROM.get((OK + 4), CERO_ANO);

          EEPROM.get((OK + 6), CERO_HORA);
          EEPROM.get((OK + 8), CERO_MINUTO);
          EEPROM.get(OK + 10, CERO_SEGUNDO);

          EEPROM.get((OK + 14), VALOR_CERO);
          EEPROM.get((OK + 18), UNIDAD_MEDIDAX[0]);
          EEPROM.get((OK + 19), UNIDAD_MEDIDAX[1]);

          EEPROM.get((OK + 20), TIPO_UNO);
          EEPROM.get((OK + 22), TIPO_DOS);
          ///////////////////////////////////////////////////////////////////////

          EEPROM.put((OK + 24), CERO_DIA);
          EEPROM.put((OK + 24 + 2), CERO_MEZ);
          EEPROM.put((OK + 24 + 4), CERO_ANO);

          EEPROM.put((OK + 24 + 6), CERO_HORA);
          EEPROM.put((OK + 24 + 8), CERO_MINUTO);
          EEPROM.put((OK + 24 + 10), CERO_SEGUNDO);

          EEPROM.put((OK + 24 + 14), VALOR_CERO);
          EEPROM.put((OK + 24 + 18), UNIDAD_MEDIDAX[0]);
          EEPROM.put((OK + 24 + 19), UNIDAD_MEDIDAX[1]);

          EEPROM.put((OK + 24 + 20), TIPO_UNO);
          EEPROM.put((OK + 24 + 22), TIPO_DOS);

          //                            OK = OK - 24;
        }

        DateTime FECHA = rtc.now();

        CERO_ANO = FECHA.year();
        CERO_MEZ = FECHA.month();
        CERO_DIA = FECHA.day();

        CERO_HORA = FECHA.hour();
        CERO_MINUTO = FECHA.minute();
        CERO_SEGUNDO = FECHA.second();

        if (digitalRead(LED_7) == HIGH)
        {

          VALOR_CERO = resistance_voltage;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 8;
          TIPO_UNO = 2;
          TIPO_DOS = 7;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_8) == HIGH)
        {

          VALOR_CERO = 0.0;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 0;
          TIPO_UNO = 0;
          TIPO_DOS = 0;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        if (digitalRead(LED_9) == HIGH)
        {

          VALOR_CERO = Voltage;
          UNIDAD_MEDIDAX[0] = 0;
          UNIDAD_MEDIDAX[1] = 1;
          TIPO_UNO = 2;
          TIPO_DOS = 9;

          EEPROM.put(OK, CERO_DIA);
          EEPROM.put(OK + 2, CERO_MEZ);
          EEPROM.put(OK + 4, CERO_ANO);

          EEPROM.put(OK + 6, CERO_HORA);
          EEPROM.put(OK + 8, CERO_MINUTO);
          EEPROM.put(OK + 10, CERO_SEGUNDO);

          EEPROM.put(OK + 14, VALOR_CERO);
          EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
          EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

          EEPROM.put(OK + 20, TIPO_UNO);
          EEPROM.put(OK + 22, TIPO_DOS);
        }

        Serial3.print("=GUARDADO...  ");
      }
FDSAFDSAFDSAFASDFDSA
      /*
      ///////////////////////////////////////////////////////////////////////////////////////////
            //VER_EEPROM
            if(strstr(cadena,"VER_EEPROM")!=0)      {

                                                      for (int ts = 1 ; ts < 51 ; ts++ )  {

                                                              VALORES_A_BUSCAR = ts * 24;



                                                              EEPROM.get( VALORES_A_BUSCAR,V_DIA);
                                                              EEPROM.get( VALORES_A_BUSCAR+2,V_MEZ);
                                                              EEPROM.get( VALORES_A_BUSCAR+4,V_ANO);

                                                              EEPROM.get( VALORES_A_BUSCAR+6,V_HORA);
                                                              EEPROM.get( VALORES_A_BUSCAR+8,V_MINUTO);
                                                              EEPROM.get( VALORES_A_BUSCAR+10,V_SEGUNDO);

                                                              EEPROM.get( VALORES_A_BUSCAR+14,V_VALOR_MEDIDO);
                                                              EEPROM.get( VALORES_A_BUSCAR+18,V_UNIDAD_MEDIDA[0]);
                                                              EEPROM.get( VALORES_A_BUSCAR+19,V_UNIDAD_MEDIDA[1]);

                                                              EEPROM.get( VALORES_A_BUSCAR+20,V_TIPO_0);
                                                              EEPROM.get( VALORES_A_BUSCAR+22,V_TIPO_1);

                                                              Serial3.print(ts);
                                                              Serial3.print(") ");

                                                              Serial3.print(V_DIA);
                                                              Serial3.print("/");
                                                              Serial3.print(V_MEZ);
                                                              Serial3.print("/");
                                                              Serial3.print(V_ANO-2000);

                                                              Serial3.print(" ");

                                                              Serial3.print(V_HORA);
                                                              Serial3.print(":");
                                                              Serial3.print(V_MINUTO);
                                                              Serial3.print(":");
                                                              Serial3.print(V_SEGUNDO);

                                                              Serial3.print(" ");

                                                              Serial3.print(V_VALOR_MEDIDO,3);
                                                              Serial3.print("_");
                                                      //        lcd.print(V_UNIDAD_MEDIDA[0]);


                                                              if (V_UNIDAD_MEDIDA[1] == 0  ){
                                                              V_TIPO_UNIDAD = "--- ";
                                                              }
                                                              if (V_UNIDAD_MEDIDA[1] > 10 ){
                                                              V_TIPO_UNIDAD = "--- ";
                                                              }



                                                              if (V_UNIDAD_MEDIDA[1] == 1 ){
                                                              V_TIPO_UNIDAD = "Voltaje ";
                                                              }

                                                              if (V_UNIDAD_MEDIDA[1] == 2 ){
                                                              V_TIPO_UNIDAD = "Ohm ";
                                                              }
                                                              if (V_UNIDAD_MEDIDA[1] == 3 ){
                                                              V_TIPO_UNIDAD = "Kohm ";
                                                              }
                                                              if (V_UNIDAD_MEDIDA[1] == 4 ){
                                                              V_TIPO_UNIDAD = "Mohm ";
                                                              }


                                                              if (V_UNIDAD_MEDIDA[1] == 5 ){
                                                              V_TIPO_UNIDAD = "uf ";
                                                              }
                                                              if (V_UNIDAD_MEDIDA[1] == 6 ){
                                                              V_TIPO_UNIDAD = "nf ";
                                                              }
                                                              if (V_UNIDAD_MEDIDA[1] == 7 ){
                                                              V_TIPO_UNIDAD = "pf ";
                                                              }



                                                              if (V_UNIDAD_MEDIDA[1] == 8 ){
                                                              V_TIPO_UNIDAD = "mA ";
                                                              }

                                                              if (V_UNIDAD_MEDIDA[1] == 9 ){
                                                              V_TIPO_UNIDAD = "*C ";
                                                              }



                                                              if (V_UNIDAD_MEDIDA[1] == 10 ){
                                                              V_TIPO_UNIDAD = "--- ";
                                                              }








                                                              Serial3.print(V_TIPO_UNIDAD);


                                                              if (V_TIPO_0 == 0 ){
                                                              V_TIPO_A = "VACIO ";
                                                              }

                                                              if (V_TIPO_0 == 1 ){
                                                              V_TIPO_A = "MEDICION ";
                                                              }

                                                              if (V_TIPO_0 == 2 ){
                                                              V_TIPO_A = "GENERAR ";
                                                              }

                                                              if (V_TIPO_0 > 2 ){
                                                              V_TIPO_A = "VACIO ";
                                                              }


                                                              Serial3.print(V_TIPO_A);


                                                                    if (V_TIPO_1 == 0 ){
                                                                    V_TIPO_B = "VACIO ";
                                                                    }

                                                                    if (V_TIPO_1 == 1 ){
                                                                    V_TIPO_B = "Volt. 0-100V " ;
                                                                    }
                                                                    if (V_TIPO_1 == 2 ){
                                                                    V_TIPO_B = "volt. 0-30V ";
                                                                    }
                                                                    if (V_TIPO_1 == 3 ){
                                                                    V_TIPO_B = "Resistencia ";
                                                                    }
                                                                    if (V_TIPO_1 == 4 ){
                                                                    V_TIPO_B = "Capacitancia ";
                                                                    }
                                                                    if (V_TIPO_1 == 5 ){
                                                                    V_TIPO_B = "Corriente 0-20mA IN ";
                                                                    }
                                                                    if (V_TIPO_1 == 6 ){
                                                                    V_TIPO_B = "TERMOCUPLA TIPO J ";
                                                                    }



                                                                    if (V_TIPO_1 == 7 ){
                                                                    V_TIPO_B = "Corriente 0-20mA OUT ";
                                                                    }
                                                                    if (V_TIPO_1 == 8 ){
                                                                    V_TIPO_B = "---- ";
                                                                    }
                                                                    if (V_TIPO_1 == 9 ){
                                                                    V_TIPO_B = "VOLTAJE 0-35V SALIDA ";
                                                                    }

                                                                    if (V_TIPO_1 > 9 ){
                                                                    V_TIPO_B = "VACIO ";
                                                                    }


                                                              Serial3.print(V_TIPO_B);
                                                              Serial3.print(";");

                                                      }

            }
      */

      // VER_EEPROM
      if (strstr(cadena, "VER_DATO") != 0)
      {

        for (int ts = 1; ts < 51; ts++)
        {

          VALORES_A_BUSCAR = ts * 24;

          EEPROM.get(VALORES_A_BUSCAR, V_DIA);
          EEPROM.get(VALORES_A_BUSCAR + 2, V_MEZ);
          EEPROM.get(VALORES_A_BUSCAR + 4, V_ANO);

          EEPROM.get(VALORES_A_BUSCAR + 6, V_HORA);
          EEPROM.get(VALORES_A_BUSCAR + 8, V_MINUTO);
          EEPROM.get(VALORES_A_BUSCAR + 10, V_SEGUNDO);

          EEPROM.get(VALORES_A_BUSCAR + 14, V_VALOR_MEDIDO);
          EEPROM.get(VALORES_A_BUSCAR + 18, V_UNIDAD_MEDIDA[0]);
          EEPROM.get(VALORES_A_BUSCAR + 19, V_UNIDAD_MEDIDA[1]);

          EEPROM.get(VALORES_A_BUSCAR + 20, V_TIPO_0);
          EEPROM.get(VALORES_A_BUSCAR + 22, V_TIPO_1);

          // ENVIAR_DATO = String(ts) + ") " + String(V_DIA) + "/" + String(V_MEZ) + "/" + (V_ANO-2000) + " " +  V_HORA + ":" + V_MINUTO + ":" + V_SEGUNDO + " " + String(V_VALOR_MEDIDO,3) ;

          // Serial3.print(ENVIAR_DATO );

          // Serial3.print("_");
          //        lcd.print(V_UNIDAD_MEDIDA[0]);

          if (V_UNIDAD_MEDIDA[1] == 0)
          {
            V_TIPO_UNIDAD = "--- ";
          }
          if (V_UNIDAD_MEDIDA[1] > 10)
          {
            V_TIPO_UNIDAD = "--- ";
          }

          if (V_UNIDAD_MEDIDA[1] == 1)
          {
            V_TIPO_UNIDAD = "Voltaje ";
          }

          if (V_UNIDAD_MEDIDA[1] == 2)
          {
            V_TIPO_UNIDAD = "Ohm ";
          }
          if (V_UNIDAD_MEDIDA[1] == 3)
          {
            V_TIPO_UNIDAD = "Kohm ";
          }
          if (V_UNIDAD_MEDIDA[1] == 4)
          {
            V_TIPO_UNIDAD = "Mohm ";
          }

          if (V_UNIDAD_MEDIDA[1] == 5)
          {
            V_TIPO_UNIDAD = "uf ";
          }
          if (V_UNIDAD_MEDIDA[1] == 6)
          {
            V_TIPO_UNIDAD = "nf ";
          }
          if (V_UNIDAD_MEDIDA[1] == 7)
          {
            V_TIPO_UNIDAD = "pf ";
          }

          if (V_UNIDAD_MEDIDA[1] == 8)
          {
            V_TIPO_UNIDAD = "mA ";
          }

          if (V_UNIDAD_MEDIDA[1] == 9)
          {
            V_TIPO_UNIDAD = "*C ";
          }

          if (V_UNIDAD_MEDIDA[1] == 10)
          {
            V_TIPO_UNIDAD = "--- ";
          }

          // Serial3.print(V_TIPO_UNIDAD);

          if (V_TIPO_0 == 0)
          {
            V_TIPO_A = "VACIO ";
          }

          if (V_TIPO_0 == 1)
          {
            V_TIPO_A = "MEDICION ";
          }

          if (V_TIPO_0 == 2)
          {
            V_TIPO_A = "GENERAR ";
          }

          if (V_TIPO_0 > 2)
          {
            V_TIPO_A = "VACIO ";
          }

          // Serial3.print(V_TIPO_A);

          if (V_TIPO_1 == 0)
          {
            V_TIPO_B = "VACIO";
          }

          if (V_TIPO_1 == 1)
          {
            V_TIPO_B = "Volt._0-100V";
          }
          if (V_TIPO_1 == 2)
          {
            V_TIPO_B = "volt._0-30V";
          }
          if (V_TIPO_1 == 3)
          {
            V_TIPO_B = "Resistencia";
          }
          if (V_TIPO_1 == 4)
          {
            V_TIPO_B = "Capacitancia";
          }
          if (V_TIPO_1 == 5)
          {
            V_TIPO_B = "Corriente_0-20mA_IN";
          }
          if (V_TIPO_1 == 6)
          {
            V_TIPO_B = "TERMOCUPLA_TIPO_J";
          }

          if (V_TIPO_1 == 7)
          {
            V_TIPO_B = "Corriente_0-20mA_OUT";
          }
          if (V_TIPO_1 == 8)
          {
            V_TIPO_B = "---- ";
          }
          if (V_TIPO_1 == 9)
          {
            V_TIPO_B = "VOLTAJE_0-35V_SALIDA";
          }

          if (V_TIPO_1 > 9)
          {
            V_TIPO_B = "VACIO";
          }

          // Serial3.print(V_TIPO_B);

          // Serial3.print(";");
          ENVIAR_DATO = "#" + String(ts) + ") " + String(V_DIA) + "/" + String(V_MEZ) + "/" +
                        (V_ANO - 2000) + " " + V_HORA + ":" + V_MINUTO + ":" + V_SEGUNDO + " " + String(V_VALOR_MEDIDO, 3) +
                        "_" + V_TIPO_UNIDAD + V_TIPO_A + V_TIPO_B + ";";
          Serial.print(ENVIAR_DATO);
          Serial3.print(ENVIAR_DATO);
        }
      }

      ///////////////////////////////////////////////////////////////////////////////////////////
      if (strstr(cadena, "SCALA_R") != 0)
      {
        res_scale_BLU++;
        if (res_scale_BLU == 3)
        {
          res_scale_BLU = 0;
        }
      }

      //      Serial3.write(13); //Enviamos un retorno de carro de la app. La app ya crea una línea nueva
      //      Serial3.write(10); //Enviamos un retorno de carro de la app. La app ya crea una línea nueva
      clean(); // Ejecutamos la función clean() para limpiar el array
    }
  }
}

void clean()
{
  //  i = 0;

  for (int cl = 0; cl <= i; cl++) //
  {
    cadena[cl] = 0;
  }
  i = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PRIMERA_VEZ_BLUTU()
{
  if (BANDERA_PRIMERA_VEZ_BLUTU == 1)
  {
    BANDERA_PRIMERA_VEZ_BLUTU = 0;
    BANDERA_PRIMERA_VEZ_PROGRAMA = 1;
    i = 0;
    clean(); // Ejecutamos la función clean() para limpiar el array

    Serial3.begin(9600);
    Serial.begin(9600);
    res_scale_BLU == 0;

    FRAME_REGISTROS = 1;

    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("CONECTANDO BLUTU...");
    delay(100);

    MCP4725_value = 0;
    MUESTREO = 0;
    MCP4725.setVoltage(MCP4725_value, false);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void PRIMERA_VEZ_PROGRAMA()
{
  if (BANDERA_PRIMERA_VEZ_PROGRAMA == 1)
  {
    BANDERA_PRIMERA_VEZ_BLUTU = 1;
    BANDERA_PRIMERA_VEZ_PROGRAMA = 0;

    FRAME_REGISTROS = 0;

    MENU_GENERAR = 1;
    MENU_MEDICION = 1;

    res_scale = 0;

    MENU_INDICE_REGISTROS = 1;
    FRAME_REGISTROS = 0;

    MCP4725_value = 0;
    MUESTREO = 0;
    MCP4725.setVoltage(MCP4725_value, false);

    i = 0;

    clean(); // Ejecutamos la función clean() para limpiar el array

    Serial.end();
    Serial3.end();

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("MEDICION            ");
    lcd.setCursor(0, 1);
    lcd.print("     000.000 mv     ");
    lcd.setCursor(0, 2);
    lcd.print("GENERAR             ");
    lcd.setCursor(0, 3);
    lcd.print("     000.000 mv     ");
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void VER_DATOS_GUARDADOS()
{

  while (digitalRead(B_DATOS_GUARDADOS) == 0)
  {
    delay(50);
  }
  delay(50);

  FRAME_REGISTROS++;
  if (FRAME_REGISTROS == 2)
  {
    FRAME_REGISTROS = 0;
  }

  if (FRAME_REGISTROS == 1)
  {

    VALORES_A_BUSCAR = MENU_INDICE_REGISTROS * 24;

    lcd.clear();

    EEPROM.get(VALORES_A_BUSCAR, V_DIA);
    EEPROM.get(VALORES_A_BUSCAR + 2, V_MEZ);
    EEPROM.get(VALORES_A_BUSCAR + 4, V_ANO);

    EEPROM.get(VALORES_A_BUSCAR + 6, V_HORA);
    EEPROM.get(VALORES_A_BUSCAR + 8, V_MINUTO);
    EEPROM.get(VALORES_A_BUSCAR + 10, V_SEGUNDO);

    EEPROM.get(VALORES_A_BUSCAR + 14, V_VALOR_MEDIDO);
    EEPROM.get(VALORES_A_BUSCAR + 18, V_UNIDAD_MEDIDA[0]);
    EEPROM.get(VALORES_A_BUSCAR + 19, V_UNIDAD_MEDIDA[1]);

    EEPROM.get(VALORES_A_BUSCAR + 20, V_TIPO_0);
    EEPROM.get(VALORES_A_BUSCAR + 22, V_TIPO_1);

    lcd.setCursor(0, 0);
    lcd.print(MENU_INDICE_REGISTROS);
    lcd.print(") ");

    lcd.setCursor(3, 0);
    lcd.print(V_DIA);
    lcd.print("/");
    lcd.print(V_MEZ);
    lcd.print("/");
    lcd.print(V_ANO - 2000);

    lcd.print(" ");

    lcd.print(V_HORA);
    lcd.print(":");
    lcd.print(V_MINUTO);
    lcd.print(":");
    lcd.print(V_SEGUNDO);

    lcd.setCursor(3, 1);
    lcd.print(V_VALOR_MEDIDO, 3);
    lcd.print("_");
    //        lcd.print(V_UNIDAD_MEDIDA[0]);

    if (V_UNIDAD_MEDIDA[1] == 0)
    {
      V_TIPO_UNIDAD = "---       ";
    }
    if (V_UNIDAD_MEDIDA[1] > 10)
    {
      V_TIPO_UNIDAD = "---       ";
    }

    if (V_UNIDAD_MEDIDA[1] == 1)
    {
      V_TIPO_UNIDAD = "Voltaje   ";
    }

    if (V_UNIDAD_MEDIDA[1] == 2)
    {
      V_TIPO_UNIDAD = "Ohm       ";
    }
    if (V_UNIDAD_MEDIDA[1] == 3)
    {
      V_TIPO_UNIDAD = "Kohm      ";
    }
    if (V_UNIDAD_MEDIDA[1] == 4)
    {
      V_TIPO_UNIDAD = "Mohm      ";
    }

    if (V_UNIDAD_MEDIDA[1] == 5)
    {
      V_TIPO_UNIDAD = "uf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 6)
    {
      V_TIPO_UNIDAD = "nf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 7)
    {
      V_TIPO_UNIDAD = "pf        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 8)
    {
      V_TIPO_UNIDAD = "mA        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 9)
    {
      V_TIPO_UNIDAD = "*C        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 10)
    {
      V_TIPO_UNIDAD = "----------";
    }

    lcd.print(V_TIPO_UNIDAD);

    if (V_TIPO_0 == 0)
    {
      V_TIPO_A = "VACIO               ";
    }

    if (V_TIPO_0 == 1)
    {
      V_TIPO_A = "MEDICION             ";
    }

    if (V_TIPO_0 == 2)
    {
      V_TIPO_A = "GENERAR            ";
    }

    if (V_TIPO_0 > 2)
    {
      V_TIPO_A = "VACIO               ";
    }

    lcd.setCursor(0, 2);
    lcd.print(V_TIPO_A);

    if (V_TIPO_1 == 0)
    {
      V_TIPO_B = "VACIO               ";
    }

    if (V_TIPO_1 == 1)
    {
      V_TIPO_B = "Volt. 0-100V        ";
    }
    if (V_TIPO_1 == 2)
    {
      V_TIPO_B = "volt. 0-30V         ";
    }
    if (V_TIPO_1 == 3)
    {
      V_TIPO_B = "Resistencia         ";
    }
    if (V_TIPO_1 == 4)
    {
      V_TIPO_B = "Capacitancia        ";
    }
    if (V_TIPO_1 == 5)
    {
      V_TIPO_B = "Corriente 0-20mA IN ";
    }
    if (V_TIPO_1 == 6)
    {
      V_TIPO_B = "TERMOCUPLA TIPO J   ";
    }

    if (V_TIPO_1 == 7)
    {
      V_TIPO_B = "Corriente 0-20mA OUT";
    }
    if (V_TIPO_1 == 8)
    {
      V_TIPO_B = "--------------------";
    }
    if (V_TIPO_1 == 9)
    {
      V_TIPO_B = "VOLTAJE 0-35V SALIDA";
    }

    if (V_TIPO_1 > 9)
    {
      V_TIPO_B = "VACIO               ";
    }

    lcd.setCursor(0, 3);
    lcd.print(V_TIPO_B);

    goto salir_x;
  }

  if (FRAME_REGISTROS == 0)
  {

    lcd.setCursor(0, 0);
    lcd.print("MEDICION            ");
    lcd.setCursor(0, 1);
    lcd.print("     000.000 mv     ");
    lcd.setCursor(0, 2);
    lcd.print("GENERAR             ");
    lcd.setCursor(0, 3);
    lcd.print("     000.000 mv     ");

    //  FRAME_REGISTROS++;
  }

salir_x:
  int jojo = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SCALA_MEDICION()
{

  if (MENU_MEDICION == 3)
  {

    while (digitalRead(B_SCALA_MEDICION) == 0)
    {
      delay(100);
    }
    delay(100);
    res_scale++;
    if (res_scale == 3)
    {
      res_scale = 0;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void GUARDAR_MEDICION()
{
  while (digitalRead(B_GUARDAR_MEDICION) == 0)
  {
    delay(50);
  }
  delay(50);

  if (FRAME_REGISTROS == 0)
  {

    OK = 50 * 24;

    for (int REGISTROS_G = 1; REGISTROS_G < 50; REGISTROS_G++)
    {

      OK = OK - 24;

      EEPROM.get(OK, CERO_DIA);
      EEPROM.get(OK + 2, CERO_MEZ);
      EEPROM.get(OK + 4, CERO_ANO);

      EEPROM.get(OK + 6, CERO_HORA);
      EEPROM.get(OK + 8, CERO_MINUTO);
      EEPROM.get(OK + 10, CERO_SEGUNDO);

      EEPROM.get(OK + 14, VALOR_CERO);
      EEPROM.get(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.get(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.get(OK + 20, TIPO_UNO);
      EEPROM.get(OK + 22, TIPO_DOS);
      ///////////////////////////////////////////////////////////////////////

      EEPROM.put(OK + 24, CERO_DIA);
      EEPROM.put(OK + 24 + 2, CERO_MEZ);
      EEPROM.put(OK + 24 + 4, CERO_ANO);

      EEPROM.put(OK + 24 + 6, CERO_HORA);
      EEPROM.put(OK + 24 + 8, CERO_MINUTO);
      EEPROM.put(OK + 24 + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 24 + 14, VALOR_CERO);
      EEPROM.put(OK + 24 + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 24 + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 24 + 20, TIPO_UNO);
      EEPROM.put(OK + 24 + 22, TIPO_DOS);

      //                            OK= OK - 24;
    }

    DateTime FECHA = rtc.now();

    CERO_ANO = FECHA.year();
    CERO_MEZ = FECHA.month();
    CERO_DIA = FECHA.day();

    CERO_HORA = FECHA.hour();
    CERO_MINUTO = FECHA.minute();
    CERO_SEGUNDO = FECHA.second();

    if (MENU_MEDICION == 1)
    {

      VALOR_CERO = Voltage;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 1;
      TIPO_UNO = 1;
      TIPO_DOS = 1;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_MEDICION == 2)
    {

      VALOR_CERO = Voltage;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 1;
      TIPO_UNO = 1;
      TIPO_DOS = 2;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_MEDICION == 3)
    {

      VALOR_CERO = measured_resistance;
      UNIDAD_MEDIDAX[0] = 0;

      if (res_scale_X == 0)
      {
        UNIDAD_MEDIDAX[1] = 2;
      }

      if (res_scale_X == 1)
      {
        UNIDAD_MEDIDAX[1] = 3;
      }

      if (res_scale_X == 2)
      {
        UNIDAD_MEDIDAX[1] = 4;
      }

      TIPO_UNO = 1;
      TIPO_DOS = 3;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_MEDICION == 4)
    {

      VALOR_CERO = CAPACITOR;
      UNIDAD_MEDIDAX[0] = 0;

      if (res_scale_X == 3)
      {
        UNIDAD_MEDIDAX[1] = 5;
      }

      if (res_scale_X == 4)
      {
        UNIDAD_MEDIDAX[1] = 7;
      }

      if (res_scale_X == 5)
      {
        UNIDAD_MEDIDAX[1] = 6;
      }

      TIPO_UNO = 1;
      TIPO_DOS = 4;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_MEDICION == 5)
    {

      VALOR_CERO = y;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 8;
      TIPO_UNO = 1;
      TIPO_DOS = 5;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_MEDICION == 6)
    {

      VALOR_CERO = TEMPERATURAX;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 9;
      TIPO_UNO = 1;
      TIPO_DOS = 6;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    /*
    Voltage
    Voltage
    measured_resistance
    CAPACITOR
    y  // CORRIENTE IN
    TEMPERATURAX

    resistance_voltage // CORRIENTE OUT
    -----------
    Voltage
    */

    lcd.setCursor(0, 1);
    lcd.print("GUARDADO...        ");
    delay(1000);
  }
}

void GUARDAR_GENERAR()
{
  while (digitalRead(B_GUARDAR_GENERAR) == 0)
  {
    delay(50);
  }
  delay(50);

  if (FRAME_REGISTROS == 0)
  {

    OK = 50 * 24;

    for (int REGISTROS_G = 1; REGISTROS_G < 50; REGISTROS_G++)
    {
      OK = OK - 24;
      EEPROM.get(OK, CERO_DIA);
      EEPROM.get((OK + 2), CERO_MEZ);
      EEPROM.get((OK + 4), CERO_ANO);

      EEPROM.get((OK + 6), CERO_HORA);
      EEPROM.get((OK + 8), CERO_MINUTO);
      EEPROM.get(OK + 10, CERO_SEGUNDO);

      EEPROM.get((OK + 14), VALOR_CERO);
      EEPROM.get((OK + 18), UNIDAD_MEDIDAX[0]);
      EEPROM.get((OK + 19), UNIDAD_MEDIDAX[1]);

      EEPROM.get((OK + 20), TIPO_UNO);
      EEPROM.get((OK + 22), TIPO_DOS);
      ///////////////////////////////////////////////////////////////////////

      EEPROM.put((OK + 24), CERO_DIA);
      EEPROM.put((OK + 24 + 2), CERO_MEZ);
      EEPROM.put((OK + 24 + 4), CERO_ANO);

      EEPROM.put((OK + 24 + 6), CERO_HORA);
      EEPROM.put((OK + 24 + 8), CERO_MINUTO);
      EEPROM.put((OK + 24 + 10), CERO_SEGUNDO);

      EEPROM.put((OK + 24 + 14), VALOR_CERO);
      EEPROM.put((OK + 24 + 18), UNIDAD_MEDIDAX[0]);
      EEPROM.put((OK + 24 + 19), UNIDAD_MEDIDAX[1]);

      EEPROM.put((OK + 24 + 20), TIPO_UNO);
      EEPROM.put((OK + 24 + 22), TIPO_DOS);

      //                            OK = OK - 24;
    }

    DateTime FECHA = rtc.now();

    CERO_ANO = FECHA.year();
    CERO_MEZ = FECHA.month();
    CERO_DIA = FECHA.day();

    CERO_HORA = FECHA.hour();
    CERO_MINUTO = FECHA.minute();
    CERO_SEGUNDO = FECHA.second();

    if (MENU_GENERAR == 1)
    {

      VALOR_CERO = resistance_voltage;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 8;
      TIPO_UNO = 2;
      TIPO_DOS = 7;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_GENERAR == 2)
    {

      VALOR_CERO = 0.0;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 0;
      TIPO_UNO = 0;
      TIPO_DOS = 0;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    if (MENU_GENERAR == 3)
    {

      VALOR_CERO = Voltage;
      UNIDAD_MEDIDAX[0] = 0;
      UNIDAD_MEDIDAX[1] = 1;
      TIPO_UNO = 2;
      TIPO_DOS = 9;

      EEPROM.put(OK, CERO_DIA);
      EEPROM.put(OK + 2, CERO_MEZ);
      EEPROM.put(OK + 4, CERO_ANO);

      EEPROM.put(OK + 6, CERO_HORA);
      EEPROM.put(OK + 8, CERO_MINUTO);
      EEPROM.put(OK + 10, CERO_SEGUNDO);

      EEPROM.put(OK + 14, VALOR_CERO);
      EEPROM.put(OK + 18, UNIDAD_MEDIDAX[0]);
      EEPROM.put(OK + 19, UNIDAD_MEDIDAX[1]);

      EEPROM.put(OK + 20, TIPO_UNO);
      EEPROM.put(OK + 22, TIPO_DOS);
    }

    lcd.setCursor(0, 3);
    lcd.print("GUARDADO...        ");
    delay(1000);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// MEDICION ARRIBA
void INCREMENTAR_MENU_MEDICION()
{
  while (digitalRead(B_SELEC_MEDICION) == 0)
  {
    delay(50);
  }
  delay(50);

  if (FRAME_REGISTROS == 0)
  {

    MENU_MEDICION++;
    if (MENU_MEDICION == 7)
    {
      MENU_MEDICION = 1;
    }
  }

  if (FRAME_REGISTROS == 1)
  {
    MENU_INDICE_REGISTROS++;
    MENU_INDICE_REGISTROS = min(50, max(1, MENU_INDICE_REGISTROS));

    VALORES_A_BUSCAR = MENU_INDICE_REGISTROS * 24;

    lcd.clear();

    EEPROM.get(VALORES_A_BUSCAR, V_DIA);
    EEPROM.get(VALORES_A_BUSCAR + 2, V_MEZ);
    EEPROM.get(VALORES_A_BUSCAR + 4, V_ANO);

    EEPROM.get(VALORES_A_BUSCAR + 6, V_HORA);
    EEPROM.get(VALORES_A_BUSCAR + 8, V_MINUTO);
    EEPROM.get(VALORES_A_BUSCAR + 10, V_SEGUNDO);

    EEPROM.get(VALORES_A_BUSCAR + 14, V_VALOR_MEDIDO);
    EEPROM.get(VALORES_A_BUSCAR + 18, V_UNIDAD_MEDIDA[0]);
    EEPROM.get(VALORES_A_BUSCAR + 19, V_UNIDAD_MEDIDA[1]);

    EEPROM.get(VALORES_A_BUSCAR + 20, V_TIPO_0);
    EEPROM.get(VALORES_A_BUSCAR + 22, V_TIPO_1);

    lcd.setCursor(0, 0);
    lcd.print(MENU_INDICE_REGISTROS);
    lcd.print(") ");

    lcd.setCursor(3, 0);
    lcd.print(V_DIA);
    lcd.print("/");
    lcd.print(V_MEZ);
    lcd.print("/");
    lcd.print(V_ANO - 2000);

    lcd.print(" ");

    lcd.print(V_HORA);
    lcd.print(":");
    lcd.print(V_MINUTO);
    lcd.print(":");
    lcd.print(V_SEGUNDO);

    lcd.setCursor(3, 1);
    lcd.print(V_VALOR_MEDIDO, 3);
    lcd.print("_");

    if (V_UNIDAD_MEDIDA[1] == 0)
    {
      V_TIPO_UNIDAD = "---       ";
    }
    if (V_UNIDAD_MEDIDA[1] > 10)
    {
      V_TIPO_UNIDAD = "---       ";
    }

    if (V_UNIDAD_MEDIDA[1] == 1)
    {
      V_TIPO_UNIDAD = "Voltaje   ";
    }

    if (V_UNIDAD_MEDIDA[1] == 2)
    {
      V_TIPO_UNIDAD = "Ohm       ";
    }
    if (V_UNIDAD_MEDIDA[1] == 3)
    {
      V_TIPO_UNIDAD = "Kohm      ";
    }
    if (V_UNIDAD_MEDIDA[1] == 4)
    {
      V_TIPO_UNIDAD = "Mohm      ";
    }

    if (V_UNIDAD_MEDIDA[1] == 5)
    {
      V_TIPO_UNIDAD = "uf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 6)
    {
      V_TIPO_UNIDAD = "nf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 7)
    {
      V_TIPO_UNIDAD = "pf        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 8)
    {
      V_TIPO_UNIDAD = "mA        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 9)
    {
      V_TIPO_UNIDAD = "*C        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 10)
    {
      V_TIPO_UNIDAD = "----------";
    }

    lcd.print(V_TIPO_UNIDAD);

    if (V_TIPO_0 == 0)
    {
      V_TIPO_A = "VACIO               ";
    }
    if (V_TIPO_0 > 2)
    {
      V_TIPO_A = "VACIO               ";
    }

    if (V_TIPO_0 == 1)
    {
      V_TIPO_A = "MEDICION            ";
    }

    if (V_TIPO_0 == 2)
    {
      V_TIPO_A = "GENERAR             ";
    }

    lcd.setCursor(0, 2);
    lcd.print(V_TIPO_A);

    if (V_TIPO_1 == 0)
    {
      V_TIPO_B = "VACIO               ";
    }
    if (V_TIPO_1 > 9)
    {
      V_TIPO_B = "VACIO               ";
    }

    if (V_TIPO_1 == 1)
    {
      V_TIPO_B = "Volt. 0-100V        ";
    }
    if (V_TIPO_1 == 2)
    {
      V_TIPO_B = "volt. 0-30V         ";
    }
    if (V_TIPO_1 == 3)
    {
      V_TIPO_B = "Resistencia         ";
    }
    if (V_TIPO_1 == 4)
    {
      V_TIPO_B = "Capacitancia        ";
    }
    if (V_TIPO_1 == 5)
    {
      V_TIPO_B = "Corriente 0-20mA IN ";
    }
    if (V_TIPO_1 == 6)
    {
      V_TIPO_B = "TERMOCUPLA TIPO J   ";
    }

    if (V_TIPO_1 == 7)
    {
      V_TIPO_B = "Corriente 0-20mA OUT";
    }
    if (V_TIPO_1 == 8)
    {
      V_TIPO_B = "SEÑAL TERMOCUPLA OUT";
    }
    if (V_TIPO_1 == 9)
    {
      V_TIPO_B = "VOLTAJE 0-35V SALIDA";
    }

    lcd.setCursor(0, 3);
    lcd.print(V_TIPO_B);
  }
}

// GENERAR ABAJO
void INCREMENTAR_MENU_GENERAR()
{
  while (digitalRead(B_SELEC_GENERAR) == 0)
  {
    delay(50);
  }
  delay(50);

  if (FRAME_REGISTROS == 0)
  {

    MENU_GENERAR++;
    if (MENU_GENERAR == 2)
    {
      MCP4725_value = 0;
      MUESTREO = 0;
      MCP4725.setVoltage(MCP4725_value, false);
    }

    if (MENU_GENERAR == 4)
    {
      MENU_GENERAR = 1;
    }
  }

  if (FRAME_REGISTROS == 1)
  {
    MENU_INDICE_REGISTROS--;
    MENU_INDICE_REGISTROS = min(50, max(1, MENU_INDICE_REGISTROS));

    VALORES_A_BUSCAR = MENU_INDICE_REGISTROS * 24;

    lcd.clear();

    EEPROM.get(VALORES_A_BUSCAR, V_DIA);
    EEPROM.get(VALORES_A_BUSCAR + 2, V_MEZ);
    EEPROM.get(VALORES_A_BUSCAR + 4, V_ANO);

    EEPROM.get(VALORES_A_BUSCAR + 6, V_HORA);
    EEPROM.get(VALORES_A_BUSCAR + 8, V_MINUTO);
    EEPROM.get(VALORES_A_BUSCAR + 10, V_SEGUNDO);

    EEPROM.get(VALORES_A_BUSCAR + 14, V_VALOR_MEDIDO);
    EEPROM.get(VALORES_A_BUSCAR + 18, V_UNIDAD_MEDIDA[0]);
    EEPROM.get(VALORES_A_BUSCAR + 19, V_UNIDAD_MEDIDA[1]);

    EEPROM.get(VALORES_A_BUSCAR + 20, V_TIPO_0);
    EEPROM.get(VALORES_A_BUSCAR + 22, V_TIPO_1);

    lcd.setCursor(0, 0);
    lcd.print(MENU_INDICE_REGISTROS);
    lcd.print(") ");

    lcd.setCursor(3, 0);
    lcd.print(V_DIA);
    lcd.print("/");
    lcd.print(V_MEZ);
    lcd.print("/");
    lcd.print(V_ANO - 2000);

    lcd.print(" ");

    lcd.print(V_HORA);
    lcd.print(":");
    lcd.print(V_MINUTO);
    lcd.print(":");
    lcd.print(V_SEGUNDO);

    lcd.setCursor(3, 1);
    lcd.print(V_VALOR_MEDIDO, 3);
    lcd.print("_");

    if (V_UNIDAD_MEDIDA[1] == 0)
    {
      V_TIPO_UNIDAD = "---       ";
    }
    if (V_UNIDAD_MEDIDA[1] > 10)
    {
      V_TIPO_UNIDAD = "---       ";
    }

    if (V_UNIDAD_MEDIDA[1] == 1)
    {
      V_TIPO_UNIDAD = "Voltaje   ";
    }

    if (V_UNIDAD_MEDIDA[1] == 2)
    {
      V_TIPO_UNIDAD = "Ohm       ";
    }
    if (V_UNIDAD_MEDIDA[1] == 3)
    {
      V_TIPO_UNIDAD = "Kohm      ";
    }
    if (V_UNIDAD_MEDIDA[1] == 4)
    {
      V_TIPO_UNIDAD = "Mohm      ";
    }

    if (V_UNIDAD_MEDIDA[1] == 5)
    {
      V_TIPO_UNIDAD = "uf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 6)
    {
      V_TIPO_UNIDAD = "nf        ";
    }
    if (V_UNIDAD_MEDIDA[1] == 7)
    {
      V_TIPO_UNIDAD = "pf        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 8)
    {
      V_TIPO_UNIDAD = "mA        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 9)
    {
      V_TIPO_UNIDAD = "*C        ";
    }

    if (V_UNIDAD_MEDIDA[1] == 10)
    {
      V_TIPO_UNIDAD = "----------";
    }

    lcd.print(V_TIPO_UNIDAD);

    if (V_TIPO_0 == 0)
    {
      V_TIPO_A = "VACIO               ";
    }
    if (V_TIPO_0 > 3)
    {
      V_TIPO_A = "VACIO               ";
    }

    if (V_TIPO_0 == 1)
    {
      V_TIPO_A = "MEDICION            ";
    }

    if (V_TIPO_0 == 2)
    {
      V_TIPO_A = "GENERAR             ";
    }

    lcd.setCursor(0, 2);
    lcd.print(V_TIPO_A);

    if (V_TIPO_1 == 0)
    {
      V_TIPO_B = "VACIO               ";
    }

    if (V_TIPO_1 > 9)
    {
      V_TIPO_B = "VACIO               ";
    }

    if (V_TIPO_1 == 1)
    {
      V_TIPO_B = "Volt. 0-100V        ";
    }
    if (V_TIPO_1 == 2)
    {
      V_TIPO_B = "volt. 0-30V         ";
    }
    if (V_TIPO_1 == 3)
    {
      V_TIPO_B = "Resistencia         ";
    }
    if (V_TIPO_1 == 4)
    {
      V_TIPO_B = "Capacitancia        ";
    }
    if (V_TIPO_1 == 5)
    {
      V_TIPO_B = "Corriente 0-20mA IN ";
    }
    if (V_TIPO_1 == 6)
    {
      V_TIPO_B = "TERMOCUPLA TIPO J   ";
    }

    if (V_TIPO_1 == 7)
    {
      V_TIPO_B = "Corriente 0-20mA OUT";
    }
    if (V_TIPO_1 == 8)
    {
      V_TIPO_B = "--------------------";
    }
    if (V_TIPO_1 == 9)
    {
      V_TIPO_B = "VOLTAJE 0-35V SALIDA";
    }

    lcd.setCursor(0, 3);
    lcd.print(V_TIPO_B);
  }
}

void MEDIR()
{

  // MEDIR VOLTAJE 0-100V
  if (MENU_MEDICION == 1)
  {

    digitalWrite(LED_1, HIGH); // turn pullup resistor on
    digitalWrite(LED_2, LOW);  // turn pullup resistor on
    digitalWrite(LED_3, LOW);  // turn pullup resistor on
    digitalWrite(LED_4, LOW);  // turn pullup resistor on
    digitalWrite(LED_5, LOW);  // turn pullup resistor on
    digitalWrite(LED_6, LOW);  // turn pullup resistor on

    int16_t adc0_0; // Leemos el ADC, con 16 bits
    adc0_0 = ads0.readADC(0);
    Voltage = (adc0_0 * 0.1875);

    Voltage = Voltage / 40.548022;
    Voltage = Voltage + 0.7;

    if (Voltage > 0.75)
    {

      lcd.setCursor(11, 0);
      lcd.print("   0-100v");

      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(3, 1);
      lcd.print(Voltage, 3);
      lcd.print("_Voltaje  ");
      delay(100);
    }
    else
    {
      lcd.setCursor(11, 0);
      lcd.print("   0-100v");

      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(3, 1);
      lcd.print("000.000");
      Voltage = 000.000;
      lcd.print("_Voltaje  ");
      delay(100);
    }
  }

  // 0-30V
  if (MENU_MEDICION == 2)
  {

    digitalWrite(LED_1, LOW);  // turn pullup resistor on
    digitalWrite(LED_2, HIGH); // turn pullup resistor on
    digitalWrite(LED_3, LOW);  // turn pullup resistor on
    digitalWrite(LED_4, LOW);  // turn pullup resistor on
    digitalWrite(LED_5, LOW);  // turn pullup resistor on
    digitalWrite(LED_6, LOW);  // turn pullup resistor on

    int16_t adc0_1; // Leemos el ADC, con 16 bits
    adc0_1 = ads0.readADC(1);
    Voltage = (adc0_1 * 0.1875);

    Voltage = Voltage / 127.55705;
    Voltage = Voltage + 0.7;

    if (Voltage > 0.75)
    {

      lcd.setCursor(11, 0);
      lcd.print("    0-30v");

      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(3, 1);
      lcd.print(Voltage, 3);
      lcd.print("_Voltaje");
      delay(100);
    }
    else
    {
      lcd.setCursor(11, 0);
      lcd.print("    0-30v");

      lcd.setCursor(0, 1);
      lcd.print("                    ");
      lcd.setCursor(3, 1);
      lcd.print("000.000");
      Voltage = 0.000;
      lcd.print("_Voltaje ");
      delay(100);
    }
  }

  // RESISTENCIAS >4 >200 >4000
  if (MENU_MEDICION == 3)
  {

    digitalWrite(LED_1, LOW);  // turn pullup resistor on
    digitalWrite(LED_2, LOW);  // turn pullup resistor on
    digitalWrite(LED_3, HIGH); // turn pullup resistor on
    digitalWrite(LED_4, LOW);  // turn pullup resistor on
    digitalWrite(LED_5, LOW);  // turn pullup resistor on
    digitalWrite(LED_6, LOW);  // turn pullup resistor on

    //  res_scale = 0;

    if (res_scale == 0)
    {

      pinMode(res_2k, OUTPUT);
      pinMode(res_20k, INPUT);
      pinMode(res_470k, INPUT);
      digitalWrite(res_2k, LOW);

      int32_t adc1_0; // Leemos el ADC, con 16 bits
      adc1_0 = ads1.readADC(0);
      resistance_voltage = (adc1_0 * 0.1875) / 1000;

      int32_t adc1_3; // Leemos el ADC, con 16 bits
      adc1_3 = ads1.readADC(3);
      battery_voltage = ((adc1_3 * 0.1875) / 1000);

      measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_2k_value);

      measured_resistance = measured_resistance * 1000;

      if (measured_resistance < 4000)
      {

        lcd.setCursor(11, 0);
        lcd.print("0-4k_Ohms");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(measured_resistance, 3);
        lcd.print("_Ohms");

        res_scale_X = res_scale;
        delay(100);
      }
      else
      {
        lcd.setCursor(11, 0);
        lcd.print("0-4k_Ohms");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(">4000");
        res_scale_X = res_scale;
        measured_resistance = 0.000;
        delay(100);
      }
    }

    if (res_scale == 1)
    {
      pinMode(res_2k, INPUT);
      pinMode(res_20k, OUTPUT);
      pinMode(res_470k, INPUT);
      digitalWrite(res_20k, LOW);

      int16_t adc1_0; // Leemos el ADC, con 16 bits
      adc1_0 = ads1.readADC(0);
      resistance_voltage = (adc1_0 * 0.1875) / 1000;

      int16_t adc1_3; // Leemos el ADC, con 16 bits
      adc1_3 = ads1.readADC(3);
      battery_voltage = ((adc1_3 * 0.1875) / 1000);

      measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_20k_value);
      measured_resistance = measured_resistance;
      if (measured_resistance < 200)
      {
        lcd.setCursor(11, 0);
        lcd.print("  4k-200k");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(measured_resistance, 3);
        lcd.print("_Kohms");
        res_scale_X = res_scale;
        delay(100);
      }
      else
      {

        lcd.setCursor(11, 0);
        lcd.print("  4k-200k");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(">200K");
        res_scale_X = res_scale;
        measured_resistance = 0.000;
        delay(100);
      }
    }

    if (res_scale == 2)
    {
      pinMode(res_2k, INPUT);
      pinMode(res_20k, INPUT);
      pinMode(res_470k, OUTPUT);
      digitalWrite(res_470k, LOW);
      delay(100);

      int16_t adc1_0; // Leemos el ADC, con 16 bits
      adc1_0 = ads1.readADC(0);
      resistance_voltage = (adc1_0 * 0.1875) / 1000;

      int16_t adc1_3; // Leemos el ADC, con 16 bits
      adc1_3 = ads1.readADC(3);
      battery_voltage = ((adc1_3 * 0.1875) / 1000);

      measured_resistance = (battery_voltage - resistance_voltage) / (resistance_voltage / Res_470k_value);
      measured_resistance = measured_resistance / 1000;

      if (measured_resistance < 2 && measured_resistance > 0)
      {

        lcd.setCursor(11, 0);
        lcd.print(" 200k-1_M");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(measured_resistance, 3);
        lcd.print("_Mohms");
        res_scale_X = res_scale;
        delay(100);
      }

      else
      {

        lcd.setCursor(11, 0);
        lcd.print(" 200k-1_M");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(">1M");
        res_scale_X = res_scale;
        measured_resistance = 0.000;
        delay(100);
      }
    }
  }

  // CAPACITANCIA
  if (MENU_MEDICION == 4)
  {

    digitalWrite(LED_1, LOW);  // turn pullup resistor on
    digitalWrite(LED_2, LOW);  // turn pullup resistor on
    digitalWrite(LED_3, LOW);  // turn pullup resistor on
    digitalWrite(LED_4, HIGH); // turn pullup resistor on
    digitalWrite(LED_5, LOW);  // turn pullup resistor on
    digitalWrite(LED_6, LOW);  // turn pullup resistor on

    int16_t adc1_3; // Leemos el ADC, con 16 bits
    adc1_3 = ads1.readADC(3);
    battery_voltage = ((adc1_3 * 0.1875) / 1000);

    if (!cap_scale)
    {

      pinMode(cap_in_analogPin, INPUT);

      pinMode(OUT_PIN, OUTPUT);
      digitalWrite(OUT_PIN, LOW);

      pinMode(chargePin, OUTPUT);
      digitalWrite(chargePin, HIGH); // apply 5 Volts

      pinMode(dischargePin, INPUT); // dischargePin

      startTime = micros();
      while (analogRead(cap_in_analogPin) < 648)
      {
      } // end while

      elapsedTime = micros() - startTime;
      microFarads = ((float)elapsedTime / resistorValue) - 0.01097;

      if (microFarads > 1)
      {
        lcd.setCursor(11, 0);
        lcd.print("CAPACITOR");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(microFarads);
        lcd.print("_uF  ");
        CAPACITOR = microFarads;
        res_scale_X = 3;
        delay(100);
      }

      else
      {

        cap_scale = true; // We change the scale to MIN - 1uF
      }

      digitalWrite(chargePin, LOW);
      pinMode(dischargePin, OUTPUT);
      digitalWrite(dischargePin, LOW); // discharging the capacitor
      while (analogRead(cap_in_analogPin) > 0)
      {
      }                             // This while waits till the capaccitor is discharged
      pinMode(dischargePin, INPUT); // this sets the pin to high impedance

    } // end of upper scale

    if (cap_scale)
    {
      pinMode(dischargePin, INPUT);
      pinMode(chargePin, INPUT);
      pinMode(OUT_PIN, OUTPUT);
      pinMode(cap_in_analogPin, INPUT);

      digitalWrite(OUT_PIN, HIGH);
      int val = analogRead(cap_in_analogPin);
      digitalWrite(OUT_PIN, LOW);

      if (val < 1000)
      {
        pinMode(cap_in_analogPin, OUTPUT);
        float capacitance = (float)val * IN_CAP_TO_GND / (float)(MAX_ADC_VALUE - val);

        lcd.setCursor(11, 0);
        lcd.print("CAPACITOR");

        lcd.setCursor(0, 1);
        lcd.print("                    ");
        lcd.setCursor(6, 1);
        lcd.print(capacitance);
        lcd.print("_pF  ");
        CAPACITOR = capacitance;
        res_scale_X = 4;
        delay(100);
      }

      else
      {
        pinMode(cap_in_analogPin, OUTPUT);
        delay(1);
        pinMode(OUT_PIN, INPUT_PULLUP);
        unsigned long u1 = micros();
        unsigned long t;
        int digVal;
        do
        {
          digVal = digitalRead(OUT_PIN);
          unsigned long u2 = micros();
          t = u2 > u1 ? u2 - u1 : u1 - u2;
        } while ((digVal < 1) && (t < 400000L));

        pinMode(OUT_PIN, INPUT);
        val = analogRead(OUT_PIN);
        digitalWrite(cap_in_analogPin, HIGH);
        int dischargeTime = (int)(t / 1000L) * 5;
        delay(dischargeTime);
        pinMode(OUT_PIN, OUTPUT);
        digitalWrite(OUT_PIN, LOW);
        digitalWrite(cap_in_analogPin, LOW);
        float capacitance = -(float)t / R_PULLUP / log(1.0 - (float)val / (float)MAX_ADC_VALUE);

        if (capacitance > 1000.0)
        {

          cap_scale = false; // We change the scale to 1uF - max
        }
        else
        {
          lcd.setCursor(11, 0);
          lcd.print("CAPACITOR");

          lcd.setCursor(0, 1);
          lcd.print("                    ");
          lcd.setCursor(6, 1);
          lcd.print(capacitance);
          lcd.print("_nF  ");
          CAPACITOR = capacitance;
          res_scale_X = 5;
          delay(100);
        }
      }
      while (micros() % 1000 != 0)
        ;
    } ////end of lower scalee
  }

  // 0-10V   ADC0_2    0-20ma  BORNERA 5
  if (MENU_MEDICION == 5)
  {

    digitalWrite(LED_1, LOW);  // turn pullup resistor on
    digitalWrite(LED_2, LOW);  // turn pullup resistor on
    digitalWrite(LED_3, LOW);  // turn pullup resistor on
    digitalWrite(LED_4, LOW);  // turn pullup resistor on
    digitalWrite(LED_5, HIGH); // turn pullup resistor on
    digitalWrite(LED_6, LOW);  // turn pullup resistor on

    int16_t adc0_2; // Leemos el ADC, con 16 bits
    adc0_2 = ads0.readADC(2);
    resistance_voltage = (adc0_2 * 0.1875);

    int16_t adc1_3; // Leemos el ADC, con 16 bits
    adc1_3 = ads1.readADC(3);
    battery_voltage = ((adc1_3 * 0.1875) / 1000);

    y = map(resistance_voltage, 0, 4340, 0, 20000);

    y = y / 1000;

    lcd.setCursor(11, 0);
    lcd.print("     mA  ");

    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(6, 1);
    lcd.print(y, 3);
    lcd.print("_mA");

    delay(100);
  }

  // TERMOCUPLA
  if (MENU_MEDICION == 6)
  {

    digitalWrite(LED_1, LOW);  // turn pullup resistor on
    digitalWrite(LED_2, LOW);  // turn pullup resistor on
    digitalWrite(LED_3, LOW);  // turn pullup resistor on
    digitalWrite(LED_4, LOW);  // turn pullup resistor on
    digitalWrite(LED_5, LOW);  // turn pullup resistor on
    digitalWrite(LED_6, HIGH); // turn pullup resistor on

    lcd.setCursor(11, 0);
    lcd.print("Tipo *K* ");

    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(6, 1);
    TEMPERATURAX = thermocouple.readCelsius();
    lcd.print(TEMPERATURAX, 3);
    lcd.print("_C");
    delay(100);
  }
}

void GENERAR()
{

  if (MENU_GENERAR == 1)
  {

    digitalWrite(LED_7, HIGH); // turn pullup resistor on
    digitalWrite(LED_8, LOW);  // turn pullup resistor on
    digitalWrite(LED_9, LOW);  // turn pullup resistor on

    int16_t adc0_3; // Leemos el ADC, con 16 bits
    adc0_3 = ads0.readADC(3);
    resistance_voltage = ((adc0_3 * 0.1875));

    int16_t adc1_3; // Leemos el ADC, con 16 bits
    adc1_3 = ads1.readADC(3);
    battery_voltage = ((adc1_3 * 0.1875) / 1000);

    lcd.setCursor(11, 2);
    lcd.print("     mA  ");

    lcd.setCursor(16, 2);
    lcd.print("mA");
    /*
            lcd.setCursor(10,2);
            lcd.print(battery_voltage,1);
            lcd.print("V");
      */
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    lcd.setCursor(6, 3);
    lcd.print(resistance_voltage, 3);
    lcd.print("_mA");

    delay(100);
  }

  if (MENU_GENERAR == 2)
  {

    digitalWrite(LED_7, LOW);  // turn pullup resistor on
    digitalWrite(LED_8, HIGH); // turn pullup resistor on
    digitalWrite(LED_9, LOW);  // turn pullup resistor on

    lcd.setCursor(10, 2);
    lcd.print("          ");

    lcd.setCursor(0, 3);
    lcd.print("                    ");
    lcd.setCursor(6, 3);

    int32_t adc1_0; // Leemos el ADC, con 16 bits
    adc1_0 = ads1.readADC(3);
    resistance_voltage = (adc1_0 * 0.1875) / 1000;
    /*
        lcd.setCursor(0,3);
        lcd.print(resistance_voltage,3);
        lcd.print(" ");
    */

    switch (MUESTREO)
    {
    case 0:
      MCP4725_value = 0;
      MCP4725_expected_output = 0;
      break;
    case 6:
      MCP4725_value = 1;
      MCP4725_expected_output = 30;
      break;
    case 12:
      MCP4725_value = 14;
      MCP4725_expected_output = 48;
      break;
    case 18:
      MCP4725_value = 32;
      MCP4725_expected_output = 72;
      break;
    case 24:
      MCP4725_value = 50;
      MCP4725_expected_output = 96;
      break;
    case 30:
      MCP4725_value = 68;
      MCP4725_expected_output = 120;
      break;
    case 36:
      MCP4725_value = 89;
      MCP4725_expected_output = 144;
      break;
    case 42:
      MCP4725_value = 110;
      MCP4725_expected_output = 168;
      break;
    case 48:
      MCP4725_value = 130;
      MCP4725_expected_output = 192;
      break;
    case 54:
      MCP4725_value = 149;
      MCP4725_expected_output = 216;
      break;
    case 60:
      MCP4725_value = 168;
      MCP4725_expected_output = 240;
      break;
    case 66:
      MCP4725_value = 187;
      MCP4725_expected_output = 264;
      break;
    case 72:
      MCP4725_value = 208;
      MCP4725_expected_output = 288;
      break;
    case 78:
      MCP4725_value = 229;
      MCP4725_expected_output = 312;
      break;
    case 84:
      MCP4725_value = 250;
      MCP4725_expected_output = 336;
      break;
    case 90:
      MCP4725_value = 270;
      MCP4725_expected_output = 360;
      break;
    case 96:
      MCP4725_value = 290;
      MCP4725_expected_output = 384;
      break;
    }

    MCP4725.setVoltage(MCP4725_value, false);

    lcd.setCursor(7, 3);
    lcd.print(MCP4725_expected_output);
    lcd.print("_*C");

    MUESTREO++;
    if (MUESTREO > 102)
    {
      MUESTREO = 0;
    }
  }

  if (MENU_GENERAR == 3)
  {

    digitalWrite(LED_7, LOW);  // turn pullup resistor on
    digitalWrite(LED_8, LOW);  // turn pullup resistor on
    digitalWrite(LED_9, HIGH); // turn pullup resistor on

    int16_t adc1_1; // Leemos el ADC, con 16 bits
    adc1_1 = ads1.readADC(1);
    Voltage = (adc1_1 * 0.1875);

    Voltage = Voltage / 126.05705;

    if (Voltage > 0)
    {
      lcd.setCursor(11, 2);
      lcd.print(" 0-34V DC");

      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(6, 3);
      lcd.print(Voltage, 3);
      lcd.print(" _V");
      delay(100);
    }
    else
    {
      lcd.setCursor(11, 2);
      lcd.print(" 0-34V DC");

      lcd.setCursor(0, 3);
      lcd.print("                    ");
      lcd.setCursor(6, 3);
      lcd.print("000.000");
      lcd.print(" _V");
      Voltage = 0.000;
      delay(100);
    }
  }
}
