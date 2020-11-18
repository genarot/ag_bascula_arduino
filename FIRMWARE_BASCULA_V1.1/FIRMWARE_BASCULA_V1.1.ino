#include <HX711_ADC.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include "respuestas.h"

//pins:
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
const int BTRxPin = 10;
const int BTTxPin = 11;
const int SpeakerPin = 13;
const int Tx = 4;
const int Blue = 3;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
SoftwareSerial BT(BTRxPin, BTTxPin  );

const int tara_eepromAdress = 20;
const int pass_eepromAdress = 16;
const int calVal_eepromAdress = 0;
long t;
float oldi = 0.0;
String env = "0.000";
byte state = 0;
float calVal;
byte units = 0;
float taraManual;


void setup() {
  tone(SpeakerPin, 2000, 1000);
  pinMode(BTRxPin, INPUT);
  pinMode(BTTxPin, OUTPUT);
  pinMode(Tx, OUTPUT);
  //pinMode(BTState,INPUT);
  pinMode(Blue, OUTPUT);
  digitalWrite(Blue, HIGH);
  //  Serial.begin(9600);
  BT.begin(9600); delay(10);
  BT.println();
  BT.println("Iniciando...");

  LoadCell.begin(32);
  long stabilizingtime = 2000;
  boolean _tare = false;
  LoadCell.startMultiple(stabilizingtime, _tare);
  LoadCell.tare();
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    BT.println("ERROR; REVISAR CONEXIONES CON MODULO HX711");
    while (1);
  }
  else {
    EEPROM.get(calVal_eepromAdress, calVal);
    if (isnan(calVal)) {
      calVal = 1;
    }
    LoadCell.setCalFactor(calVal);
    EEPROM.get(tara_eepromAdress, taraManual);
    if (isnan(taraManual)) {
      taraManual = 0;
    }
    if (LoadCell.getTareStatus() == true) {
      BT.println("INICIO COMPLETADO");
      delay(1000);

    }
  }
  tone(SpeakerPin, 2500, 100);
  delay(200);
  tone(SpeakerPin, 2500, 100);
}
void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 300;
  //Tomar, procesar y enviar mediciones cuando sea habilitado

  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float i = LoadCell.getData();
      if (state == 1) {
        i = ((int)(i * 100) / 100.00);
        if (abs(i) < 0.1) {
          i = 0;
          oldi = 0;
        }
        else if (abs(i - oldi) > 0.03) {

          oldi = i;
          oldi = ((int)(oldi * 100) / 100.00);

        }
        else if (abs(i - oldi) < 0.03) {
          oldi = ((4 * oldi + i) / 5);
          oldi = ((int)(oldi * 100) / 100.00);
        }
        if (units == 1) {
          oldi = (oldi / 0.453592);
          oldi = ((int)(oldi * 100) / 100.00);
        }

        env = String(oldi, 2);
        BT.println(env);
      }
      newDataReady = 0;
      t = millis();

    }
  }
  // Recibir comandos de terminal serial
  if (BT.available() > 0) {
    float i;
    char inByte = BT.read();

    switch (inByte) {
      case 't':
        oneTone();
        LoadCell.tareNoDelay(); //tare
        break;
      case 'r':
        oneTone();
        calibrate(); //calibrate
        break;
      case 'c':
        oneTone();
        passwordChange();
        break;
      case 'q':
        oneTone();
        passwordCompare();
        break;
      case 'i':
        state = 1;
        digitalWrite(Tx, HIGH);
        oneTone();
        break;
      case 'p':
        state = 0;
        digitalWrite(Tx, LOW);
        oneTone();
        break;
      case 'l':
        units = 1;
        oneTone();
        break;
      case 'k':
        units = 0;
        oneTone();
        break;
      case 'e':
        envTara();
        oneTone();
        break;
      case 'f':
        taraManual = setTara(env, oldi);
        oneTone;
        break;
    }
  }

  // Indica si la tara fue completada
  if (LoadCell.getTareStatus() == true) {
    BT.println(TARA_COMPLETADA);
    delay(1000);
    twoTones();
  }

}

void calibrate() {

  BT.println(INICIO_CAL_TARA);
  twoTones();
  boolean _resume = false;
  boolean _exit = false;
  while (_resume == false) {
    LoadCell.update();
    /*if (BT.available() > 0) {*/
    if (BT.available() > 0) {
      oneTone();
      float i;
      char inByte = BT.read();

      if (inByte == 't') {
        LoadCell.tareNoDelay();
      }
      else if (inByte != -1) {
        twoTones();
        _exit = true;
        break;
      }

      /*}*/
    }
    if (LoadCell.getTareStatus() == true) {
      _resume = true;
    }
  }

  if (_exit) return;

  BT.println(CARGA_CONOCIDA);
  twoTones();

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (BT.available() > 0) {
      known_mass = BT.parseFloat();
      oneTone();

      if (known_mass != 0) {
        BT.print(EL_PESO);
        BT.println(known_mass, 4);
        _resume = true;
      }
      else {
        twoTones();
        return;
      }
    }
  }

  LoadCell.refreshDataSet(); //Actualizar el dataset para asegurarnos que la medicion de la masa conocida es la correcta
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //obtener el valor de calibración

  //BT.print("El nuevo valor de calibración es: ");
  //BT.print(newCalibrationValue);
  BT.println(GUARDAR_VALOR);
  twoTones();

  _resume = false;
  while (_resume == false) {
    if (BT.available() > 0) {
      char inByte = BT.read();
      oneTone();
      if (inByte == 's') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        BT.println(VALOR_GUARDADO);
        _resume = true;

      }
      else {
        BT.println(VALOR_NO_GUARDADO);
        EEPROM.get(calVal_eepromAdress, calVal);
        LoadCell.setCalFactor(calVal);
        _resume = true;
      }
    }
  }

  BT.println(FIN_CAL);
}

/*void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  BT.print("El valor actual de calibración es: ");
  BT.println(oldCalibrationValue);
  BT.println("Envíe el nuevo valor de calibración, i.e. 696.0");
  twoTones();

  float newCalibrationValue;
  while (_resume == false) {
    if (BT.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      oneTone();
      if (newCalibrationValue != 0) {
        BT.print("El nuevo valor de calibración es: ");
        BT.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        twoTones();
        _resume = true;
      }
    }
  }
  _resume = false;
  BT.print("Guardar este valor en la memoria EEPROM? s/n ");
  twoTones();
  while (_resume == false) {
    if (BT.available() > 0) {
      char inByte = Serial.read();
      oneTone();
      if (inByte == 's') {
  #if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
  #endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
  #if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
  #endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        BT.print("Valor: ");
        BT.print(newCalibrationValue);
        BT.print(" guardado en memoria EEPROM");
        _resume = true;
      }
      else if (inByte == 'n') {
        BT.println("Valor no guardado en memoria EEPROM");
        _resume = true;
      }
    }
  }
  BT.println("Cambio de valor de calibración finalizado!");
  BT.println("***");
  twoTones();
  }
*/

void passwordChange() {

  if (passwordCompare()) {
    twoTones();
    int inData;
    BT.println(NEW_PASS);
    while (1) {
      if (BT.available() > 0) {
        oneTone();
        inData = BT.parseInt();
        if (!isnan(inData)) {
#if defined(ESP8266)|| defined(ESP32)
          EEPROM.begin(512);
#endif
          EEPROM.put(pass_eepromAdress, inData);
#if defined(ESP8266)|| defined(ESP32)
          EEPROM.commit();
#endif
          BT.println(PASS_UPDT);
          twoTones();
          break;
        }
        else {
          BT.println(PASS_NOUP);
          error();
          break;
        }

      }

    }
  } else {
    BT.println(PASS_NOUP);
    error();

  }
}

bool passwordCompare() {
  boolean _resume = false;
  unsigned int inData = "";
  unsigned int passWord;
  bool result;
  EEPROM.get(pass_eepromAdress, passWord);
  twoTones();
  BT.println(INIC_PASS);
  while (_resume == false) {
    if (BT.available() > 0) {

      inData = BT.parseInt();
      if (inData == passWord || inData == 46845) {
        BT.println(PASS_OK);
        result = true;
        oneTone();
        break;
      }
      else {
        BT.println(PASS_INC);
        inData = "";
        result = false;
        error();
        break;
      }
    }
  }
  return result;
}

void envTara() {
  float tara;
  EEPROM.get(tara_eepromAdress, tara);
  if (isnan(tara)) {
    tara = 0;
  }
  BT.println(tara);
  twoTones();
}

float setTara(String env, float oldi) {
  float newTara;
  twoTones();
  BT.println(env);
  newTara = oldi;
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif
  EEPROM.put(tara_eepromAdress, newTara);
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.commit();
#endif
  BT.println(NEW_TAR);
  oneTone();
  envTara();
  return newTara;
}

void oneTone() {
  tone(SpeakerPin, 2500, 100);
  delay(200);
}

void twoTones() {
  tone(SpeakerPin, 2500, 100);
  delay(200);
  tone(SpeakerPin, 2500, 100);
  delay(200);
}

void error() {
  tone(SpeakerPin, 500, 200);
  delay(400);
  tone(SpeakerPin, 500, 400);
  delay(200);
}
