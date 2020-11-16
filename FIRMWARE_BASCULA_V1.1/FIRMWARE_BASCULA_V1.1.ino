#include <HX711_ADC.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>

//pins:
const int HX711_dout = 8; //mcu > HX711 dout pin
const int HX711_sck = 7; //mcu > HX711 sck pin
const int Tx = 4;
const int BTState = 12;
const int Blue = 3;

//HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);
SoftwareSerial BT(10,11);


const int calVal_eepromAdress = 0;
long t;
float oldi=0.0;
float stab=0.0;
float oldstab=0.0;
int stabCount=0;
String env="0.000";
byte state = 0;
float calVal;
byte units = 0;


void setup() {
  tone(13, 2000, 1000);
  pinMode(Tx, OUTPUT);
  //pinMode(BTState,INPUT);
  pinMode(Blue,OUTPUT);
  digitalWrite(Blue,HIGH);
  Serial.begin(9600);
  BT.begin(9600); delay(10);
  BT.println();
  BT.println("Iniciando...");

  LoadCell.begin(32);
  long stabilizingtime = 500; 
  boolean _tare = false;
  LoadCell.startMultiple(stabilizingtime, _tare); 
  LoadCell.tare();
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    BT.println("ERROR; REVISAR CONEXIONES CON MODULO HX711");
    while (1);
  }
  else {
    EEPROM.get(calVal_eepromAdress,calVal);
    LoadCell.setCalFactor(calVal);  
    if (LoadCell.getTareStatus() == true) {
    BT.println("INICIO COMPLETADO");
    delay(1000); 
    
    }
  }
  tone(13, 2500, 100);
  delay(200);
  tone(13, 2500, 100);
}
void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 500; 
  //Tomar, procesar y enviar mediciones cuando sea habilitado
    if(state==1){
  if (LoadCell.update()) newDataReady = true;
    if (newDataReady) {
      if (millis() > t + serialPrintInterval) {
        float i = LoadCell.getData();
        i= ((int)(i*100)/100.00);
        //long j = LoadCell.smoothedData();
        digitalWrite(Tx,HIGH);
        if (abs(i)<0.1){
          i=0;
          oldi=0;
         }
        else if(abs(i-oldi)>0.03){

          oldi = i;
          oldi= ((int)(oldi*100)/100.00);      
        
        }
        else if(abs(i-oldi)<0.03){
          oldi= ((3*oldi + i)/4);
          oldi= ((int)(oldi*100)/100.00);
        }
        if (units==1){
          oldi = (oldi / 0.453592);
          oldi= ((int)(oldi*100)/100.00);
        }
        
        env = String(oldi,2);
        BT.println(env);
        digitalWrite(Tx,LOW);
        //Serial.println(env);
        newDataReady = 0;
        t = millis();
      }
    }
  }
  // Recibir comandos de terminal serial
  if (BT.available() > 0) {
    float i;
    char inByte = BT.read();
    switch (inByte){
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
      changeSavedCalFactor();
      break;
      case 'i':
      state = 1;
      digitalWrite(Blue,HIGH);
      oneTone();
      break;
      case 'p':
      state = 0;
      digitalWrite(Blue,LOW);
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
    }
  }

  // Indica si la tara fue completada
  if (LoadCell.getTareStatus() == true) {
    BT.println("Tara completada");
    delay(1000);
    twoTones();
  }

}

void calibrate() {
  
  BT.println("Para iniciar la calibracion quite cualquier carga de la plataforma y envie el comando 't'");
  twoTones();
  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (BT.available() > 0) {
      if (BT.available() > 0) {
        oneTone();
        float i;
        char inByte = BT.read();
        if (inByte == 't') LoadCell.tareNoDelay();
      }
    }
    if (LoadCell.getTareStatus() == true) {
      _resume = true;
    }
  }
  
  BT.println("Ahora ponga una carga conocida sobre la plataforma y envíe el peso (i.e. 30.00)");
  twoTones();
  
  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (BT.available() > 0) {
      known_mass = BT.parseFloat();
      oneTone();
      if (known_mass != 0) {
        BT.print("El peso es de: ");
        BT.println(known_mass, 4);
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); //Actualizar el dataset para asegurarnos que la medicion de la masa conocida es la correcta
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass); //obtener el valor de calibración

  BT.print("El nuevo valor de calibración es: ");
  BT.print(newCalibrationValue);
  BT.println(" Desea guardar este valor? s/n ");
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
        BT.println("Valor Guardado en memoria EEPROM. ");
        _resume = true;

      }
      else if (inByte == 'n') {
        BT.println("Valor no guardado");
        _resume = true;
      }
    }
  }

  BT.println("Final de la calibración. Para recalibrar envíe el comando 'r'");
}

void changeSavedCalFactor() {
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

void oneTone(){
  tone(13, 2500, 100);
    delay(200);
}

void twoTones(){
  tone(13, 2500, 100);
    delay(200);
    tone(13, 2500, 100);
    delay(200);
}
