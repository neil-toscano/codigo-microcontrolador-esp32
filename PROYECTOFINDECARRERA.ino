#include <Arduino.h>
#include <ArduinoJson.h>
#include <time.h>
#include <NTPClient.h>
#include <FirebaseESP32.h>
#include <WiFiUdp.h>
#include <DHTesp.h>
#include <Adafruit_NeoPixel.h>

//------------------DEFINE:Constantes----------------------------
//Temp y humedad de aire*****************************************
#define pinDHT      18
#define powerDHT   15
//Ventiladores
#define ENA        13
#define ENB        12
//Calefacción
#define Heating    21
//Humidificador
#define Humidifier 14
//Luz*************************************************************
#define LDR1       34 
#define LDR2       35
#define RGBLED     27
#define PIN        2
//PH y NPK: Rs485 modbus*****************************************
//Rx2 -->R0
//Tx2 -->DI
#define DERE       23
//Humedad de suelo************************************************
#define HumSensor1 32
#define HumSensor2 33 
#define PowerSensor1 25
#define PowerSensor2 26
#define Valvula1   4
#define Valvula2   19
//ConsideracionesHardware*******************************************
#define ADCResolucion 3.3/4095
#define Preescaler1 48000
#define Preescaler2 8000
//Luz***************************************************************
#define A  1000     //Resistencia en oscuridad en KΩ
#define B_LDR 15        //Resistencia a la luz (10 Lux) en KΩ
#define Rc 1       //Resistencia calibracion en KΩ
//ParámetrosDeMonitoreoYControl=4***********************************
#define PARAMETROS_CONTROL 4
//Firebase**********************************************************
#define FIREBASE_HOST "https://projectocarrera-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyCFj27zofYoHq-vCgG5jasliB0OZHG-mfw"


//---------------------VariablesYTipoConst---------------------------------
//-FIREBASE----------------------------------------------------------------
FirebaseData firebaseData;
FirebaseJsonData result;
FirebaseJson json;
FirebaseJsonData red;
FirebaseJsonData green;
FirebaseJsonData blue;
FirebaseJsonData encen;
FirebaseJsonData AireCaliente;
FirebaseJsonData AireFrio;
FirebaseJsonData Apagar;
FirebaseJsonData checBox;
FirebaseJsonData checBoxSuelo,regar,apagar_regador;
FirebaseJsonData checBoxHumAir,vaporizar,apagar_vapor;
FirebaseJsonData checkBoxLuz;

//VariablesDeRed-----------------------------------------------------------
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "south-america.pool.ntp.org");
const char* ssid = "GDSC_UNI";
const char* password = "console.log(GDSC_UNI);";
//VariablesTemperaturaYHumedad---------------------------------------------
float TempRangeMax = 20.0;//Rango máximo de temperatura
float TempRangeMin = 12.0;//Rango minimo de temperatura
float HumRangeMax = 85.0; //Rango maximo de humedad de aire
float HumRangeMin = 70.0; //Rango minimo de humedad de aire
TempAndHumidity data;
DHTesp dht;
//Humedaddesuelo------------------------------------------------------------
int tiempomuestreo = 1; //en minutos
int tiemporiego = 6; // en segundos
int HumSuelLimit=5000;
int medida1,medida2;
volatile bool activarTimer=false;
volatile bool desactivarTimer=false;
//Luz------------------------------------------------------------------------
int lightvalue1,lightvalue2,ilum1,ilum2;
int maxlux =300;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(9, PIN, NEO_GRB + NEO_KHZ800);
//PHyNPK----------------------------------------------------------------------
const byte PHandNPK[] = {0x01,0x03,0x00,0x03,0x00,0x04, 0xb4, 0x09};
byte ResponsePHandNPKData[13];
//AlmacenamientoDeTiempo-------------------------------------------------------
uint64_t Tiempoloop;
hw_timer_t *timer1 = NULL; 
//hw_timer_t *timer2 = NULL;
//ValoresAnteriores------------------------------------------------------------
struct ParametrosMedicion
{
  float temperatura;
  float humedad;
  int humedadsuelo;
  int luz;
  byte N;
  byte P;
  byte K;
  float PH; 
};
ParametrosMedicion MedicionAnterior = {0.0,0.0,0,0,0,0,0,0};
// ARRAY QUE RECOGERA LAS VARIABLES DE MANUAL U AUTOMATICO DE LOS PARAMETROS DE RIEGO
bool ButtonControl[PARAMETROS_CONTROL]={0,0,0,0};
//ButtonControl[0]: Temperatura
  //Si ButtonControl[0] = 0, el modo automático esta activado
  //Si ButtonControl[0] = 1, el modo manual esta activado

//ButtonControl[1]: Humedad de aire
  //Si ButtonControl[1] = 0, el modo automático esta activado
  //Si ButtonControl[1] = 1, el modo manual esta activado

//ButtonControl[2]: Humedad de suelo
  //Si ButtonControl[2] = 0, el modo automático esta activado
  //Si ButtonControl[2] = 1, el modo manual esta activado

//ButtonControl[3]: Luz
  //Si ButtonControl[3] = 0, el modo automático esta activado
  //Si ButtonControl[3] = 1, el modo manual esta activado    

//A su vez, cada estado tiene un número asociado para el switchcase del loop()
//Esto es por la fórmula 2*i+ButtonControl[i]

//Los valores asociados a cada estado resultan:
  //Si ButtonControl[0] = 0  -------> 0 case ----> Temperatura en modo automatico
  //Si ButtonControl[0] = 1  -------> 1 case ----> Temperatura en modo manual
  //Si ButtonControl[1] = 0  -------> 2 case ----> Humedad del aire en modo automatico
  //Si ButtonControl[1] = 1  -------> 3 case ----> Humedad del aire en modo manual
  //Si ButtonControl[2] = 0  -------> 4 case ----> Humedad del suelo en modo automatico
  //Si ButtonControl[2] = 1  -------> 5 case ----> Humedad del suelo en modo manual
  //Si ButtonControl[3] = 0  -------> 6 case ----> Luz en modo automatico
  //Si ButtonControl[3] = 1  -------> 7 case ----> Luz en modo manual

//*0FuncionesDeSetup******************************************************
void InicializacionGPIO();
void InicializacionTimer();
//FuncionesDeFirebase***************************************************
//comunicacion Wifi
void enviarDatosFirebase(float temperatura);
void conectarWiFi();
void inicializarFirebase();
//*1FuncionesDeTemperaturaYHumedad**************************************
//Lectura de la temperatura y humedad
void readValueTempAndHum();
void CargarTemperatura();
//Accionador de los ventiladores y calefaccion
//void AccionarVentilacionYCalor(bool ventilacion, bool calentamiento);
void GestionActuadoresTemperatura(float temper);
void GestionActuadoresTemperaturaMANUAL(int airefrio, int airecaliente);
//Accion de control de la humedad
void encenderHumidificador();
void apagarHumidificador();
void GestionActuadoresHumedad(float hum);
void GestionActuadoresHumedadMANUAL(int vaporizar);
void CargarHumedad();
//*2Funcionesdehumedaddesuelo********************************************
//LecturaDeLaHumedadDeSuelo
bool CheckTimerAndReadData();
void ReadHSuelData();
//AccionDeControl
void EncenderValvulas();
//cargar valor en la estructura;
void CargarValorHSuelo();
//ManejoDeTimers para la lectura y el riego
void FinTimer();
void IRAM_ATTR lectura();
//void IRAM_ATTR riego();
//*3Luz******************************************************************
void ReadAndPrintLDR();
void cargarValorLux();
void EncenderLed(int R,int G, int B);
//*4PHNPK****************************************************************
bool ObtenerNPKyData();
void CargarValorPHYNPK();

// DEvoluciones de Array son 4 arrays
int* devuelveArrayTemp(); //[check,frio,caliente,apagar]
int* devuelveArrayHumedadSuelo();//{ckeckbox,regar,Apagar}
int* devuelveArrayHumedadAmbiente();  //{ckeckbox,Vaporizar,Apagar}
int* devuelveArrayLuz(); //{ckeckbox} auto->manual

// Manejo Luces=================================
void leerLuzManual();// si le das click al checkbox y envias el color se enciende y si desmarcas el checkbox se apaaga el led 
void ApagarLuzAutomaBlanco(); //Con esto obligas que se apague el led
void encenderLuzAutomaBlanco();// enciendes luz blanca

void EncenderTodo();

bool IS_HUM_ON=false;



void setup() {
  Serial.begin(9600);
  Serial2.begin(4800, SERIAL_8N1, 16, 17);
  pinMode(22,OUTPUT);
  pixels.begin();
  conectarWiFi();//conexion***************** 
  timeClient.setTimeOffset(-18000);// la hora esta sincronizado con un servidor de Estados unidos
  //Existe un desface de 5 horas
  //timeClient.begin();// iniciamos la conexion***********
  //Serial.println("Conexión exitosa");
  inicializarFirebase();
  InicializacionGPIO();
  InicializacionTimer();
}
void loop() {
  //cargarFirebase();
  int *ArrayManualTemp = devuelveArrayTemp();
  int *ArrayManualHumAm = devuelveArrayHumedadAmbiente();
  int *ArrayManualHumSuel = devuelveArrayHumedadSuelo();
  int *ArrayManualLuz = devuelveArrayLuz();

  ButtonControl[0] = ArrayManualTemp[0];
  ButtonControl[1] = ArrayManualHumAm[0];
  ButtonControl[2] = ArrayManualHumSuel[0];
  ButtonControl[3] = ArrayManualLuz[0];

  char bufferaux[40];
  byte j=0;
  byte auxiliar=0;
  readValueTempAndHum();
  for(j=0;j<PARAMETROS_CONTROL; j++){
      auxiliar = byte(ButtonControl[j]) + 2*j;
      switch (auxiliar)
      {
      case 0:
        {
          Serial.println("Estas en el caso 0");
          sprintf(bufferaux,"Temperature: %.2f °C",data.temperature);
          Serial.println("TEMPERATURA:");
          Serial.println(bufferaux);
          GestionActuadoresTemperatura(data.temperature);
          CargarTemperatura();
        }
        break;
      case 1:
        {
          Serial.println("Estas en el caso 1");
          GestionActuadoresTemperaturaMANUAL(ArrayManualTemp[1], ArrayManualTemp[2]);
          //leer();
          //AccionarVentilacionYCalor(valorleer1,valorleer2);
          //TempAndHumidity data = dht.getTempAndHumidity(); 
          CargarTemperatura();
        }
        break;
      case 2:
        {
          Serial.println("Estas en el caso 2");
          GestionActuadoresHumedad(data.humidity);
          Serial.println("\nHUMEDAD:");
          sprintf(bufferaux,"Humidity: %.2f °C",data.humidity);
          Serial.println(bufferaux);
          CargarHumedad();
        }
        break;
      case 3:
        {
          Serial.println("Estas en el caso 3");
          GestionActuadoresHumedadMANUAL(ArrayManualHumAm[1]);
          //leer();
          //TempAndHumidity data = dht.getTempAndHumidity(); 
          CargarHumedad();
        }
        break;
      case 4:
        {
          Serial.println("Estas en el caso 4");
          Serial.println("\n HUMEDAD DE SUELO");
          if(CheckTimerAndReadData()){
            Serial.println("lectura realizada:");
            Serial.print("Sensor 1: ");
            Serial.println(medida1);
            Serial.print("Sensor 2: ");
            Serial.println(medida2);
            Serial.print("Medida promedio: ");
            Serial.println((medida1+medida2)/2);
            CargarValorHSuelo();
          }
          if((medida1+medida2)/2 > HumSuelLimit){
            EncenderValvulas();
          }
          FinTimer();
        }
        break;
      case 5:
        {
          Serial.println("Estas en el caso 5");
          //read valores de firebase
          //tiemporiego=
          //tiempomuestreo=
          //InicializacionTimer()
          //int medida1,medida2;
          if(ArrayManualHumSuel[1] !=0 ){
            EncenderValvulas();
          }
          ReadHSuelData();
          CargarValorHSuelo();
        
        }
        break;
      case 6:
        {
          Serial.println("Estas en el caso 6");
          //int lightvalue1,lightvalue2;
          ReadAndPrintLDR();
          cargarValorLux();

          if((ilum1+ilum2)/2 >maxlux){
            ApagarLuzAutomaBlanco();
            //APAGARLUZ();
            //Send alert to firebase
          }
          if((ilum1+ilum2)/2 < maxlux){
            encenderLuzAutomaBlanco();
          }
          
        }
        break;
      case 7:
        {
          Serial.println("Estas en el caso 7");
          //leer()
          //int lightvalue1,lightvalue2;
          leerLuzManual();
          ReadAndPrintLDR();
          cargarValorLux();
        //if(iluminarahora){
          //digitalWrite(RGBLED, HIGH);
        //}

        }
        break;
      default:
        break;
      }
  }
  digitalWrite(2,LOW);



//int* tempera = devuelveArrayHumedadSuelo();//devuelve un array {checkbox,vaporizar,apagar}
//
//Serial.println(tempera[0]);
//Serial.println(tempera[1]);
//Serial.println(tempera[2]);
//
//leerLuzManual();
//Serial.println("fin");
  
  
  bool errordelectura = ObtenerNPKyData();
  while(errordelectura == 0){
    Tiempoloop=esp_timer_get_time();
    while((esp_timer_get_time() - Tiempoloop) < 5000000UL){}
    
    errordelectura = ObtenerNPKyData();
  }
  CargarValorPHYNPK();
  Serial.println("----------------------------------------");
  Serial.println(MedicionAnterior.temperatura);
  Serial.println(MedicionAnterior.humedad);
  Serial.println(MedicionAnterior.humedadsuelo);
  Serial.println(MedicionAnterior.luz);
  Serial.println(MedicionAnterior.PH);
  Serial.println(MedicionAnterior.N);
  Serial.println(MedicionAnterior.P);
  Serial.println(MedicionAnterior.K);
  Serial.println("--------------------------------------");

  EncenderTodo();
  delay(10000);


  Tiempoloop=esp_timer_get_time();
  while((esp_timer_get_time() - Tiempoloop) < 5000000UL){}

}




void InicializacionGPIO(){
  //*******************TEMPERATURA Y HUMEDAD********************
  //Alimentacion del DHT22
  pinMode(powerDHT,OUTPUT);
  digitalWrite(powerDHT,HIGH);
  //Inicializamos el dht22
  dht.setup(pinDHT, DHTesp::DHT22);
  //Alimentación de los ventiladores y del humidificador
  pinMode(ENA, OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(Heating, OUTPUT);
  digitalWrite(Heating,HIGH);
  pinMode(Humidifier,OUTPUT); 

  //*****************HUMEDAD DE SUELO**************************
  analogReadResolution(12);//4096
  analogSetPinAttenuation(HumSensor1, ADC_11db);
  analogSetPinAttenuation(HumSensor2, ADC_11db);
  pinMode(PowerSensor1, OUTPUT);
  pinMode(PowerSensor2, OUTPUT);
  pinMode(Valvula1, OUTPUT);
  pinMode(Valvula2, OUTPUT);
  digitalWrite(Valvula1,HIGH);
  digitalWrite(Valvula2,HIGH);
  //******************LUZ LDR**********************************
  pinMode(RGBLED,OUTPUT);
  digitalWrite(RGBLED,LOW);

  //****************NPKANDPH SENSOR*****************************
  pinMode(DERE,OUTPUT);
  digitalWrite(DERE,LOW);
  
  //delay(1000);
}
void InicializacionTimer(){
  timer1 = timerBegin(1, Preescaler1, true);
  timerAttachInterrupt(timer1, &lectura, true);
  //timerAlarmWrite(timer1, 100000*tiempomuestreo, true);  
  timerAlarmWrite(timer1, 25000*tiempomuestreo, true);

  //timer2 = timerBegin(2, Preescaler2, true);
  //timerAttachInterrupt(timer2, &riego, true);
  //timerAlarmWrite(timer2, 5000*tiemporiego, true);

  timerAlarmEnable(timer1);
  //digitalWrite(PowerSensor,HIGH);
  //timerAlarmEnable(timer2);
  //timerStop(timer2);
}
bool ObtenerNPKyData(){
  //Serial.println("que tal");
  Serial.println("Sending command PH AND NPK");
  Serial.print("Response: ");
  byte j=0;
  uint64_t startTime;
  uint8_t rxByte;

    //byte i = 0;
    digitalWrite(DERE,HIGH);


    Serial2.write(PHandNPK, sizeof(PHandNPK));

    Serial2.flush();
    
    digitalWrite(DERE,LOW);
    startTime = esp_timer_get_time();

    while ( (esp_timer_get_time() -startTime) < 2000000UL ) {
      if ( Serial2.available() ) {
        rxByte = Serial2.read();
        if(j<13 && j>0){
            ResponsePHandNPKData[j] = rxByte;
            Serial.print(ResponsePHandNPKData[j],HEX);
            Serial.print( " " );
            j++;
        }
        if(j==0 && rxByte ==1){
          ResponsePHandNPKData[j] = rxByte;
          Serial.print(ResponsePHandNPKData[j],HEX);
          Serial.print( " " );
          j++;
        }
        
        //myData[i]=rxBytes
        
      }
    }
  Serial.println();
  Serial.println();
  
  if(ResponsePHandNPKData[0]!=1 && ResponsePHandNPKData[1]!=3){
      return 0;
  }

  return 1;
}
void CargarValorPHYNPK(){
  float calculoPH = (ResponsePHandNPKData[3]*256+ResponsePHandNPKData[4])/10.0;
  int calculoN= ResponsePHandNPKData[5]*256+ResponsePHandNPKData[6];
  int calculoP= ResponsePHandNPKData[7]*256+ResponsePHandNPKData[8];
  int calculoK= ResponsePHandNPKData[9]*256+ResponsePHandNPKData[10];
  if(MedicionAnterior.PH != calculoPH){
    MedicionAnterior.PH = calculoPH;
  }
  if(MedicionAnterior.N != calculoN){
    MedicionAnterior.N = calculoN;
  }
  if(MedicionAnterior.P !=calculoP){
    MedicionAnterior.P = calculoP;
  }
  if(MedicionAnterior.K != calculoK){
    MedicionAnterior.K = calculoK;
  }
}



void readValueTempAndHum(){
  byte aux=0;
  data = dht.getTempAndHumidity();
  if(isnan(data.humidity) ||isnan(data.temperature)){
    do{
    digitalWrite(powerDHT,LOW);
    delay(100);
    digitalWrite(powerDHT,HIGH);
    data = dht.getTempAndHumidity();
    aux++;
  }while(aux<255);
  }
}
void CargarTemperatura(){
  if(data.temperature != MedicionAnterior.temperatura){
    MedicionAnterior.temperatura = data.temperature;
  }
}

void CargarHumedad(){
  if(data.humidity != MedicionAnterior.humedad){
    MedicionAnterior.humedad = data.humidity;
  }
}

void GestionActuadoresTemperatura(float temper){
  if (temper > TempRangeMax){
    //Serial.print("entre");
    //AccionarVentilacionYCalor(true, false);
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(Heating,HIGH);
  }
  if (temper < TempRangeMin)
  {
    //AccionarVentilacionYCalor(true, true);
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(Heating,LOW);
  }
  if((temper < TempRangeMax) && (temper > TempRangeMin)){
    //AccionarVentilacionYCalor(false, false);
    digitalWrite(ENA,LOW);
    digitalWrite(ENB,LOW);
    digitalWrite(Heating,HIGH);
  }
}
void GestionActuadoresTemperaturaMANUAL(int airefrio, int airecaliente){
  if(airefrio!=0){
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(Heating,HIGH);
  }
  if(airecaliente!=0){
    digitalWrite(ENA,HIGH);
    digitalWrite(ENB,HIGH);
    digitalWrite(Heating,LOW);
  }
  if(airefrio==airecaliente && airecaliente==0){
    digitalWrite(ENA,LOW);
    digitalWrite(ENB,LOW);
    digitalWrite(Heating,HIGH);
  }
}


void GestionActuadoresHumedad(float hum){
  if(hum < HumRangeMin){
    encenderHumidificador();
  }
  if(hum>HumRangeMax){
    apagarHumidificador();
  }
}

void GestionActuadoresHumedadMANUAL(int vaporizar){
  if(vaporizar){
    encenderHumidificador();
  }
  else{
    apagarHumidificador();
  }
}
//void AccionarVentilacionYCalor(bool ventilacion, bool calentamiento){
  //digitalWrite(ENA,ventilacion);
  //digitalWrite(ENB,ventilacion);
  //digitalWrite(InA, LOW);
  //digitalWrite(InB, ventilacion);
  //digitalWrite(Heating, ~calentamiento);
//}

void encenderHumidificador(){
  if(IS_HUM_ON == false){

    digitalWrite(Humidifier,HIGH);
    delay(750);
    digitalWrite(Humidifier, LOW);
    IS_HUM_ON = true;
  }
  
}

void apagarHumidificador(){
  if(IS_HUM_ON == true){
    
    digitalWrite(Humidifier,HIGH);
    delay(750);
    digitalWrite(Humidifier, LOW);
    delay(1500);
    digitalWrite(Humidifier,HIGH);
    delay(750);
    digitalWrite(Humidifier, LOW);
    IS_HUM_ON = false;
  }
  
}
//*********************HUMEDAD SUELO******************************

void IRAM_ATTR lectura() {
  activarTimer = true;
}
void IRAM_ATTR riego() {
  desactivarTimer = true;
  digitalWrite(22,LOW);
  digitalWrite(Valvula1, HIGH);
  digitalWrite(Valvula2, HIGH);
}
bool CheckTimerAndReadData(){
    if(activarTimer){
        ReadHSuelData();
        return 1;
        }
    return 0;
}
void ReadHSuelData(){
    digitalWrite(PowerSensor1,HIGH);
    digitalWrite(PowerSensor2,HIGH);
    delay(500);
    medida1 = analogRead(HumSensor1);
    medida2 = analogRead(HumSensor2);
    digitalWrite(PowerSensor1,LOW);
    digitalWrite(PowerSensor1,LOW);
}
void EncenderValvulas(){
  digitalWrite(22,HIGH);
  digitalWrite(Valvula1, LOW);
  digitalWrite(Valvula2, LOW);
  delay(tiemporiego*1000);
  digitalWrite(Valvula1, HIGH);
  digitalWrite(Valvula2, HIGH);
  //timerRestart(timer2);
  //timerStart(timer2);
}

void CargarValorHSuelo(){
    if(((medida1+medida2)/2) != MedicionAnterior.humedadsuelo){
        MedicionAnterior.humedadsuelo = ((medida1+medida2)/2);
    }
    activarTimer =false;
}


void FinTimer(){
    if(desactivarTimer){
        //timerStop(timer2);
        desactivarTimer = false;
        
    }
    
}
//****************************LUZ******************************************

void ReadAndPrintLDR(){
    lightvalue1 = analogRead(LDR1);
    lightvalue2 = analogRead(LDR2);
    ilum1 = ((long)lightvalue1*A*10)/((long)B_LDR*Rc*(4096-lightvalue1));
    ilum2 = ((long)lightvalue2*A*10)/((long)B_LDR*Rc*(4096-lightvalue2));
    Serial.println("Datos del ADC/LDR 34");
    Serial.println(lightvalue1);
    Serial.print("intensidad de luz: ");
    Serial.println(ilum1);
    Serial.println();
    delay(500);

    Serial.println("Datos del ADC/LDR 35");
    Serial.println(lightvalue2);
    Serial.print("intensidad de luz: ");
    Serial.println(ilum2);
    Serial.println();

    Serial.println("Intensidad promedio");
    Serial.println((ilum1+ilum2)/2);

}

void cargarValorLux(){
    if(((ilum1 + ilum2)/2) != MedicionAnterior.luz){
            MedicionAnterior.luz=((ilum1 + ilum2)/2);
          }
}




//------------------------------FIREBASE-----------------------------------
void conectarWiFi() {///WIFI====================
  WiFi.begin(ssid, password);
  Serial.print("Conectando a WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.print("Conexión WiFi establecida. Dirección IP: ");
  Serial.println(WiFi.localIP());
}
void inicializarFirebase() {
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);
  Firebase.setMaxRetry(firebaseData, 3);
}

void enviarDatosFirebase(float temperatura) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor", temperatura);
  json.set("fecha",buffer);
  json.set("nombre","Valor_temp");
  if (Firebase.pushJSON(firebaseData, "sensores/temperatura", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}




void enviarDatosHumedadAireFirebase(float humedad) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor", humedad);
  json.set("fecha",buffer);
  json.set("nombre","Humedad_valor");
  if (Firebase.pushJSON(firebaseData, "sensores/HumedadAmbiente", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosHumedadSueloFirebase(float humedad) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  float Hume_suelo=100-(100*humedad)/4095;
  

  json.set("valor", Hume_suelo);
  json.set("fecha",buffer);
  json.set("nombre","Humedad_valor");
  if (Firebase.pushJSON(firebaseData, "sensores/HumedadSuelo", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosPHFirebase(float PH) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor", PH);
  json.set("fecha",buffer);
  json.set("nombre","PH_Valor");
  if (Firebase.pushJSON(firebaseData, "sensores/PhSuelo", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosPotasioFirebase(float potasio) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor",potasio);
  json.set("fecha",buffer);
  json.set("nombre","Potasio");
  if (Firebase.pushJSON(firebaseData, "sensores/nutrientes/potasio", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosFosforoFirebase(float fosforo) {
  Serial.println("fosforo");
  Serial.println(fosforo);

   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  //char buffer[30];
  char buffer[100];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor",fosforo);
  json.set("fecha",buffer);
  json.set("nombre","Potasio");
  if (Firebase.pushJSON(firebaseData, "sensores/nutrientes/fosforo", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosNitrogenoFirebase(float nitrogeno) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor",nitrogeno);
  json.set("fecha",buffer);
  json.set("nombre","Nitrogeno");
  if (Firebase.pushJSON(firebaseData, "sensores/nutrientes/nitrogeno", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}
void enviarDatosLucxFirebase(float Lucx) {
   timeClient.update();
// Obtiene el tiempo actual en segundos desde el 1 de enero de 1970
  time_t currentTime =  timeClient.getEpochTime(); // Obtener el tiempo en formato epoch

  // Convierte el tiempo actual a una estructura de tiempo local
  struct tm *localTime = localtime(&currentTime);
  char buffer[30];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S.000Z", localTime);

  // Imprimir la hora formateada
  Serial.println(buffer);
  

  json.set("valor",Lucx);
  json.set("fecha",buffer);
  json.set("nombre","Lucx");
  if (Firebase.pushJSON(firebaseData, "sensores/Lucs", json)) {
    Serial.println("Datos enviados correctamente");
  } else {
    Serial.println("Error al enviar los datos");                 
    Serial.println(firebaseData.errorReason());
  }
  
}

void EncenderTodo(){
  enviarDatosFirebase(MedicionAnterior.temperatura);

  delay(50);
  enviarDatosHumedadAireFirebase(MedicionAnterior.humedad);
  delay(50);
  enviarDatosHumedadSueloFirebase(MedicionAnterior.humedadsuelo);
  delay(50);
  enviarDatosPHFirebase(MedicionAnterior.PH);
  delay(50);
  enviarDatosFosforoFirebase(MedicionAnterior.P);
  delay(50);
  enviarDatosNitrogenoFirebase(MedicionAnterior.N);
  delay(50);
  enviarDatosPotasioFirebase(MedicionAnterior.K);
  delay(50);
  enviarDatosLucxFirebase(MedicionAnterior.luz);
  delay(50);


}

// DEvolver los arrays que tenemos para cada ventana.....................................

int* devuelveArrayTemp() {
    //{ckeckbox,aireFrio,AireCaliente,Apagar}
  static int array[] = {0, 0, 0, 0};
   if (Firebase.getJSON(firebaseData, "/sensores/estado_vent/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(AireCaliente , "AireCaliente");
     json.get(AireFrio , "AireFrio");
     json.get(Apagar , "Apagar");
     json.get(checBox , "checBox");
     int Cali=AireCaliente.to<int>();
     int Frio=AireFrio.to<int>();
     int Apa=Apagar.to<int>();
     int Chec=checBox.to<int>();
    array[0]=Chec;
    array[1]=Frio;
    array[2]=Cali;
    array[3]=Apa;
     
     
    
    return array;

  } else {
    Serial.println(firebaseData.errorReason());
  }
    
}

int* devuelveArrayHumedadSuelo() {
    //{ckeckbox,regar,Apagar}
  static int array[] = {0, 0, 0};
   if (Firebase.getJSON(firebaseData, "/sensores/estado_Suelo/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(regar , "regar");
     json.get(apagar_regador , "Apagar");
     json.get(checBoxSuelo , "checkBox");
     int regando=regar.to<int>();
     int Apagando=apagar_regador.to<int>();
     int Chec=checBoxSuelo.to<int>();
    array[0]=Chec;
    array[1]=regando;
    array[2]=Apagando;
     
     
    
    return array;

  } else {
    Serial.println(firebaseData.errorReason());
  }
    
}


int* devuelveArrayHumedadAmbiente() {
    //{ckeckbox,Vaporizar,Apagar}
  static int array[] = {0, 0, 0};
  
   if (Firebase.getJSON(firebaseData, "/sensores/estado_ambien/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(vaporizar , "vaporizador");
     json.get(apagar_vapor , "Apagar");
     json.get(checBoxHumAir , "checkBox");
     int vaporizar=regar.to<int>();
     int Apa=apagar_vapor.to<int>();
     int Chec=checBoxHumAir.to<int>();
    array[0]=Chec;
    array[1]=vaporizar;
    array[2]=Apa;
     
     
    
    return array;

  } else {
    Serial.println(firebaseData.errorReason());
  }
    
}


int* devuelveArrayLuz() {
    //{ckeckbox} auto->manual
  static int array[] = {0};
  
   if (Firebase.getJSON(firebaseData, "/sensores/luz/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(checkBoxLuz , "checkbox");
    
     int checkLuz=checkBoxLuz.to<int>();
     
    array[0]=checkLuz;
    
     
     
    
    return array;

  } else {
    Serial.println(firebaseData.errorReason());
  }
    
}
//MANEJO LUCES---------------------------
void EncenderLed(int R,int G, int B){
  pixels.setPixelColor(0, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(1, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(2, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(3, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(4, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(5, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(6, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(7, pixels.Color(R,G,B)); // 
  pixels.setPixelColor(8, pixels.Color(R,G,B)); // 
  Serial.println("encendiendo");
pixels.show(); // This sends the updated pixel color to the hardware.

  
}

void leerLuzManual() {// si le das click al checkbox y envias el color se enciende y si desmarcas el checkbox se apaaga
// 
  
    if (Firebase.getJSON(firebaseData, "/sensores/luz/")) {
     FirebaseJson &json = firebaseData.to<FirebaseJson>();
     json.get(red , "R");
     json.get(green , "G");
     json.get(blue , "B");
     json.get(encen , "E");
     int Red=red.to<int>();
     int Green=green.to<int>();
     int Blue=blue.to<int>();
     int Encen=encen.to<int>();
     if(Encen==0){
      pixels.clear();
      pixels.show();
      Serial.println("apagando lux");
     }
     else{
      EncenderLed(Red,Green,Blue);
     }
     
     
    

  } else {
    Serial.println(firebaseData.errorReason());
  }
}

void encenderLuzAutomaBlanco(){// enciendes luz blanca
  EncenderLed(255,255,255);

}
void ApagarLuzAutomaBlanco(){ //con esta intruccion obligas al led a apagarce ya sea si esta en modo manual o automatico
  pixels.clear(); // 
  pixels.show(); //
}