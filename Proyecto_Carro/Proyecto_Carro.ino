#include <ThingSpeak.h>
#include <DHT.h>
#include <DHT_U.h>

#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

//#include <CapacitiveSensor.h>
#include <UltraDistSensor.h>
#include <EEPROM.h>



//Pines para el motor 1
#define vel1 32 //enable1
#define forward_m1 33 //in1
#define reverse_m1 25 //in2
//Pines para el motor 2
#define vel2 14 //enable2
#define forward_m2 27//in3
#define reverse_m2 26 //in4

//Pines para el motor 3 (Cortadora)
#define vel3 5 //enable1
#define forward_m3 18//in1
#define reverse_m3 19 //in2

#define button_enc 16 //botón para encender y apagar motores
#define button_cap 17  //botón de sensor capacitivo
#define Trig 23 //Pin del trigger
#define Echo 22 //Pin del 
//#define Sensor_Cap1 34
//#define Sensor_Cap2 35

//Configuraciones para el PWM
const int frequency = 3000;
const int pwm_channel = 0;
const int resolution = 8;
int dutyCycle = 255; //Velocidad de los motores de 0 a 255

/*Definimos que el Pin de Datos del sensor estará conectado al pin 13 del ESP32*/
#define DHTPIN  13 
/*Definimos que el tipo de sensor que estamos utilizando es un DHT11*/
#define DHTTYPE DHT11// DHT 11
/*Se actualizan algunos parámetros del DHT11 con los puntos definidos anteriormente*/ 
DHT dht(DHTPIN, DHTTYPE);
/*Definimos como constantes Char las credenciales de acceso a la red WiFi*/
const char* ssid="Mi Wifi";
const char* password="NoMolestes";
/*Definimos las credenciales para la conexión a la plataforma*/
unsigned long channelID = 1850691;
const char* WriteAPIKey ="605ZB0HJ0MCSJW89";
/*Definimos el cliente WiFi que usaremos*/
WiFiClient cliente;


//Número de malezas
int n_malezas=0;

UltraDistSensor ultra;
int distance;

int val = 0; //val se emplea para almacenar el estado del boton
int state = 0; // 0 motores apagados, mientras que 1 motores encendidos.
int old_val = 0; // almacena el antiguo valor de val

//CapacitiveSensor sensor=CapacitiveSensor(Sensor_Cap1, Sensor_Cap2);

enum {encendido, avance, obstaculo, maleza, corte} estado ;

void IRAM_ATTR Encendido(){
  val= digitalRead(button_enc); // lee el estado del Boton
  if ((val == HIGH) && (old_val == LOW)){
  state=1-state;
  delay(10);
  }
  old_val = val; // valor antiguo 
  if (state==1){
   estado=avance; // envía el estado avance
  }
  else{
    estado=encendido; // envía el estado encendido
    }
}

void setup() {
  pinMode(vel1, OUTPUT);
  pinMode(forward_m1, OUTPUT);
  pinMode(reverse_m1, OUTPUT);
  pinMode(vel2, OUTPUT);
  pinMode(forward_m2, OUTPUT);
  pinMode(reverse_m2, OUTPUT);
  pinMode(vel3, OUTPUT);
  pinMode(forward_m3, OUTPUT);
  pinMode(reverse_m3, OUTPUT);
  pinMode(button_enc, INPUT);
  pinMode(button_cap, INPUT);

  ledcSetup(pwm_channel, frequency, resolution);
  ledcAttachPin(vel1, pwm_channel);
  ledcAttachPin(vel2, pwm_channel);
  ledcAttachPin(vel3, pwm_channel);
  ledcWrite(pwm_channel, dutyCycle);

  ultra.attach(Trig,Echo);
  Serial.begin(115200);
  dht.begin();
  /*Iniciamos la conexión a la red WiFi, y se imprimirán caracteres indicando el tiempo que tarda la conexión*/
  WiFi.begin(ssid,password);
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  /*Una vez conectado, se imprimirá una frase y se iniciará la conexión a la Plataforma usando el cliente definido anteriormente*/
  Serial.println("Conectado al WiFi");
  ThingSpeak.begin(cliente);
  delay(5000);

  estado=encendido;
  attachInterrupt(digitalPinToInterrupt(button_enc),Encendido, RISING); //Interrupción del botón encendido/avance
  

}

void medicion(){
  /*Realizamos la lectura de Temperatura y Humedad del sensor*/
  float temperatura= dht.readTemperature();
  float humedad = dht.readHumidity();
  /*Imprimimos los valores obtenidos en el terminal Serial*/
  Serial.print("Temperatura registrada: ");
  Serial.print(temperatura);
  Serial.println("°C");
  Serial.print("Humedad registrada: ");
  Serial.print(humedad);
  Serial.println("%");
  Serial.println("-----------------------------------------");
  /*Indicamos el orden de envío por campos o Field, en el orden definido de la plataforma, junto a los valores del sensor*/
  ThingSpeak.setField(1,temperatura);
  ThingSpeak.setField(2,humedad);
}


void loop() {
  switch(estado){
    case encendido:
    Serial.println("Encendido");
    //Todos los motores apagados.
    digitalWrite(forward_m1, LOW);
    digitalWrite(reverse_m1, LOW);
    digitalWrite(forward_m2, LOW);
    digitalWrite(reverse_m2, LOW);
    digitalWrite(forward_m3, LOW);
    digitalWrite(reverse_m3, LOW);
        break;
    
    case avance:
    Serial.println("Avance");
    //Enciende primer motor
    digitalWrite(forward_m1, LOW);
    digitalWrite(reverse_m1, HIGH);
    //Enciende segundo motor
    digitalWrite(forward_m2, LOW);
    digitalWrite(reverse_m2, HIGH);
    distance=ultra.distanceInCm(); //Calcula la distancia en que se encuentre un objeto en cm
    Serial.println(distance);
    if (distance<10){
      //Si un obstáculo se encuentra a una distancia menor a 10cm se detienen los motores y pasa al estado obstáculo.
      digitalWrite(forward_m1, LOW);
      digitalWrite(reverse_m1, LOW);
      digitalWrite(forward_m2, LOW);
      digitalWrite(reverse_m2, LOW);
      estado=obstaculo;
        break; 
      }


    if(digitalRead(button_cap)){
      Serial.println("Capacitivo");
      //Reducción de velocidad de motores hasta llegar a cero.
      while(dutyCycle>0){
        dutyCycle = dutyCycle - 51;
        ledcWrite(pwm_channel, dutyCycle);
        delay(200);
        }
      digitalWrite(forward_m1, LOW);
      digitalWrite(reverse_m1, LOW);
      digitalWrite(forward_m2, LOW);
      digitalWrite(reverse_m2, LOW);
      dutyCycle=255;
      estado=corte;  
      }
        break;

    case obstaculo:
    Serial.println("Obstáculo");
    //Se enciende los motores de las ruedas en retroceso.
    digitalWrite(forward_m1, HIGH);
    digitalWrite(reverse_m1, LOW);
    digitalWrite(forward_m2, HIGH);
    digitalWrite(reverse_m2, LOW);
    delay(1000); //Por un segundo
    //Se enciende el motor de la rueda izquierda para girar a la derecha
    digitalWrite(forward_m1, LOW);
    digitalWrite(reverse_m1, LOW);
    digitalWrite(forward_m2, LOW);
    digitalWrite(reverse_m2, HIGH);
    delay(1000); //Por un segundo
    estado=avance; //Cambio a estado de avance
        break;

    case maleza:
    Serial.println("Maleza");
    n_malezas++;
    /*Usamos un retardo de 5 segundos, y utilizamos la función Medición para la lectura de los sensores*/
    delay(5000);
    medicion();
    /*Hacemos la conexión y envío de datos a la plataforma, utilizando las credenciales definidas anteriormente*/
    ThingSpeak.writeFields(channelID,WriteAPIKey);
    /*Imprimimos una frase indicando el envío, y agregamos un retardo de 5 segundos*/
    Serial.println("Datos enviados a ThingSpeak!");
    delay(5000);
    estado=avance;
    
        break;

    case corte:
    Serial.println("Corte");
    ledcWrite(pwm_channel, dutyCycle);
    digitalWrite(forward_m3, LOW);
    digitalWrite(reverse_m3, HIGH);
    delay(2000);
    digitalWrite(forward_m3, LOW);
    digitalWrite(reverse_m3, LOW);
    estado=maleza;
        break;
    }
    
    

}
