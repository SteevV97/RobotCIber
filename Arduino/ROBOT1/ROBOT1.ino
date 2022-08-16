#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32SharpIR.h>

//MQTT
const char *mqtt_server = "192.168.1.6";
const int mqtt_port = 1883;
const char *mqtt_user = "";
const char *mqtt_pass = "";
const char *root_topic_subscribe = "Robot1/system";
const char *ultrasonicoIzq_topic_publish = "Robot1/UltrasonicoIzquierda";
const char *ultrasonicoDer_topic_publish = "Robot1/UltrasonicoDerecha";
const char *IR_topic_publish = "Robot1/IR";

//SENSORES
// sensor ultrasonico izquierdo
const int TriggerI = 4;   //4
const int EchoI = 2;   //16
// sensor ultrasonico derecho
const int TriggerD = 14 ;  //14
const int EchoD = 12;   //12
//sensor IR
const int pinIR = 27;
//MOTORES DC
//motor izquierda
int motor1Pin1 = 18; //18 in1 primero
int motor1Pin2 = 19; //19 in2
//motor derecha
int motor1Pin3 = 16; //16 in3 primero
int motor1Pin4 = 17; //17 in4
//
int tiempoMov = 100;
//MOTOR BRUSHLESS


String _topic;
String _payload;
//Red Wifi
char* ssid = "CELERITY_VALLEJO";
char* password = "11063797";

WiFiClient espClient;
PubSubClient client(espClient);
//Charr array para envio de datos
char ultrasonicoIzq[25];
char ultrasonicoDer[25];
char IR[25];

ESP32SharpIR sensor( ESP32SharpIR::GP2Y0A21YK0F, pinIR);


void setup() {
  Serial.begin(115200);
  // sensor ultrasonico izquierdo
  pinMode(TriggerI, OUTPUT);
  pinMode(EchoI, INPUT);
  digitalWrite(TriggerI, LOW);
  // sensor ultrasonico derecho
  pinMode(TriggerD, OUTPUT);
  pinMode(EchoD, INPUT);
  digitalWrite(TriggerD, LOW);
  //motores
  pinMode (motor1Pin1, OUTPUT);
  pinMode (motor1Pin2, OUTPUT);
  pinMode (motor1Pin3, OUTPUT);
  pinMode (motor1Pin4, OUTPUT);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  sensor.setFilterRate(0.1f);
  delay(500);
  parar();

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  if (client.connected()) {
    //lectura del utrasonico izquierda
    long distanceIzq;
    distanceIzq = lecturaUltrasonico(TriggerI, EchoI);
    Serial.print("sensor ultrasonico izquierdo: ");
    Serial.println(distanceIzq);
    String(distanceIzq).toCharArray(ultrasonicoIzq, 25);

    //lectura del utrasonico derecho
    long distanceDer;
    distanceDer = lecturaUltrasonico(TriggerD, EchoD);
    Serial.print("sensor ultrasonico derecho: ");
    Serial.println(distanceDer);
    String(distanceDer).toCharArray(ultrasonicoDer, 25);

    // Lectura IR
    float D_cm = sensor.getDistanceFloat(); //lectura de distancia
    String(D_cm).toCharArray(IR, 25);
    delay(100);

    //topicos
    client.publish(ultrasonicoIzq_topic_publish, ultrasonicoIzq);
    client.publish(ultrasonicoDer_topic_publish, ultrasonicoDer);
    client.publish(IR_topic_publish, IR);
    delay(500);

  }
  client.loop();
}



void setup_wifi() {
  delay(3000);
  Serial.println();
  Serial.print("conectado a ssid");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("Conectado a red wifi!");
  Serial.println("Dirección IP:");
  Serial.println(WiFi.localIP());

}

void reconnect() {
  while (!client.connected()) {

    Serial.print("Intentndo conexión mqtt...");
    String clientId = "ESP8266";
    clientId += String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("Conectado!");
      client.subscribe("Robot1/control");
      if (client.subscribe(root_topic_subscribe)) {
        Serial.println("Suscription ok");
      } else {
        Serial.println("Suscription fallo");
      }
    } else {
      Serial.print("falló :C con error ->");
      Serial.print(client.state());
      Serial.println("intentamos en 5 seg");
      delay(500);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String content = "";
  if ((char)payload[0] == 'U') {
    Serial.println("Up");
    adelante();
  }else if((char)payload[0] == 'D'){
    Serial.println("Down");
    atras();
  }else if((char)payload[0] == 'L'){
    Serial.println("Left");
    izquierda();
  }else if((char)payload[0] == 'R'){
    Serial.println("Right");
    derecha();
  }else if((char)payload[0] == 'G'){
    Serial.println("Gun");
    activarArma();
  }
}

//MOVIMIENTOS MOTOR
void adelante() {
  digitalWrite (motor1Pin1, HIGH);
  digitalWrite (motor1Pin2, LOW);
  digitalWrite (motor1Pin3, HIGH);
  digitalWrite (motor1Pin4, LOW);
  delay(tiempoMov);
  parar();
}
void atras() {
  digitalWrite (motor1Pin1, LOW);
  digitalWrite (motor1Pin2, HIGH);
  digitalWrite (motor1Pin3, LOW);
  digitalWrite (motor1Pin4, HIGH);
  delay(tiempoMov);
  parar();
}
void derecha() {
  digitalWrite (motor1Pin1, HIGH);
  digitalWrite (motor1Pin2, LOW);
  digitalWrite (motor1Pin3, LOW);
  digitalWrite (motor1Pin4, HIGH);
  delay(tiempoMov);
  parar();
}
void izquierda() {
  digitalWrite (motor1Pin1, LOW);
  digitalWrite (motor1Pin2, HIGH);
  digitalWrite (motor1Pin3, HIGH);
  digitalWrite (motor1Pin4, LOW);
  delay(tiempoMov);
  parar();
}
void parar() {
  digitalWrite (motor1Pin1, LOW);
  digitalWrite (motor1Pin2, LOW);
  digitalWrite (motor1Pin3, LOW);
  digitalWrite (motor1Pin4, LOW);
}

///MOVIMIENTO Brushles
void activarArma() {

}
void detenerArma() {

}

//funcion para el IR
float distanciaIR()
{
  long suma = 0;
  for (int i = 0; i < 20; i++)
  {
    suma = suma + analogRead(34);
  }
  float adc = suma / 20;
  float distancia_cm = 17569.7 * pow(adc, -1.2062);
  return (distancia_cm);
}


//Lectura Distancia Ultrasonico
long lecturaUltrasonico(int pinTrig, int pinEcho) {
  long tiempo;
  long distancia;
  digitalWrite(pinTrig, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrig, LOW);
  tiempo = pulseIn(pinEcho, HIGH);
  distancia = tiempo / 59;
  return distancia;
}
