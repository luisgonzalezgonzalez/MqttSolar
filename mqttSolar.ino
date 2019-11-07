#include <ESP32Ping.h>
#include <ping.h>
#include "OneWire.h" 
#include "DallasTemperature.h"
/* 
Arduino code for heltec ESP-32 with OLED (www.heltec.cn)
After connection is made to WIFI a connection is made to an MQTT server
A subscription to the "temp" topic is made
MQTT Payload received is display
Works in conjunction with pubtemp.py example for Raspberry Pi & DS18B20 temperature probe

esp_temp_display_mqtt.ino 1.00 
---------------------------------------------------------------------------------                                                                                
 Luis Gonzalez luis.gonzalez@gmail.com
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                                                       
                                                                                  
 Revision History                                                                  
 V1.00 - Created       
 ----------------------------------------------------------------------------------
*/

#include <Wire.h>  // Only needed for Arduino 1.6.5 and earlier
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"` 
//#include "SH1106Wire.h" //If you need the 1106 driver then replace above line with this line

#define DISPLAY_HEIGHT 64
#define DISPLAY_WIDTH  128

SSD1306  display(0x3c, 4, 15);
//SH1106Wire display(ADDRESS, SDA, SDC); //If you need the 1106 driver then replace above line with this line

const char* ssid     = "wifissid";          // your network SSID (name of wifi network)
const char* password = "wifipassword";     // your network password
const char* mqtt_server = "192.168.1.16"; // your mqtt server ip
const int mqtt_port = 1883;                // your mqtt server port
const char* mqtt_topic_1 = "/termo/on";           // topic (don't change this)
const char* mqtt_topic_2 = "/termo/potencia"; 
const char* mqtt_topic_3 = "/energia/termo";           // topic (don't change this)
const char* mqtt_topic_4 = "/termo/temperatura"; // topic (don't change this)

const int reintentosWifi = 30;

const boolean debug = true;

bool activo_loop = false;
String ip = "";
String text1 = "";
String text2 = "0%";
String text3 = "0W";
const uint16_t potencia[11]={0,17000,22000,25600,28800,32000,36000,39100,43000,48000,65535};
//0 STOP
//17000    200w
//22000    400W
//25600    600W
//28800    800W
//32000   1000W
//36000   1200W
//39100   1400W
//43000   1600W
//48000   1800W
//65535   2000W



// Pin conexion Sondas Temperatura
const int oneWirePin = 23;

//configuracion pines pwm
uint8_t pin_pwm = 12;

uint16_t delay_cambio_potencia = 2000;

uint16_t potencia_actual = 0;          // a value from 0 to 65535 representing the pwm value
uint16_t potencia_destino = 0;
float temperatura_agua=0;

OneWire oneWire(oneWirePin);
DallasTemperature tempSensor(&oneWire);

WiFiClient client;
PubSubClient mqtt_client(client);





/******** void publisher***********************
    function that publish to the broker
    receives the message and topic
 *                                              *
 ***********************************************/
void publisher(char* topic, String topublish) {
  //USE_SERIAL.println("connected");
  // Once connected, publish an announcement...
  int length = topublish.length();
  char bufferdata[length];
  topublish.toCharArray(bufferdata,length+1);
  //client.publish("/opends/pwm", bufferdata);
  mqtt_client.publish(topic, bufferdata);
}

uint16_t pwm_calc(uint16_t pwm_val) {
  // Calc value with sin and cos
  double value;
  if (pwm_val >= 90) {
    value = (( (1 + cos((180-pwm_val)*3.1416/180))) /2 ) * 65535 ;
  } else {
    value = (( sin(pwm_val*3.1416/180) /2 ) * 65535) ;
  }
  if (pwm_val == 180) value = 65535;
  return ((uint16_t)value);
}


void println_oled(String txtPrint) {
  // Print to the screen
  display.setFont(ArialMT_Plain_10); // create more fonts at http://oleddisplay.squix.ch/
  display.clear();
  display.println(txtPrint);
  // Draw it to the internal screen buffer
  display.drawLogBuffer(0, 0);
  // Display it on the screen
  display.display();
  delay(500);
}

void print_oled(String txtPrint) {
  // Print to the screen
  display.setFont(ArialMT_Plain_10); // create more fonts at http://oleddisplay.squix.ch/
  display.clear();
  display.print(txtPrint);
  // Draw it to the internal screen buffer
  display.drawLogBuffer(0, 0);
  // Display it on the screen
  display.display();
  delay(500);
}

void callback(char* topic, byte * payload, unsigned int  length) {
  int i;
  String strPayload;
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //display.setFont(ArialMT_Plain_24); // create more fonts at http://oleddisplay.squix.ch/
  strPayload="";
    
  for (int i = 0; i < length; i++) {
    strPayload+=(char)payload[i];
  }
  if(String(topic)==String(mqtt_topic_1)){
    Serial.println("Es el topic /termo/on");
    if(String(strPayload)==String("1")){
      Serial.println("Enciendo el termo");
      publisher("/termo/estado", "1");
      text1="ON" ;
    }
    if(String(strPayload)==String("0")){
      Serial.println("Apago el termo");
      text1="OFF";
      publisher("/termo/estado", "0");
      
    }
    text1=ip ;
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, ip);
    display.setFont(ArialMT_Plain_24);
  }
  else{
//    Serial.print("NO es el topic 1");
//    Serial.println();
//    Serial.print("topic=" + String(topic));
//    Serial.println();
//    Serial.print("mqtt_topic_1=" + String(mqtt_topic_1));
//    Serial.println();
    //display.drawString(0, 0, text1);
  }
  text1=ip ;
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, ip);
  display.setFont(ArialMT_Plain_24);
  
  if(String(topic)==String(mqtt_topic_2)){
    Serial.println("Es el topic /termo/potencia");
    if(strPayload.toInt()>10){
      potencia_destino = 10;
    }
    else if(strPayload.toInt()<0){
      potencia_destino = 0;
    }
    else{
      potencia_destino =  strPayload.toInt();
    }
  }
  
  if(String(topic)==String(mqtt_topic_3)){
    Serial.println("Es el topic 3");
    display.drawString(0, 40, strPayload + "W");
    text3=strPayload + String("W");
    //compruebo si la potencia destino es superior a 2, en ese casi no hay consumo la bajo a dos inmediatamente
    if(strPayload.toInt()<180 && potencia_actual>2){
      //en este caso es termo ha llegado al tope de calentamiento, bajo la potencia del termo para no castigar al dimmer
      potencia_destino = 2;
    }
  }
  else{
//    Serial.print("NO es el topic 3");
//    Serial.println();
//    Serial.print("topic=" + String(topic));
//    Serial.println();
//    Serial.print("mqtt_topic_3=" + String(mqtt_topic_3));
//    Serial.println();
    display.drawString(0, 40, text3);
  }
  

  if(potencia_destino>potencia_actual){
    potencia_actual+=1;
    Serial.println("Subo Potencia");
    Serial.println("Aplico nivel " + String(potencia_actual) + "=" + String(potencia[potencia_actual]));
    ledcWrite(0,potencia[potencia_actual]);
    publisher("/termo/potencia_aplicada", String(potencia_actual));
    text2="UP " + String(potencia_actual*10) + "% " + String(int(temperatura_agua)) + "ยบ"; 
    display.drawString(0, 20, text2);
    display.display();
    delay(delay_cambio_potencia);
  }
  else if(potencia_destino<potencia_actual){
      potencia_actual=potencia_destino;
      Serial.println("Bajo Potencia");
      Serial.println("Aplico nivel " + String(potencia_actual) + "=" + String(potencia[potencia_actual]));
      ledcWrite(0,potencia[potencia_actual]);
      publisher("/termo/potencia_aplicada", String(potencia_actual));
      text2="DOWN " + String(potencia_actual*10) + "% " + String(int(temperatura_agua)) + "ยบ"; 
      display.drawString(0, 20, text2);
      display.display();
      delay(delay_cambio_potencia);
  }
  else{
    Serial.println("Potencia Correcta");
    text2=String(potencia_actual*10) + + "% " + String(int(temperatura_agua)) + "ยบ"; 
    display.drawString(0, 20, text2);
    display.display();
  }



  
  
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println();

//inicicalizo el sensor de temperatura
  tempSensor.begin();

  // Initialize channels 
  // channels 0-15, resolution 1-16 bits, freq limits depend on resolution
  Serial.println("INICIALIZO PWM");
  ledcSetup(0, 1000, 16); // 1 kHz PWM, 8-bit resolution
  ledcAttachPin(pin_pwm, 0); // assign pins to chanel
  Serial.println("pin_pwm=" + String(pin_pwm));
  ledcWrite(0, potencia_actual);  // Write new pwm value


  
  
  
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50);
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  display.init();
  Serial.println("Inicio Display y Software");
  display.setLogBuffer(5, 30);
  display.flipScreenVertically();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10); // create more fonts at http://oleddisplay.squix.ch/
  println_oled("Connecting to SSID: ");
  println_oled(ssid);
  WiFi.begin(ssid, password);
  // attempt to connect to Wifi network:
  int temporal=0;
  while (WiFi.status() != WL_CONNECTED && temporal<reintentosWifi) {
    print_oled(".");
    // wait 1 second for re-trying
    delay(1000);
    temporal+=1;
  }
  if(temporal==reintentosWifi){
    Serial.println("maximo numero intentos, reiniciamos ESP");
    ESP.restart();
  }
  ip = WiFi.localIP().toString().c_str();
  Serial.println("IP=" + ip);
  println_oled("");
  println_oled("Success!");
  delay(1000);
  Serial.println("establezco el server MQTT");
  mqtt_client.setServer(mqtt_server, mqtt_port);
  //Serial.println("Conecto al mqtt");
  //mqtt_client.connect("termo");
  Serial.println("Establezco el callback MQTT");
  mqtt_client.setCallback(callback);
  Serial.println("Callback");
  activo_loop=true;
}

void reconnect() {
  //ante sde nada compruebo que haya ping al servidor
  IPAddress ip (192, 168, 1, 16); // The remote ip to ping
  int contador=0;
  boolean salDelBucle=false;
  while(!salDelBucle){
    bool ret = Ping.ping(ip);
    if(ret){
      int avg_time_ms = Ping.averageTime();
      Serial.print("SUCCESS! RTT = ");
      Serial.print(avg_time_ms);
      Serial.println(" ms");
      salDelBucle=true;
    } 
    else {
      Serial.print("Ping " + String(contador) + " Perdido");
      contador=contador+1;
      if(contador>30){
        Serial.println("Reinicio por reintentos de PING");
        ESP.restart();
      }
    }
  }
