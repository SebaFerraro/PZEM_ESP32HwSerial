#include <HardwareSerial.h>
#include <PZEM004T.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
//#include <BH1750FVI.h>
//#include <OneWire.h> 
//#include <DallasTemperature.h>
#include "Ticker.h"
//#include "DHTesp.h"

#define RXD1 23
#define TXD1 22
#define RXD2 16
#define TXD2 17
#define Chances 10
#define TOKEN_TB "OSGYLIQfRlRwwsoAgQVe"  //TOKEN de AUTH PLATAFORMA WEB
#define NAME "CMDCSensorETHP01"
#define MQTT_MAX_PACKET_SIZE 256
#define DELAYT 60000
#define CANT_VALUV 11

const char* ssid     = "10110";
const char* password = "T3cn0C10r";
static int Wconectado = 0;
const char* mqtt_server = "190.2.22.61";

//DHTesp dht;
TaskHandle_t tempTaskHandle = NULL;
WiFiClient espClient;
PubSubClient client(espClient);
//BH1750FVI LightSensor(BH1750FVI::k_DevModeContHighRes2);
//OneWire oneWire(DS18_DATA); 
//DallasTemperature ds18b20(&oneWire);


//PZEM004T pzem(10, 11);
HardwareSerial PzemSerial2(2);
HardwareSerial PzemSerial1(1);


PZEM004T pzem1(&PzemSerial2,RXD2,TXD2);
PZEM004T pzem2(&PzemSerial1,RXD1,TXD1);

IPAddress ip[2];
float v1 = 0;
float i1 = 0;
float p1 = 0;
float e1 = 0;
float v2 = 0;
float i2 = 0;
float p2 = 0;
float e2 = 0;
float s1 = 0;
float s2 = 0;
float pf1 =0;
float pf2 = 0;
float Valores1[3][CANT_VALUV]={0};
float Valores2[3][CANT_VALUV]={0};
float sum1[3]={0.0};
float prom1[3]={0.0};
float vmax1[3]={0.0};
float sum2[3]={0.0};
float prom2[3]={0.0};
float vmax2[3]={0.0};
uint8_t ivaloresuv1=0;
uint8_t bvaloresuv1=0;
uint8_t ivaloresuv2=0;
uint8_t bvaloresuv2=0;
uint32_t antes=0;
uint32_t despues=0;
long rssi = 0;
uint32_t FALLA1=0;
uint32_t FALLA2=0;
char buf[100];
  
uint8_t Promedio1(void){
  int num=0;
  FALLA1=1;
  sum1[0]=sum1[1]=sum1[2]=0.0;
  prom1[0]=prom1[1]=prom1[2]=0.0;
  vmax1[0]=vmax1[1]=vmax1[2]=0.0;
  Serial.println("------------------- PROMEDIO 1 -------------------------------");
  sprintf(buf,"Valores 1:V1M: %f I1M: %f P1M: %f\n", vmax1[0], vmax1[1], vmax1[2]);
  Serial.print (buf);
  sprintf(buf,"Promedio1: ivaloresuv1 %d  bvaloresuv1 %d \n",ivaloresuv1,bvaloresuv1);
  Serial.print (buf);
  uint8_t vals1 = ivaloresuv1;
  if (bvaloresuv1 == 1)
     vals1=CANT_VALUV;
  for (int v=0; v < vals1 ; v++){
     num++;
     sum1[0]=sum1[0]+Valores1[0][v];
     sum1[1]=sum1[1]+Valores1[1][v];
     sum1[2]=sum1[2]+Valores1[2][v];
     if (Valores1[0][v] > vmax1[0])
          vmax1[0]=Valores1[0][v];
     if (Valores1[1][v] > vmax1[1])
          vmax1[1]=Valores1[1][v];
     if (Valores1[2][v] > vmax1[2])
          vmax1[2]=Valores1[2][v];
     sprintf(buf,"Valores: V1 : %f  I1: %f  P1: %f V1M: %f I1M: %f P1M: %f Numero: %d\n", Valores1[0][v], Valores1[1][v], Valores1[2][v] , vmax1[0], vmax1[1], vmax1[2], v);
     Serial.print (buf);
  }
  if ( num > 0 ){
    prom1[0]=sum1[0]/num;
    prom1[1]=sum1[1]/num;
    prom1[2]=sum1[2]/num;
    sprintf(buf,"Promedio 1: %f  %f %f Suma: %f  %f %f Numero: %d\n",prom1[0],prom1[1],prom1[2],sum1[0],sum1[1],sum1[2],num);
    Serial.print (buf);
  }else{
    sprintf(buf,"Sin Promedio: no hay valores. Suma: %f  %f %f Numero: %d\n",sum1[0],sum1[1],sum1[2],num);
    Serial.print (buf);
  }
  return num;
}

uint8_t Promedio2(void){
  int num=0;
  FALLA2=1;
  sum2[0]=sum2[1]=sum2[2]=0.0;
  prom2[0]=prom2[1]=prom2[2]=0.0;
  vmax2[0]=vmax2[1]=vmax2[2]=0.0;
  Serial.println("------------------- PROMEDIO 2 -------------------------------");
  sprintf(buf,"Valores 2:V2M: %f I2M: %f P2M: %f\n", vmax2[0], vmax2[1], vmax2[2]);
  Serial.print (buf);
  sprintf(buf,"Promedio2: ivaloresuv2 %d  bvaloresuv2 %d \n",ivaloresuv2,bvaloresuv2);
  Serial.print (buf);
  uint8_t vals2 = ivaloresuv2;
  if (bvaloresuv2 == 1)
     vals2=CANT_VALUV;
  for (int v=0; v < vals2 ; v++){
     num++;
     sum2[0]=sum2[0]+Valores2[0][v];
     sum2[1]=sum2[1]+Valores2[1][v];
     sum2[2]=sum2[2]+Valores2[2][v];
     //sprintf(buf,"Valores MAX 2:V2M: %f I2M: %f P2M: %f\n", vmax2[0], vmax2[1], vmax2[2]);
     //Serial.print (buf);
     if (Valores2[0][v] > vmax2[0]){
          //Serial.println("Entra al if");
          //Serial.println(Valores2[0][v]);
          //Serial.println(vmax2[0]);
          //vmax2[0]=248.77;
          //Serial.println(vmax2[0]);
          vmax2[0]=Valores2[0][v];
     }
     if (Valores2[1][v] > vmax2[1])
          vmax2[1]=Valores2[1][v];
     if (Valores2[2][v] > vmax2[2])
          vmax2[2]=Valores2[2][v];
     sprintf(buf,"Valores: V2 : %f  I2: %f  P2: %f V2M: %f I2M: %f P2M: %f Numero: %d\n", Valores2[0][v], Valores2[1][v], Valores2[2][v] , vmax2[0], vmax2[1], vmax2[2], v);
     Serial.print (buf);
  }
  if ( num > 0 ){
    prom2[0]=sum2[0]/num;
    prom2[1]=sum2[1]/num;
    prom2[2]=sum2[2]/num;
    sprintf(buf,"Promedio 2: %f  %f %f Suma: %f  %f %f Numero: %d\n",prom2[0],prom2[1],prom2[2],sum2[0],sum2[1],sum2[2],num);
    Serial.print (buf);
  }else{
    sprintf(buf,"Sin Promedio 2: no hay valores. Suma: %f  %f %f Numero: %d\n",sum2[0],sum2[1],sum2[2],num);
    Serial.print (buf);
  }
  return num;
}


void AgregaVal1(float * valor){
      Valores1[0][ivaloresuv1]=valor[0];
      Valores1[1][ivaloresuv1]=valor[1];
      Valores1[2][ivaloresuv1]=valor[2];
      if(ivaloresuv1<(CANT_VALUV-1)){
       ivaloresuv1++;
      }else{
       ivaloresuv1=0;
       bvaloresuv1=1;
      }
    sprintf(buf,"Agrega Val1: %f %f %f   ival: %d\n",valor[0],valor[1],valor[2],ivaloresuv1);
    Serial.print (buf);
}

void AgregaVal2(float * valor){
      Valores2[0][ivaloresuv2]=valor[0];
      Valores2[1][ivaloresuv2]=valor[1];
      Valores2[2][ivaloresuv2]=valor[2];
      if(ivaloresuv2<(CANT_VALUV-1)){
       ivaloresuv2++;
      }else{
       ivaloresuv2=0;
       bvaloresuv2=1;
      }
    sprintf(buf,"Agrega Val2: %f %f %f   ival: %d\n",valor[0],valor[1],valor[2],ivaloresuv2);
    Serial.print (buf);
}


void EventoWifi(WiFiEvent_t event)
{
    Serial.printf("[Evento-Wifi] evento: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi conectedo");
      Serial.print("IP : ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi se pedio la conexion...");
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 inicio cliente..");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 cliente conectado a AP");
      break;
    default:      
      Serial.println("Evento WiFi desconocido.");
      break;
    }
}

void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(EventoWifi);
  WiFi.begin(ssid, password);
  Serial.println("Fin Wifi_init.");
}

void envia_mqtt(int res1, int res2,float v1, float v2, float vmax1, float vmax2, float i1, float i2, float imax1, float imax2, float p1, float p2, float pmax1, float pmax2, uint16_t e1, uint16_t e2, float pf1, float pf2, long rssi){
  if (Wconectado == 1){
    if(!client.connected()) {
      Serial.print("Conectando CIOR ThingsBoard node ...");
      if ( client.connect(NAME, TOKEN_TB, NULL) ) {
        Serial.println( "[OK]" );
      } else {
        Serial.print( "[ERROR] [ rc = " );
        Serial.print( client.state() );
      }
    }  
    if (client.connected()) {
      char attributes[200];
      int rsus=0;
      String srssi = String(rssi);
      
      String payload = "{";
      payload += "\"rssi\":";
      payload += rssi;
      payload += "}";
      
      payload.toCharArray( attributes, 200 );
      rsus=client.publish( "v1/devices/me/telemetry", attributes );
      Serial.print( "Publish : ");
      Serial.println(rsus);
      Serial.print( "Atributos : ");
      Serial.print( attributes );
      Serial.print( " LEN Atributos : ");
      Serial.println( sizeof(attributes)/sizeof(char) );
      
      if (res1){  
        String sv1 = String(v1);
        String svm1 = String(vmax1);
        String si1 = String(i1);
        String sim1 = String(imax1);
        String sp1 = String(p1);
        String spm1 = String(pmax1);
        String se1 = String(e1);
        String spf1 = String(pf1);
        
      
        //String stermica = String(hic);
        //String lux = String(lx);
        //String temp2 = String(t2);
                 
        payload = "{";
        payload += "\"v1\":";
        payload += sv1;
        payload += ",";
        payload += "\"i1\":";
        payload += si1;
        payload += ",";
        payload += "\"p1\":";
        payload += sp1;
        payload += ",";
        payload += "\"e1\":";
        payload += se1;
        payload += ",";
        payload += "\"pf1\":";
        payload += spf1;
        payload += "}";

        payload.toCharArray( attributes, 200 );
        rsus=client.publish( "v1/devices/me/telemetry", attributes );
        Serial.print( "Publish : ");
        Serial.println(rsus);
        Serial.print( "Atributos : ");
        Serial.print( attributes );
        Serial.print( " LEN Atributos : ");
        Serial.println( sizeof(attributes)/sizeof(char) );
        
        payload = "{";
        payload += "\"vmax1\":";
        payload += svm1;
        payload += ",";
        payload += "\"imax1\":";
        payload += sim1;
        payload += ",";
        payload += "\"pmax1\":";
        payload += spm1;
        payload += "}";

        payload.toCharArray( attributes, 200 );
        rsus=client.publish( "v1/devices/me/telemetry", attributes );
        Serial.print( "Publish : ");
        Serial.println(rsus);
        Serial.print( "Atributos Max: ");
        Serial.print( attributes );
        Serial.print( " LEN Atributos Max: ");
        Serial.println( sizeof(attributes)/sizeof(char) );
      }
      if (res2){
        String sv2 = String(v2);
        String svm2 = String(vmax2);
        String si2 = String(i2);
        String sim2 = String(imax2);
        String sp2 = String(p2);
        String spm2 = String(pmax2);
        String se2 = String(e2);
        String spf2 = String(pf2);
                 
        payload = "{";
        payload += "\"v2\":";
        payload += sv2;
        payload += ",";
        payload += "\"i2\":";
        payload += si2;
        payload += ",";
        payload += "\"p2\":";
        payload += sp2;
        payload += ",";
        payload += "\"e2\":";
        payload += se2;
        payload += ",";
        payload += "\"pf2\":";
        payload += spf2;
        payload += "}";
      
        payload.toCharArray( attributes, 200 );
        rsus=client.publish( "v1/devices/me/telemetry", attributes );
        Serial.print( "Publish : ");
        Serial.println(rsus);
        Serial.print( "Atributos : ");
        Serial.print( attributes );
        Serial.print( " LEN Atributos : ");
        Serial.println( sizeof(attributes)/sizeof(char) );

        payload = "{";
        payload += "\"vmax2\":";
        payload += svm2;
        payload += ",";
        payload += "\"imax2\":";
        payload += sim2;
        payload += ",";
        payload += "\"pmax2\":";
        payload += spm2;
        payload += "}";

        payload.toCharArray( attributes, 200 );
        rsus=client.publish( "v1/devices/me/telemetry", attributes );
        Serial.print( "Publish : ");
        Serial.println(rsus);
        Serial.print( "Atributos Max: ");
        Serial.print( attributes );
        Serial.print( " LEN Atributos Max: ");
        Serial.println( sizeof(attributes)/sizeof(char) );

       
      }
    }
  }
}


uint8_t pzem004_init(void){
  uint8_t pzr1, pzr2;
  pzr1=pzem1.setAddress(ip[0]);
  pzr2=pzem2.setAddress(ip[1]);
  Serial.println("Setea dir PZEMs...");
  if(pzr1 && pzr2){
        Serial.println("OK");
   }else{
        Serial.println("Error");
  }
  return pzr1 && pzr2;
}


uint8_t pzem004_read1(void){
    uint8_t n=0;
    Serial.println("Read PZEM1...");
    //while(n1<Chances){
      v1 = pzem1.voltage(ip[0]);
      i1 = pzem1.current(ip[0]);
      p1 = pzem1.power(ip[0]);
      e1 = pzem1.energy(ip[0]);
      Serial.print(".");
      delay(500);
      //n1++;
      if (v1 >= 0.0 && i1 >= 0.0 && p1 >= 0.0 && e1 >= 0.0)
         n=1;
     return n;
}

uint8_t pzem004_read2(void){
    uint8_t n=0;
    Serial.println("Read PZEM2...");
    //while(n1<Chances){
      v2 = pzem2.voltage(ip[1]);
      i2 = pzem2.current(ip[1]);
      p2 = pzem2.power(ip[1]);
      e2 = pzem2.energy(ip[1]);
      Serial.print(".");
      delay(500);
      //n1++;
      if (v2 >= 0.0 && i2 >= 0.0 && p2 >= 0.0 && e2 >= 0.0)
         n=1;
     return n;
}


void setup() {
  Serial.begin(115200);
 
  ip[0] = IPAddress(0, 0, 0, 1);
  ip[1] = IPAddress(0, 0, 0, 2);
  pzem004_init();
  delay(1000);
  client.setServer(mqtt_server, 1884);
  //dht.setup(DHT_DATA, DHTesp::DHT22);
  //LightSensor.begin();
  //ds18b20.begin();    
  Serial.println("Fin Setup()");
  
}

void loop() {
   float vpi1[3]={0};
   float vpi2[3]={0};
   //Serial.println("Loop");
   if (Wconectado == 0){
    Serial.println("Error No conectado wifi Wifi_init.");
    Wifi_init();
    delay(45000);
   }else{
      uint8_t resultado=pzem004_read1();
      if (resultado){
        Serial.println("Lectura correcta PZEM1.");
        vpi1[0]=v1;
        vpi1[1]=i1;
        vpi1[2]=p1;
        AgregaVal1(vpi1);
             
      }else{
        Serial.println("Lectura incorrecta PZEM1.");
      }
      resultado=pzem004_read2();
      if (resultado){
        Serial.println("Lectura correcta PZEM2.");
        vpi2[0]=v2;
        vpi2[1]=i2;
        vpi2[2]=p2;
        AgregaVal2(vpi2);
             
      }else{
        Serial.println("Lectura incorrecta PZEM2.");
      }
     delay(2000);
         
    despues=millis();
    sprintf(buf,"TIEMPOS : Despues %lu  Antes %lu  D-A %lu \n",despues ,antes,(despues - antes));
    Serial.print (buf);
    if((despues - antes) > DELAYT){
      antes=despues;
      int res1=Promedio1();
      if (res1 > 0){
        s1=prom1[0] * prom1[1];  
        if (s1 > 0){
          pf1=prom1[2]/s1;
        }else{
          pf1=0;
        }
      }
      int res2=Promedio2();
      if (res2 > 0){
        s2=prom2[0] * prom2[1];  
        if (s2 > 0){
          pf2=prom2[2]/s2;
        }else{
          pf2=0;
        }
      }
      
      rssi=WiFi.RSSI();
      Serial.print("RSSI");
      Serial.println(rssi);
      Serial.println("Envia MQTT:   ");
      envia_mqtt(res1,res2,prom1[0],prom2[0],vmax1[0],vmax2[0],prom1[1],prom2[1],vmax1[1],vmax2[1],prom1[2],prom2[2],vmax1[2],vmax2[2],e1,e2,pf1,pf2,rssi);
      Serial.print("Delay:   ");
      int tdelay=DELAYT - (despues - antes);
      Serial.print((despues - antes));
      Serial.print("  ");
      Serial.println(tdelay);
      
    }
  }
}
