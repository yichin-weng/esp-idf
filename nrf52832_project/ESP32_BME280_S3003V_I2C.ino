#include "ESP32_BME280_I2C.h"
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <time.h>

#define JST     3600* 9

const char* ssid = "Buffalo-G-1FF0";
const char* password = "kiyonolabJ412";


const char* server_google="script.google.com";
const char* key_google="AKfycbx5OWqGWO7ogujAZnFqAarcK0TJ9wU_xEMqDkK_ZN-fo78KX8Fp";

WiFiServer server(80);

WiFiClientSecure sslclient;

const uint8_t Address = 0x76;
const uint8_t sda = 21;
const uint8_t scl = 22;
const uint32_t frequency = 30000;

const int pin_ACDL = 5;
const int pin_MCDL = 4;

ESP32_BME280_I2C bme280i2c(Address, scl, sda, frequency);

time_t t,t0,tdiff;

void setup() {
  Serial.begin(115200);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();

  
  delay(1000); //Take some time to open up the Serial Monitor

  //Example Indoor navigation
  uint8_t t_sb = 0; //stanby 0.5ms
  uint8_t filter = 4; //IIR filter = 16
  uint8_t osrs_t = 2; //OverSampling Temperature x2
  uint8_t osrs_p = 5; //OverSampling Pressure x16
  uint8_t osrs_h = 1; //OverSampling Humidity x1
  uint8_t Mode = 3; //Normal mode

  bme280i2c.ESP32_BME280_I2C_Init(t_sb, filter, osrs_t, osrs_p, osrs_h, Mode);

  pinMode(pin_ACDL, OUTPUT);
  pinMode(pin_MCDL, OUTPUT);
  
  
  Wire.begin();
  
  delay(1000);

  configTime( JST, 0, "ntp.nict.jp", "ntp.jst.mfeed.ad.jp");

}

int IO_cal=0;
int IO_count=0;
static const char *wd[7] = {"Sun","Mon","Tue","Wed","Thr","Fri","Sat"};

void loop() {
  struct tm *tm;

  t = time(NULL);
  tm = localtime(&t);
  Serial.printf(" %04d/%02d/%02d(%s) %02d:%02d:%02d\n",
        tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
        wd[tm->tm_wday],
        tm->tm_hour, tm->tm_min, tm->tm_sec);



  
  //bme_get();
  double temperature, pressure, humidity;

  bme280i2c.Read_All(&temperature, &pressure, &humidity);

  char temp_c[10], hum_c[10], pres_c[10],co2_c[10],mac_address[64];
  sprintf(temp_c, "%2.2lf", temperature);
  sprintf(hum_c, "%2.2lf", humidity);
  sprintf(pres_c, "%4.2lf", pressure);
  sprintf(co2_c,"%d",getCO2ppm());

  Serial.println("-----------------------");
  uint8_t mac3[6];
  esp_read_mac(mac3, ESP_MAC_WIFI_STA);
  Serial.printf("[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
  sprintf(mac_address,"%02X:%02X:%02X:%02X:%02X:%02X", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
  
  uint8_t mac4[7];
  esp_read_mac(mac4, ESP_MAC_WIFI_SOFTAP);
  Serial.printf("[Wi-Fi SoftAP] Mac Address  = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);

  Serial.print("IP address = "); Serial.println(WiFi.localIP());
  
  Serial.print("Temperature = "); Serial.println(temp_c);
  Serial.print("Humidity = "); Serial.println(hum_c);
  Serial.print("Pressure = "); Serial.println(pres_c);
  Serial.print("CO2 conc. = "); Serial.println(co2_c);
  Serial.println("-----------------------");
  Serial.flush();


  //google===================================================================================
  String URL="https://script.google.com/macros/s/";
  URL += key_google;
  URL += "/exec?";
  URL += "&1_cell=";
  URL += mac_address;
  URL += "&2_cell=";
  URL += temp_c;
  URL += "&3_cell=";
  URL += hum_c;
  URL += "&4_cell=";
  URL += pres_c;
  URL += "&5_cell=";
  URL += co2_c;

  Serial.println(URL);
  Serial.println("Conneting to google_sever...");
  if(!sslclient.connect(server_google,443)){
    Serial.print("Failed to connect to ");Serial.println(server_google);
    Serial.println("");
  }
  else{
    Serial.print("Connected to ");Serial.println(server_google);
    sslclient.println("GET " + URL);
    //sslclient.println(URL);
    delay(1000);
    sslclient.stop();
    Serial.println("Finished to write");
  }
  //google===================================================================================


  if(IO_count!=0){
    tdiff=difftime(t,t0);
    if(tdiff > 900){
      IO_cal=0; 
      IO_count=0;
    }
  }
  
  if(IO_cal==0){
    digitalWrite(pin_ACDL, LOW);
    digitalWrite(pin_MCDL, LOW);
  }
  else{
    digitalWrite(pin_ACDL, HIGH);
    digitalWrite(pin_MCDL, LOW);
  }
  

  WiFiClient client = server.available();   // listen for incoming clients

  if (client) {                             // if you get a client,
    Serial.println("New Client.");           // print a message out the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();


            client.print("========================================================<br>");
            client.print("Time<br>");
            client.print("========================================================<br>");
            client.printf(" %04d/%02d/%02d(%s) %02d:%02d:%02d<br>",
                          tm->tm_year+1900, tm->tm_mon+1, tm->tm_mday,
                          wd[tm->tm_wday],
                          tm->tm_hour, tm->tm_min, tm->tm_sec);
            client.print("========================================================<br><br><br>");

            client.print("========================================================<br>");
            client.print("Network<br>");
            client.print("========================================================<br>");
            client.printf("[Wi-Fi Station] Mac Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]);
            client.print("<br>");
            client.printf("[Wi-Fi SoftAP] Mac Address  = %02X:%02X:%02X:%02X:%02X:%02X\r\n", mac4[0], mac4[1], mac4[2], mac4[3], mac4[4], mac4[5]);
            client.print("<br>");
            client.print("IP address = "); client.println(WiFi.localIP());
            client.print("<br>");
            client.print("========================================================<br><br><br>");
            
            client.print("========================================================<br>");
            client.print("Sensor<br>");
            client.print("========================================================<br>");
            client.print("Temperature = "); client.println(temp_c);
            client.print("<br>");
            client.print("Humidity = "); client.println(hum_c);
            client.print("<br>");
            client.print("Pressure = "); client.println(pres_c);
            client.print("<br>");
            client.print("CO2 conc. = "); client.println(co2_c);
            client.print("<br>");
            client.print("========================================================<br><br><br>");



            client.print("========================================================<br>");
            client.print("Calibration<br>");
            client.print("========================================================<br>");
            client.print("After clicking Start, please wait 15 minutes or until the CO2 conc. resets to 400 ppm.<br>");
            // the content of the HTTP response follows the header:
            client.print("<a href=\"/Y\">Start</a><br>");
            client.print("<a href=\"/N\">Stop</a><br>");
            client.print("Status: ");
            if(IO_cal==0){
              client.print("Calibration OFF<br>");
            }
            else{
              client.print("Calibration ON<br>");
              tdiff=difftime(t,t0);
              tm = localtime(&tdiff);
              client.printf("Elapsed time: %02d:%02d<br>",tm->tm_min, tm->tm_sec);
            }
            client.print("========================================================<br><br><br>");

            client.print("========================================================<br>");
            client.print("<a href=\"/\">Top page</a><br>");
            client.print("========================================================<br><br><br>");
            

            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("GET /Y")) {
          IO_cal=1;
          if(IO_count==0){
            t0=time(NULL);
            IO_count=1;               
          }
          
        }
        if (currentLine.endsWith("GET /N")) {
          IO_cal=0;
          IO_count=0;              
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("Client Disconnected.");
  }

  
  
  delay(10000);
}

//************** BME280 measure *************************
void bme_get() {
  double temperature, pressure, humidity;

  bme280i2c.Read_All(&temperature, &pressure, &humidity);

  char temp_c[10], hum_c[10], pres_c[10];
  sprintf(temp_c, "%2.2lf", temperature);
  sprintf(hum_c, "%2.2lf", humidity);
  sprintf(pres_c, "%4.2lf", pressure);

  Serial.println("-----------------------");
  Serial.print("Temperature = "); Serial.println(temp_c);
  Serial.print("Humidity = "); Serial.println(hum_c);
  Serial.print("Pressure = "); Serial.println(pres_c);
  Serial.println("-----------------------");
  Serial.flush();
}


//============================S300-3V===========================
int getCO2ppm() {
  byte tmpBuf[7];
  sendCommand('R');

  Wire.requestFrom(0x31, 7);
  
  for (int i = 0; Wire.available(); i++) {
    tmpBuf[i] = Wire.read();
    delay(1);
  }

  if (tmpBuf[0] != 0x08 ||
      tmpBuf[3] == 0xff ||
      tmpBuf[4] == 0xff ||
      tmpBuf[5] == 0xff ||
      tmpBuf[6] == 0xff) {
        return 0;
  }
      
  return (tmpBuf[1] << 8) | tmpBuf[2];
}

void sendCommand(byte val) {
  Wire.beginTransmission(0x31);
  Wire.write(val);
  Wire.endTransmission();
}

void sleep() {
  sendCommand('S');
  delay(4000);
}

void wakeup() {
  sendCommand('W');
  delay(6000);
}
