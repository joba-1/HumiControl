#include <Arduino.h>

// HumiControl, Joachim Banzhaf, 2019, GPLv2

// for WifiManager from https://github.com/tzapu/WiFiManager
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
char ap_ssid[33];  // AP SSID with measured values
#ifndef NAME
  #define NAME "HumiControl"
#endif
const char basename[] = NAME;

// for webserver
#include <ESP8266mDNS.h>
ESP8266WebServer web_server(80);
const char msgTemplate[] = "<!DOCTYPE html><html><head>"
  "<title>%s Humidity Control</title>"
  "<meta http-equiv='refresh' content='10'>"
  "</head><body><h1>%s</h1><h2>Humidity Control</h2><table>"
  "<tr><td style='width:50%%;'>Current Time</td><td style='float:right;'>%s</td><td style='width:20%%'>UTC</td></tr>"
  "<tr><td>Uptime</td><td style='float:right;'>%u:%02u:%02u</td><td>d:h:m</td></tr>"
  "<tr><td>Soil Moisture</td><td style='float:right;'>%0.2f</td><td>%%</td></tr>"
  "<tr><td>Air Humidity</td><td style='float:right;'>%0.2f</td><td>%%</td></tr>"
  "<tr><td>Temperature</td><td style='float:right;'>%0.2f</td><td>&deg;C</td></tr>"
  "<tr><td>Pressure</td><td style='float:right;'>%0.2f</td><td>mBar</td></tr>"
  "<tr><td><form action='/on'><button style='width:90%%;'>Fan On</button></form></td>"
  "<td colspan='2'><form action='/off'><button style='width:90%%;'>Fan Off</button></form></td></tr>"
  "<tr><form action='/speed'><td><input type='range' id='speed' min='0' max='100' value='%u' name='percent'></td>"
  "<td colspan='2'><button style='width:90%%;'>Speed <span id='value'></span></button></td></form></tr>"
  "</table><p/><a href='/version'>Firmware version</a><script>"
  "var s=document.getElementById(\"speed\"); var v=document.getElementById(\"value\");"
  "v.innerHTML=s.value; s.oninput=function(){v.innerHTML=this.value;}"
  "</script></body></html>";

// for online update
#include <WiFiClient.h>
#include <ESP8266HTTPUpdateServer.h>
#ifndef VERSION
  #define VERSION "0.1"
#endif
#define VERDATE __DATE__ " " __TIME__
const char versionTemplate[] = "<!DOCTYPE html><html><head>"
  "<title>%s Humidity Control</title>"
  "</head><body><h1>%s</h1><h2>Humidity Control</h2><table>"
  "<tr><td>Version</td><td>" VERSION "</td><td>" VERDATE "</td></tr>"
  "<tr><td>Author</td><td>Joachim Banzhaf</td><td>joachim.banzhaf@gmail.com</td></tr>"
  "<tr><td>License</td><td>GPL V2</td><td>(c) 2019</td></tr>"
  "<form method='POST' action='/update' enctype='multipart/form-data'>"
  "<tr><td>Update</td><td><input type='file' name='update'></td>"
  "<td><input type='submit' value='Update'></td></tr></form>"
  "<form method='POST' action='/reset'>"
  "<tr><td>Reset</td><td>Humidity Control</td><td><button>Now</button></td></tr></form>"
  "</table><p/>Hint: Update with a new firmware.bin can be done with"
  "<p/>curl -vF 'image=@firmware.bin' %s.local/update"
  "<p/><a href='/'>Humidity Control</a> as <a href='/json'>JSON</a></body></html>";
char msg[sizeof(msgTemplate) + 3*sizeof(basename) + 20];
ESP8266HTTPUpdateServer esp_updater;

// for UTC time
#include <NTPClient.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;
NTPClient ntpTime(ntpUDP, "europe.pool.ntp.org", 0, 600000);

// for json page
const char jsonTemplate[] = "{\"name\":\"%s\",\"version\":\"%s\","
  "\"monitor\":{\"utc_time\":{\"value\":\"%s\",\"units\":\"h:m:s\"},"
  "\"soil_moisture\":{\"value\":%0.2f,\"units\":\"percent\"},"
  "\"air_humidity\":{\"value\":%0.2f,\"units\":\"percent\"},"
  "\"temperature\":{\"value\":%0.2f,\"units\":\"celsius\"},"
  "\"pressure\":{\"value\":%0.2f,\"units\":\"mbar\"},"
  "\"speed\":{\"value\":%u,\"units\":\"percent\"}}}";

// for button press to reset wifi settings
#define BUTTON_PIN     D3
#define BUTTON_PRESSED LOW

// for LED status
#include <Ticker.h>
Ticker ticker;
#define LED_PIN D7
#define LED_ON  HIGH
#define LED_OFF LOW

// for capacitive moisture sensor calibration data (raw A0 reading in water/air)
#define A0_WET 875
#define A0_DRY 445
double soil_moisture;

// for fan control
#define FANPOWER_PIN D5
#define FANRPM_PIN   D6
#define MIN_PWM     700
static uint16_t curr_pwm = 0;

// for BME280 API from https://github.com/BoschSensortec/BME280_driver.git
#include <Wire.h>
#include "bme280.h"
#define I2C_SCL_PIN D1
#define I2C_SDA_PIN D2

struct bme280_dev dev;
struct bme280_data comp_data;
bool   sensor_found;


void tick() {
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // toggle led state
}


// called when WiFiManager enters configuration mode: change blink frequency
void configModeCallbackSlowBlink( WiFiManager *myWiFiManager ) {
  ticker.attach(0.8, tick);
}


// Interface function required for the Bosch driver
void user_delay_ms(uint32_t period) {
  delay(period);
}


// Interface function required for the Bosch driver
int8_t user_i2c_read( uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
  int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr | 0b10000000);
  if( (rslt = Wire.endTransmission(false)) == 0 ) {
    if( Wire.requestFrom(dev_id, len) == len ) {
      while( len-- ) {
        *(reg_data++) = Wire.read();
      }
    }
    else {
      rslt = 3;
    }
  }

  return rslt;
}


// Interface function required for the Bosch driver
int8_t user_i2c_write( uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len ) {
  Wire.beginTransmission(dev_id);
  Wire.write(reg_addr | 0b10000000);
  while( len-- ) {
    Wire.write(*(reg_data++));
  }
  return Wire.endTransmission(); // 0 == ok
}


uint16_t pwm2percent( uint16_t pwm ) {
  if( pwm <= MIN_PWM ) return 0;
  pwm -= MIN_PWM;
  return (pwm * 100 + (PWMRANGE - MIN_PWM) / 2) / (PWMRANGE - MIN_PWM);
}


uint16_t percent2pwm( uint16_t percent ) {
  if ( percent == 0 ) return 0;
  return MIN_PWM + (percent * (PWMRANGE - MIN_PWM) + 50) / 100;
}


double readSoilMoisture() {
  return map(constrain( analogRead(A0), A0_DRY, A0_WET ), A0_WET, A0_DRY, 0, 10000) / 100.0;
}

void print_sensor_data( struct bme280_data *comp_data, double soil_percent ) {
#ifdef BME280_FLOAT_ENABLE
  Serial.printf("%0.2f °C,  %0.2f mBar,  %0.2f %% Air Humidity,  %0.2f %% Soil Moisture,  %u %% Fan Speed\r\n",
    comp_data->temperature, comp_data->pressure/100, comp_data->humidity,
    soil_percent, pwm2percent(curr_pwm));
#else
  printf("%ld, %ld, %ld\r\n",
    comp_data->temperature, comp_data->pressure, comp_data->humidity);
#endif
}


int8_t bme280_prepare_forced_mode( struct bme280_dev *dev ) {
  uint8_t settings_sel;

  /* Recommended mode of operation: Indoor navigation */
  dev->settings.osr_h = BME280_OVERSAMPLING_1X;
  dev->settings.osr_p = BME280_OVERSAMPLING_16X;
  dev->settings.osr_t = BME280_OVERSAMPLING_2X;
  dev->settings.filter = BME280_FILTER_COEFF_16;

  settings_sel = BME280_OSR_PRESS_SEL
               | BME280_OSR_TEMP_SEL
               | BME280_OSR_HUM_SEL
               | BME280_FILTER_SEL;

  return bme280_set_sensor_settings(settings_sel, dev);
}


int8_t bme280_forced_mode( struct bme280_dev *dev, struct bme280_data *comp_data ) {
  bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
  /* Wait for the measurement to complete and print data max @25Hz */
  dev->delay_ms(40);
  return bme280_get_sensor_data(BME280_ALL, comp_data, dev);
}


void respond() {
  digitalWrite(LED_PIN, LED_OFF);
  web_server.sendHeader("Connection", "close");

  uint16_t speed = pwm2percent(curr_pwm);
  unsigned long now = millis();
  snprintf(msg, sizeof(msg), msgTemplate, basename, basename,
    ntpTime.getFormattedTime().c_str(), now / (1000*60*60*24),
    (now / (1000*60*60)) % 24, (now / (1000*60)) % 60, soil_moisture,
    comp_data.humidity, comp_data.temperature, comp_data.pressure/100, speed);

  web_server.send(200, "text/html", msg);
  digitalWrite(LED_PIN, LED_ON);
}


void respondVersion() {
  digitalWrite(LED_PIN, LED_OFF);
  web_server.sendHeader("Connection", "close");

  snprintf(msg, sizeof(msg), versionTemplate, basename, basename, basename);

  web_server.send(200, "text/html", msg);
  digitalWrite(LED_PIN, LED_ON);
}


void respondJson() {
  digitalWrite(LED_PIN, LED_OFF);
  web_server.sendHeader("Connection", "close");

  snprintf(msg, sizeof(msg), jsonTemplate, basename, VERSION,
    ntpTime.getFormattedTime().c_str(), soil_moisture, comp_data.humidity,
    comp_data.temperature, comp_data.pressure/100, pwm2percent(curr_pwm));

  web_server.send(200, "application/json", msg);
  digitalWrite(LED_PIN, LED_ON);
}


void respondSpeed() {
  digitalWrite(LED_PIN, LED_OFF);
  web_server.sendHeader("Connection", "close");

  uint16_t speed = web_server.arg("percent").toInt();
  if( speed <= 100 ) {
    analogWrite(FANPOWER_PIN, percent2pwm(100));
    curr_pwm = percent2pwm(speed);
    Serial.printf("Set speed to %lu (%lu%%)\n", curr_pwm, speed);
    delay(100);
    analogWrite(FANPOWER_PIN, curr_pwm);
  }
  else {
    Serial.println("No valid speed found");
  }

  web_server.sendHeader("Location", "/");
  web_server.send(302, "text/plain", "ok");
  digitalWrite(LED_PIN, LED_ON);
}


void setupWifi() {
  // WiFi.hostname(basename);
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallbackSlowBlink);
  wifiManager.setTimeout(180); // try for 3 min
  if( !wifiManager.autoConnect(ap_ssid) ) {
    Serial.println("Failed to connect. Rebooting...");
    ESP.reset();
    delay(1000);
  }

  if( !MDNS.begin(basename) )
    Serial.println("MDNS.begin(hostname) failed");

  ntpTime.begin();

  esp_updater.setup(&web_server);

  web_server.on("/version", respondVersion);
  web_server.on("/json", respondJson);
  web_server.on("/speed", respondSpeed);
  web_server.on("/on", []() {
    digitalWrite(LED_PIN, LED_OFF);
    web_server.sendHeader("Connection", "close");

    curr_pwm = percent2pwm(100);
    Serial.printf("/on -> pwm = %u\n", curr_pwm);
    analogWrite(FANPOWER_PIN, curr_pwm);

    web_server.sendHeader("Location", "/");
    web_server.send(302, "text/plain", "ok");
    digitalWrite(LED_PIN, LED_ON);
  });
  web_server.on("/off", []() {
    digitalWrite(LED_PIN, LED_OFF);
    web_server.sendHeader("Connection", "close");

    curr_pwm = percent2pwm(0);
    Serial.printf("/off -> pwm = %u\n", curr_pwm);
    analogWrite(FANPOWER_PIN, curr_pwm);

    web_server.sendHeader("Location", "/");
    web_server.send(302, "text/plain", "ok");
    digitalWrite(LED_PIN, LED_ON);
  });
  web_server.on("/reset", []() {
    digitalWrite(LED_PIN, LED_OFF);
    web_server.sendHeader("Connection", "close");

    web_server.sendHeader("Connection", "close");
    web_server.send(200, "text/plain", "ok: reset\n");
    digitalWrite(LED_PIN, LED_ON);

    delay(200);
    ESP.restart();
  });
  web_server.onNotFound(respond);

  web_server.begin();
  MDNS.addService("http", "tcp", 80);
}


void setup() {
  Serial.begin(115200);

  // Fast led blink frequency during setup
  pinMode(LED_PIN, OUTPUT);
  ticker.attach(0.2, tick);

  pinMode(FANPOWER_PIN, OUTPUT);
  analogWriteFreq(100*1000);           // high ultrasonic or fan will beep
  analogWrite(FANPOWER_PIN, curr_pwm); // fan initially off

  // for bme280
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Setup BME280 for reading single values
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;

  soil_moisture = readSoilMoisture();

  sensor_found = false;
  if( bme280_init(&dev) == 0
   && bme280_prepare_forced_mode(&dev) == 0
   && bme280_forced_mode(&dev, &comp_data) == 0 ) {
    sensor_found = true;
    snprintf(ap_ssid, sizeof(ap_ssid)-1, "%s_Soil_%d_Air_%d", basename,
      int(soil_moisture+0.5), int(comp_data.humidity+0.5));
  }
  else {
    snprintf(ap_ssid, sizeof(ap_ssid)-1, "%s_S%d", basename, int(soil_moisture+0.5));
  }
  ap_ssid[sizeof(ap_ssid)-1] = '\0';

  // Setup WiFi
  setupWifi();

  // Configure button for resetting wifi settings
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Switch led on after setup
  ticker.detach();
  digitalWrite(LED_PIN, LED_ON);

  Serial.printf("\nStarted %s V" VERSION " - " VERDATE " with IP=%s\n",
    basename, WiFi.localIP().toString().c_str());
}


void handle_button() {
  static unsigned long pressed_since = 0;

  int button_state = digitalRead(BUTTON_PIN);
  if( button_state == BUTTON_PRESSED ) {
    unsigned long now = millis();
    if( now == 0 ) now++;
    if( pressed_since == 0 ) {
      pressed_since = now;
      digitalWrite(LED_PIN, LED_OFF);
      curr_pwm = percent2pwm(curr_pwm != 0 ? 0 : 100);
      analogWrite(FANPOWER_PIN, curr_pwm);
    }
    else if ( now - pressed_since > 5000 ) {
      digitalWrite(LED_PIN, LED_ON);
      while( digitalRead(BUTTON_PIN) == BUTTON_PRESSED ) {
        delay(10);
      }
      Serial.println("Deleting configuration.");
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      Serial.println("Rebooting...");
      ESP.reset();
      delay(1000);
    }
  }
  else { // not pressed
    if( pressed_since != 0 ) {
      pressed_since = 0;
      digitalWrite(LED_PIN, LED_ON);
    }
  }
}


void handle_sensor() {
  static unsigned long last_measurement = 0;

  unsigned long now = millis();
  if( now - last_measurement > 1000 ) {
    last_measurement = now;
    soil_moisture = readSoilMoisture();
    if( bme280_forced_mode(&dev, &comp_data) == 0 ) {
      print_sensor_data(&comp_data, soil_moisture);
    }
  }
}


void loop() {
  ntpTime.update();
  web_server.handleClient();
  handle_button();
  if( sensor_found ) {
    handle_sensor();
  }
  delay(10);
}
