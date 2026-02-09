#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ---------- Délai d'envoi des données sur Node-RED ----------
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 5000; // 5 secondes

// ---------- WIFI + MQTT ----------
const char* ssid = "Basile";
const char* password = "45nxb6a7anvx9w3";
const char* mqtt_server = "mqtt.ci-ciad.utbm.fr";

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- OLED ----------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ---------- Capteurs ----------
Adafruit_BME280 bme;
Adafruit_CCS811 ccs;

// ---------- LEDs ----------
#define LED_PIN 25
#define NUM_LEDS 8
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------- Min/Max par capteur ----------
struct Range { float min; float max; };
Range ranges[] = {
  {10, 30},       // temperature
  {10, 95},      // humidite
  {950, 1050},   // pression
  {0, 10},       // vent
  {400, 2000},   // co2
  {0, 600}       // tvoc
};

// ---------- Index des capteurs ----------
enum Sensors {TEMP=0, HUM, PRES, VENT, CO2, TVOC};

// ---------- CO2 ----------
uint16_t eCO2 = 400;
uint16_t tvocValue = 0;
float co2Filtered = 400.0;       // moyenne glissante
#define CO2_SMOOTH 0.85
#define CO2_BUZZER_ON 1050
#define CO2_BUZZER_OFF 1000

// ---------- Buzzer ----------
#define BUZZER_PIN 26
bool buzzerManuel = false;
bool alerteCO2 = false;

// ---------- Ignorer l'alarme ----------
bool muteCO2 = false;
unsigned long muteStart = 0;
const unsigned long MUTE_DURATION = 60000; // 1 minute

// ---------- Vent ----------
#define HALL_PIN 33
#define ANEMO_DIAM 0.28
#define ANEMO_CIRC (3.1416 * ANEMO_DIAM)
#define WIND_INTERVAL 100
#define WIND_TIMEOUT 500
#define WIND_SCALE 1.0
#define WIND_ALPHA 0.2
#define WIND_HYST 0.8

volatile uint32_t pulseCount = 0;
uint32_t lastMeasure = 0;
uint32_t lastPulseTime = 0;
float windSpeed = 0.0;
float windPrev = 0.0;

// ---------- Mode LED ----------
String ledMode = "off"; // peut être : temperature, humidite, pression, vent, co2, tvoc, off

// ---------- WIFI ----------
void setupWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

// ----- LED bar dégradé gauche->droite -----
void displayScale(float value, Range r) {
  float t = (value - r.min) / (r.max - r.min);
  t = constrain(t, 0, 1);

  int ledsOn = round(t * NUM_LEDS);

  for (int i = 0; i < NUM_LEDS; i++) {
    if (i < ledsOn) {
      float ratio = (float)i / (NUM_LEDS - 1);
      int R = round(255 * ratio);
      int G = round(255 * (1 - ratio));
      int B = 0;
      strip.setPixelColor(i, R, G, B);
    } else {
      strip.setPixelColor(i, 0, 0, 0);
    }
  }
  strip.show();
}

// ----- ISR Hall -----
void IRAM_ATTR hallISR() {
  pulseCount++;
  lastPulseTime = millis();
}

// ----- MQTT Callback -----
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  // BUZZER
  if (String(topic) == "meteo/groupe7/cmd/buzzer") {
    if (msg == "ON")  buzzerManuel = true;
    if (msg == "OFF") buzzerManuel = false;
    if (msg == "STOP") {
      muteCO2 = true;
      muteStart = millis();
    }
  }

  // LED mode
  if (String(topic) == "meteo/groupe7/led/mode") {
    ledMode = msg; // stocke le mode LED courant
  }
}

// ----- Reconnect MQTT -----
void reconnectMQTT() {
  while (!client.connected()) {
    if (client.connect("ESP32_Groupe7")) {
      client.subscribe("meteo/groupe7/cmd/#");
      client.subscribe("meteo/groupe7/led/mode");
    } else {
      delay(2000);
    }
  }
}

// ----- Setup -----
void setup() {
  setupWifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  Serial.begin(115200);
  Wire.begin(21, 22);

  // OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) while(1);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Capteurs
  if(!bme.begin(0x76)) while(1);
  if(!ccs.begin()) while(1);

  // LEDs
  strip.begin();
  strip.setBrightness(30);
  strip.show();

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Anémomètre
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  lastMeasure = millis();
  lastPulseTime = millis();
}

// ----- Loop -----
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  uint32_t now = millis();

  // ----- Lecture BME280 -----
  float temp = bme.readTemperature();
  float hum  = bme.readHumidity();
  float press = bme.readPressure() / 100.0;

  // ----- Lecture CCS811 -----
  if(ccs.available() && !ccs.readData()) {
    eCO2 = ccs.geteCO2();
    tvocValue = ccs.getTVOC();
  }

  // ----- Moyenne glissante CO2 -----
  co2Filtered = CO2_SMOOTH * co2Filtered + (1.0 - CO2_SMOOTH) * eCO2;

  // ----- Buzzer CO2 -----
  if(!alerteCO2 && co2Filtered >= CO2_BUZZER_ON) alerteCO2 = true;
  else if(alerteCO2 && co2Filtered <= CO2_BUZZER_OFF) alerteCO2 = false;

  if (muteCO2 && millis() - muteStart >= MUTE_DURATION) muteCO2 = false;

  if (buzzerManuel || (alerteCO2 && !muteCO2)) digitalWrite(BUZZER_PIN, HIGH);
  else digitalWrite(BUZZER_PIN, LOW);

  // ----- Vent -----
  if(now - lastMeasure >= WIND_INTERVAL) {
    float dt = (now - lastMeasure) / 1000.0;
    uint32_t pulses = pulseCount;
    pulseCount = 0;

    float rawSpeed = (pulses * ANEMO_CIRC) / dt / WIND_SCALE;

    if(rawSpeed > windSpeed + WIND_HYST) windSpeed += WIND_HYST;
    else if(rawSpeed < windSpeed - WIND_HYST) windSpeed -= WIND_HYST;

    windSpeed = WIND_ALPHA * windPrev + (1.0 - WIND_ALPHA) * windSpeed;
    windPrev = windSpeed;

    lastMeasure = now;
  }

  if(now - lastPulseTime > WIND_TIMEOUT) {
    windSpeed *= 0.5;
    if(windSpeed < 0.01) windSpeed = 0;
  }

  // ----- OLED -----
  display.clearDisplay();
  display.setCursor(0,0);
  display.print("T: "); display.print(temp,1); display.println(" C");
  display.print("H: "); display.print(hum,0); display.println(" %");
  display.print("P: "); display.print(press,0); display.println(" hPa");
  display.print("CO2: "); display.print((int)co2Filtered); display.println(" ppm");
  display.print("TVOC: "); display.print(tvocValue); display.println(" ppb");
  display.print("Vent: "); display.print(windSpeed,1); display.println(" m/s");
  display.display();

  // ----- LED dynamique -----
  if(ledMode == "temperature") displayScale(temp, ranges[TEMP]);
  else if(ledMode == "humidite") displayScale(hum, ranges[HUM]);
  else if(ledMode == "pression") displayScale(press, ranges[PRES]);
  else if(ledMode == "vent") displayScale(windSpeed, ranges[VENT]);
  else if(ledMode == "co2") displayScale(co2Filtered, ranges[CO2]);
  else if(ledMode == "tvoc") displayScale(tvocValue, ranges[TVOC]);
  else if(ledMode == "off") {
    strip.clear();
    strip.show();
  }

  // ----- Publication Node-RED -----
  if (millis() - lastSend >= SEND_INTERVAL) {
    lastSend = millis();
    client.publish("meteo/groupe7/temperature", String(temp).c_str());
    client.publish("meteo/groupe7/humidite", String(hum).c_str());
    client.publish("meteo/groupe7/pression", String(press).c_str());
    client.publish("meteo/groupe7/co2", String((int)co2Filtered).c_str());
    client.publish("meteo/groupe7/tvoc", String(tvocValue).c_str());
    client.publish("meteo/groupe7/vent/freq", String(windSpeed).c_str());
  }

  delay(200);
}
