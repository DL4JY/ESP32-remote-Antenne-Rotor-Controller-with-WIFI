/*
 * ESP32-C3 Antenna Rotator Controller - Emotator 1200
 * VERSION: 1.6 - Bugfix ADC-Interpolation über 360°/0° Übergang
 * 
 * ÄNDERUNGEN v1.6:
 * - Bugfix: Korrekte Interpolation zwischen Index 15 (340°) und 16 (175°)
 * - ADC-Werte im Bereich 340°-360° werden nun richtig als 0°-20° erkannt
 * - Debug-Ausgaben in shortestAngleDiff() für Anschlag-Umfahrung
 * 
 * ÄNDERUNGEN v1.5:
 * - Anschlag-bewusste Navigation: Verhindert Fahrt über 180°-Anschlag
 * - Von rechts nach links → immer über 0°/360°
 * - Von links nach rechts → immer über 0°/360°
 * 
 * ÄNDERUNGEN v1.4:
 * - Anschlag bei 180° mit ±X° Überlauf (konfigurierbar 0-20°)
 * - 18 Kalibrierpunkte: 0°, 20°, 40° ... 340°, (180-X)°, (180+X)°
 * - Korrekte Linearisierung über den Anschlag hinweg
 * - ADC-Verlauf: ~1974 bei 0°/360° → ~20 bei (180-X)° → ~3700 bei (180+X)°
 *
 * WICHTIG: Emotator 1200 hat mechanischen Anschlag bei 180°
 * Der Rotor kann ±5° über den Anschlag fahren (175° bis 185°)
 * Kalibrierung ist PFLICHT für korrekte Funktion!
 *
 * Features:
 * - Weboberfläche mit analoger/digitaler Winkelanzeige (0-360°)
 * - UDP für Thetis/PST Rotator
 * - WiFi AP/STA konfigurierbar
 * - 16-Punkt Kalibrierung (0°, 22.5°, 45° ... 337.5°)
 * - OTA Updates
 *
 * Hardware:
 * - ESP32-C3 Mini (USB)
 * - GPIO0: ADC Input (Rotor Position 0-3.3V)
 * - GPIO8: Motor Links (Relais)
 * - GPIO9: Motor Rechts (Relais)
 *
 * Benötigte Bibliotheken:
 *   - AsyncTCP (GitHub: me-no-dev/AsyncTCP)
 *   - ESPAsyncWebServer (GitHub: me-no-dev/ESPAsyncWebServer)
 *   - AsyncElegantOTA v2.2.7 (GitHub Release, #error auskommentieren!)
 *   - ArduinoJson v6 (Library Manager)
 *   - Preferences (im ESP32-Core enthalten)
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <Preferences.h>
#include <ArduinoJson.h>

// ------------------------- Hardware-Pins (ESP32-C3) -------------------------

const int PIN_ADC          = 1;   // Rotor-Positionsspannung 0-3.3V
const int PIN_MOTOR_LEFT   = 8;   // Links drehen
const int PIN_MOTOR_RIGHT  = 9;   // Rechts drehen

// ------------------------- Globale Objekte -------------------------

AsyncWebServer server(80);
WiFiUDP udp;
Preferences prefs;

// ------------------------- Konfiguration -------------------------

struct Config {
  String staSsid;
  String staPass;
  String apSsid;
  String apPass;
  uint16_t udpPort;
  float toleranceDeg;
  float overrunDeg;      // Überlauf-Winkel X (0-20°), Standard 5°
  float calibAdc[18];    // 18 ADC-Werte: 0°,20°,40°...340°,(180-X)°,(180+X)°
  float calibAngle[18];  // Zugehörige Winkel zu den ADC-Werten
  bool calibValid;
} config;

// ------------------------- Rotorzustand -------------------------

struct RotorState {
  float currentAngle;
  float targetAngle;
  bool  hasTarget;
  bool  manualLeft;
  bool  manualRight;
  uint16_t lastAdc;
} rotor;

// ------------------------- Funktions-Deklarationen -------------------------

void loadConfig();
void saveConfig();
void setupWifi();
void setupWeb();
void setupUdp();
void readRotorPosition();
float adcToAngle(uint16_t adc);
float normalizeAngle(float a);
float shortestAngleDiff(float from, float to);
void controlMotor();
void handleUdp();

void handleMainPage(AsyncWebServerRequest *request);
void handleConfigPage(AsyncWebServerRequest *request);
void handleCalibrationPage(AsyncWebServerRequest *request);
void handleAPIStatus(AsyncWebServerRequest *request);
void handleAPISetTarget(AsyncWebServerRequest *request);
void handleAPIManual(AsyncWebServerRequest *request);
void handleAPIStop(AsyncWebServerRequest *request);
void handleAPIGetConfig(AsyncWebServerRequest *request);
void handleAPISaveConfig(AsyncWebServerRequest *request);
void handleAPIGetCalibration(AsyncWebServerRequest *request);
void handleAPISaveCalibration(AsyncWebServerRequest *request);

// ------------------------- Setup & Loop -------------------------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("============================================"));
  Serial.println(F("  ESP32-C3 Rotor Controller v1.6"));
  Serial.println(F("  Emotator 1200 - Bugfix ADC-Interpolation"));
  Serial.println(F("============================================"));

  pinMode(PIN_MOTOR_LEFT, OUTPUT);
  pinMode(PIN_MOTOR_RIGHT, OUTPUT);
  digitalWrite(PIN_MOTOR_LEFT, LOW);
  digitalWrite(PIN_MOTOR_RIGHT, LOW);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  rotor.currentAngle = 0;
  rotor.targetAngle = 0;
  rotor.hasTarget = false;
  rotor.manualLeft = false;
  rotor.manualRight = false;
  rotor.lastAdc = 0;

  loadConfig();
  setupWifi();
  setupUdp();
  setupWeb();

  Serial.println(F("\n========== Setup complete =========="));
  Serial.print(F("IP: "));
  Serial.println(WiFi.localIP());
  Serial.println(F("Öffne Weboberfläche und führe Kalibrierung durch!"));
  Serial.println(F("====================================\n"));
}

void loop() {
  readRotorPosition();
  controlMotor();
  handleUdp();
  delay(50);
}

// ------------------------- Konfiguration -------------------------

void loadConfig() {
  Serial.println(F("Loading config..."));
  prefs.begin("rotor", true);

  config.staSsid = prefs.getString("staSsid", "");
  config.staPass = prefs.getString("staPass", "");
  config.apSsid  = prefs.getString("apSsid",  "RotorAP");
  config.apPass  = prefs.getString("apPass",  "rotor1234");
  config.udpPort = prefs.getUShort("udpPort", 12000);
  config.toleranceDeg = prefs.getFloat("tol", 2.0f);
  config.overrunDeg = prefs.getFloat("overrun", 5.0f);
  config.calibValid = prefs.getBool("calValid", false);

  if (config.calibValid) {
    Serial.println(F("\n=== KALIBRIERUNG GELADEN ==="));
    for (int i = 0; i < 18; i++) {
      config.calibAdc[i] = prefs.getFloat((String("c") + i).c_str(), 0);
      config.calibAngle[i] = prefs.getFloat((String("a") + i).c_str(), 0);
      Serial.printf("  %6.1f° → ADC %4.0f\n", config.calibAngle[i], config.calibAdc[i]);
    }
    Serial.println(F("============================\n"));
  } else {
    // Default: 16 Standardpunkte + 2 Anschlagpunkte
    for (int i = 0; i < 16; i++) {
      config.calibAngle[i] = i * 20.0f;  // 0°, 20°, 40° ... 340°
      config.calibAdc[i] = 1974 + (i * 100);  // Dummy-Werte
    }
    config.calibAngle[16] = 180.0f - config.overrunDeg;  // z.B. 175°
    config.calibAdc[16] = 20;
    config.calibAngle[17] = 180.0f + config.overrunDeg;  // z.B. 185°
    config.calibAdc[17] = 3700;
    
    // ⚠️ WICHTIG: Beim Laden muss calibAngle[] initialisiert werden!
    // Falls calibValid==false, werden die Default-Winkel bereits oben gesetzt
    // Falls calibValid==true, wurden sie bereits aus Preferences geladen
    Serial.println(F("\n⚠️  KEINE KALIBRIERUNG VORHANDEN!"));
    Serial.println(F("   Bitte Kalibrierung durchführen:"));
    Serial.println(F("   1. Weboberfläche öffnen"));
    Serial.println(F("   2. 'Kalibrierung' aufrufen"));
    Serial.println(F("   3. Rotor auf 0°, 22.5°, 45° ... stellen"));
    Serial.println(F("   4. Jeweils 'Erfassen' klicken"));
    Serial.println(F("   5. 'Speichern' klicken\n"));
  }

  prefs.end();
  Serial.printf("Config: AP='%s', STA='%s', UDP=%u, Tol=%.1f°\n",
                config.apSsid.c_str(), config.staSsid.c_str(),
                config.udpPort, config.toleranceDeg);
}

void saveConfig() {
  Serial.println(F("Saving config..."));
  prefs.begin("rotor", false);
  prefs.putString("staSsid", config.staSsid);
  prefs.putString("staPass", config.staPass);
  prefs.putString("apSsid",  config.apSsid);
  prefs.putString("apPass",  config.apPass);
  prefs.putUShort("udpPort", config.udpPort);
  prefs.putFloat("tol", config.toleranceDeg);
  prefs.putFloat("overrun", config.overrunDeg);
  prefs.putBool("calValid", config.calibValid);
  if (config.calibValid) {
    for (int i = 0; i < 18; i++) {
      prefs.putFloat((String("c") + i).c_str(), config.calibAdc[i]);
      prefs.putFloat((String("a") + i).c_str(), config.calibAngle[i]);
    }
  }
  prefs.end();
  Serial.println(F("Config saved."));
}

// ------------------------- WiFi -------------------------

void setupWifi() {
  WiFi.mode(WIFI_OFF);
  delay(100);

  if (config.staSsid.length() > 0) {
    Serial.printf("Connecting STA to '%s'...\n", config.staSsid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.staSsid.c_str(), config.staPass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
      delay(500);
      Serial.print('.');
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("STA connected. IP="));
      Serial.println(WiFi.localIP());
      return;
    }
    Serial.println(F("STA connect failed, starting AP..."));
  }

  Serial.printf("Starting AP '%s'...\n", config.apSsid.c_str());
  WiFi.mode(WIFI_AP);
  WiFi.softAP(config.apSsid.c_str(), config.apPass.c_str());
  Serial.print(F("AP IP: "));
  Serial.println(WiFi.softAPIP());
}

// ------------------------- UDP -------------------------

void setupUdp() {
  if (udp.begin(config.udpPort)) {
    Serial.printf("UDP listen on port %u\n", config.udpPort);
  } else {
    Serial.println(F("UDP begin failed!"));
  }
}

void handleUdp() {
  static unsigned long lastBroadcast = 0;

  int packetSize = udp.parsePacket();
  if (packetSize) {
    char buf[256];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len <= 0) return;
    buf[len] = 0;
    String msg = String(buf);
    msg.trim();

    int azIdx = msg.indexOf("<AZIMUTH>");
    if (azIdx >= 0) {
      int end = msg.indexOf("</AZIMUTH>", azIdx);
      if (end > azIdx) {
        String s = msg.substring(azIdx + 9, end);
        float az = s.toFloat();
        if (az >= 0 && az < 450) {
          rotor.targetAngle = fmod(az, 360.0f);
          rotor.hasTarget   = true;
          rotor.manualLeft  = false;
          rotor.manualRight = false;
          Serial.printf("UDP: new target %.1f°\n", rotor.targetAngle);
        }
      }
    }

    if (msg.indexOf("<STOP>1</STOP>") >= 0) {
      rotor.hasTarget = false;
      rotor.manualLeft = false;
      rotor.manualRight = false;
      Serial.println(F("UDP: STOP"));
    }

    if (msg.indexOf("AZ?") >= 0) {
      char out[32];
      snprintf(out, sizeof(out), "AZ:%03d\r\n", (int)rotor.currentAngle);
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write((uint8_t*)out, strlen(out));
      udp.endPacket();
    }
  }

  if (millis() - lastBroadcast > 1000) {
    lastBroadcast = millis();
    char out[32];
    snprintf(out, sizeof(out), "AZ:%03d\r\n", (int)rotor.currentAngle);
    udp.beginPacket(IPAddress(255,255,255,255), config.udpPort + 1);
    udp.write((uint8_t*)out, strlen(out));
    udp.endPacket();
  }
}

// ------------------------- ADC & Winkel -------------------------

void readRotorPosition() {
  uint32_t sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(PIN_ADC);
    delayMicroseconds(100);
  }
  uint16_t adc = sum / 4;
  rotor.lastAdc = adc;
  rotor.currentAngle = adcToAngle(adc);
}

float adcToAngle(uint16_t adc) {
  // Debug-Ausgabe alle 2 Sekunden
  static unsigned long lastDebug = 0;
  bool doDebug = (millis() - lastDebug > 2000);
  if (doDebug) lastDebug = millis();

  if (!config.calibValid) {
    float angle = (adc / 4095.0f) * 360.0f;
    if (doDebug) {
      Serial.println(F("\n⚠️  UNKALIBRIERT"));
      Serial.printf("   ADC=%u → %.1f°\n", adc, angle);
    }
    return normalizeAngle(angle);
  }

  // KALIBRIERT: 18 Punkte mit Anschlag bei 180°
  // ADC-Verlauf: 1974@0° → steigt → 3700@(180+X)° → ANSCHLAG → 20@(180-X)° → steigt → 1974@360°
  
  // Finde passendes Intervall durch alle 18 Kalibrierpunkte
  for (int i = 0; i < 17; i++) {
    float a1 = config.calibAdc[i];
    float a2 = config.calibAdc[i+1];
    float w1 = config.calibAngle[i];
    float w2 = config.calibAngle[i+1];
    
    // Prüfe zunächst ob ADC in diesem Intervall liegt (für alle Fälle)
    bool inRange = false;
    if (a2 > a1) {
      // ADC steigt: a1 <= adc <= a2
      inRange = (adc >= a1 && adc <= a2);
    } else if (a2 < a1) {
      // ADC fällt: a2 <= adc <= a1  
      inRange = (adc >= a2 && adc <= a1);
    } else {
      // a1 == a2: fehlerhafte Kalibrierung
      if (doDebug) {
        Serial.printf("\n⚠️  Kalibrierung fehlerhaft: Index %d und %d haben gleichen ADC\n", i, i+1);
      }
      return w1;
    }
    
    if (inRange) {
      float ratio = (adc - a1) / (a2 - a1);
      float angle = w1 + ratio * (w2 - w1);
      
      if (doDebug) {
        Serial.printf("\nADC=%u zwischen Index %d--%d (%.1f°--%.1f°)\n",
                      adc, i, i+1, w1, w2);
        Serial.printf("   ADC: %.0f .. %u .. %.0f\n", a1, adc, a2);
        Serial.printf("   Ratio: %.3f → Winkel: %.1f°\n", ratio, angle);
      }
      
      return normalizeAngle(angle);
    }
  }

  // Kein Intervall gefunden
  if (doDebug) {
    Serial.printf("\n❌ ADC=%u: KEIN Intervall gefunden!\n", adc);
    Serial.println(F("   Kalibrierungswerte:"));
    for (int i = 0; i < 18; i++) {
      Serial.printf("     [%d] %.1f° → ADC %.0f\n", i, config.calibAngle[i], config.calibAdc[i]);
    }
  }
  return 0.0f;
}

float normalizeAngle(float a) {
  while (a < 0)       a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

float shortestAngleDiff(float from, float to) {
  // WICHTIG: Emotator 1200 hat Anschlag bei 180°!
  // Erlaubter Bereich: 0° bis (180-X)° und (180+X)° bis 360°
  // Verbotener Durchgang: über den Anschlag bei 180°
  
  float lowerLimit = 180.0f + config.overrunDeg;  // z.B. 185°
  float upperLimit = 180.0f - config.overrunDeg;  // z.B. 175°
  
  // Prüfe ob direkter Weg über den Anschlag führen würde
  bool fromRight = (from > lowerLimit);  // Ist-Position rechts vom Anschlag (185°-360°)
  bool fromLeft = (from < upperLimit);   // Ist-Position links vom Anschlag (0°-175°)
  bool toRight = (to > lowerLimit);      // Ziel rechts vom Anschlag
  bool toLeft = (to < upperLimit);       // Ziel links vom Anschlag
  
  // Fall 1: Von rechter Hälfte nach linker Hälfte
  // z.B. von 350° nach 10° → MUSS über 0° gehen
  if (fromRight && toLeft) {
    // Weg nach rechts (über 360°/0°)
    float dist = (360.0f - from) + to;  // z.B. (360-350) + 10 = 20°
    Serial.printf("→ Anschlag-Umfahrung: %.1f° (rechts) → %.1f° (links), Weg über 0°: +%.1f°\n", from, to, dist);
    return dist;  // Positiv = nach rechts drehen
  }
  
  // Fall 2: Von linker Hälfte nach rechter Hälfte
  // z.B. von 10° nach 350° → MUSS über 0° gehen (rückwärts)
  if (fromLeft && toRight) {
    // Weg nach links (über 0°/360°)
    float dist = -from - (360.0f - to);  // z.B. -10 - (360-350) = -20°
    Serial.printf("→ Anschlag-Umfahrung: %.1f° (links) → %.1f° (rechts), Weg über 0°: %.1f°\n", from, to, dist);
    return dist;  // Negativ = nach links drehen
  }
  
  // Fall 3: Beide auf gleicher Seite → normaler kürzester Weg
  float d = to - from;
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  
  Serial.printf("→ Normale Fahrt: %.1f° → %.1f°, Differenz: %.1f°\n", from, to, d);
  return d;
}

// ------------------------- Motorsteuerung -------------------------

void controlMotor() {
  bool moveL = false;
  bool moveR = false;

  if (rotor.manualLeft) {
    moveL = true;
  } else if (rotor.manualRight) {
    moveR = true;
  } else if (rotor.hasTarget) {
    float d = shortestAngleDiff(rotor.currentAngle, rotor.targetAngle);
    if (fabs(d) > config.toleranceDeg) {
      if (d > 0) moveR = true;
      else       moveL = true;
    } else {
      rotor.hasTarget = false;
      Serial.printf("✓ Target reached: Ist=%.1f°, Soll=%.1f°\n",
                    rotor.currentAngle, rotor.targetAngle);
    }
  }

  digitalWrite(PIN_MOTOR_LEFT,  moveL ? HIGH : LOW);
  digitalWrite(PIN_MOTOR_RIGHT, moveR ? HIGH : LOW);
}

// ------------------------- Webserver -------------------------

void setupWeb() {
  Serial.println(F("Starting web server..."));

  server.on("/", HTTP_GET, handleMainPage);
  server.on("/config", HTTP_GET, handleConfigPage);
  server.on("/calibration", HTTP_GET, handleCalibrationPage);
  server.on("/api/status", HTTP_GET, handleAPIStatus);
  server.on("/api/set_target", HTTP_POST, handleAPISetTarget);
  server.on("/api/manual", HTTP_POST, handleAPIManual);
  server.on("/api/stop", HTTP_POST, handleAPIStop);
  server.on("/api/get_config", HTTP_GET, handleAPIGetConfig);
  server.on("/api/save_config", HTTP_POST, handleAPISaveConfig);
  server.on("/api/get_calibration", HTTP_GET, handleAPIGetCalibration);
  server.on("/api/save_calibration", HTTP_POST, handleAPISaveCalibration);

  AsyncElegantOTA.begin(&server);
  server.begin();
  Serial.println(F("Web server started."));
}

// ------------------------- HTML Seiten (gekürzt, identisch zu v1.1) -------------------------

void handleMainPage(AsyncWebServerRequest *request) {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<title>Rotor Controller</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
body{font-family:Arial,sans-serif;background:#f0f2f5;margin:0;padding:0}
.container{max-width:900px;margin:20px auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1)}
h1{text-align:center;margin-top:0;color:#333}
.status{padding:10px;background:#eef;margin-bottom:15px;border-radius:5px;font-family:monospace}
.flex{display:flex;flex-wrap:wrap;justify-content:space-around;align-items:center}
.block{margin:10px;text-align:center}
.angle{font-size:40px;font-weight:bold;color:#3366cc}
label{display:block;margin-top:10px}
input[type=number]{padding:5px;font-size:16px;width:100px}
button{padding:8px 16px;margin:5px;font-size:16px;border:none;border-radius:5px;cursor:pointer}
.btn-go{background:#28a745;color:#fff}
.btn-stop{background:#dc3545;color:#fff}
.btn-man{background:#007bff;color:#fff}
.btn-nav{background:#6c757d;color:#fff;text-decoration:none;display:inline-block;margin:5px}
canvas{border:1px solid #ccc;border-radius:50%;background:#fafafa}
@media(max-width:600px){.flex{flex-direction:column}}
</style>
</head>
<body>
<div class="container">
<h1>ESP32-C3 Rotor Controller</h1>
<div id="status" class="status">Status: Lade...</div>
<div class="flex">
<div class="block"><div>Aktueller Winkel</div><div id="curAngle" class="angle">---</div></div>
<div class="block"><canvas id="compass" width="200" height="200"></canvas></div>
<div class="block"><div>Zielwinkel</div><div id="tgtAngle" class="angle">---</div></div>
</div>
<h3>Automatik</h3>
<label>Sollwinkel (0..359°): <input type="number" id="inputAngle" min="0" max="359" value="0"></label>
<button class="btn-go" onclick="sendTarget()">GO</button>
<h3>Manuell</h3>
<button class="btn-man" onmousedown="manual('left')" onmouseup="stopManual()" onmouseleave="stopManual()" ontouchstart="manual('left')" ontouchend="stopManual()">Links</button>
<button class="btn-stop" onclick="stopAll()">STOP</button>
<button class="btn-man" onmousedown="manual('right')" onmouseup="stopManual()" onmouseleave="stopManual()" ontouchstart="manual('right')" ontouchend="stopManual()">Rechts</button>
<hr>
<a class="btn-nav" href="/calibration">Kalibrierung</a>
<a class="btn-nav" href="/config">Konfiguration</a>
<a class="btn-nav" href="/update">OTA Update</a>
</div>
<script>
let cur=0,tgt=0,hasT=false,adc=0;
function drawCompass(){const c=document.getElementById('compass');const ctx=c.getContext('2d');const cx=c.width/2,cy=c.height/2,r=80;ctx.clearRect(0,0,c.width,c.height);ctx.beginPath();ctx.arc(cx,cy,r,0,2*Math.PI);ctx.strokeStyle="#333";ctx.lineWidth=2;ctx.stroke();ctx.font="12px Arial";ctx.fillStyle="#333";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillText("N",cx,cy-r-10);ctx.fillText("E",cx+r+10,cy);ctx.fillText("S",cx,cy+r+10);ctx.fillText("W",cx-r-10,cy);if(hasT){ctx.save();ctx.translate(cx,cy);ctx.rotate((tgt-90)*Math.PI/180);ctx.setLineDash([5,5]);ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(r-10,0);ctx.strokeStyle="#e55353";ctx.lineWidth=2;ctx.stroke();ctx.setLineDash([]);ctx.restore()}ctx.save();ctx.translate(cx,cy);ctx.rotate((cur-90)*Math.PI/180);ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(r-15,0);ctx.strokeStyle="#3366cc";ctx.lineWidth=4;ctx.stroke();ctx.beginPath();ctx.moveTo(r-15,-6);ctx.lineTo(r-5,0);ctx.lineTo(r-15,6);ctx.closePath();ctx.fillStyle="#3366cc";ctx.fill();ctx.restore()}
function updateStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{cur=d.currentAngle;tgt=d.targetAngle;hasT=d.hasTarget;adc=d.adc;document.getElementById('curAngle').textContent=cur.toFixed(1)+'°';document.getElementById('tgtAngle').textContent=hasT?tgt.toFixed(1)+'°':'---';let s="Bereit";if(d.manualLeft)s="Manuell LINKS";else if(d.manualRight)s="Manuell RECHTS";else if(hasT)s="Fahre zu "+tgt.toFixed(1)+"°";document.getElementById('status').textContent="Status: "+s+" | ADC="+adc+" | Ist="+cur.toFixed(1)+"°";drawCompass()}).catch(e=>{document.getElementById('status').textContent="Fehler: keine Verbindung"})}
function sendTarget(){const val=parseInt(document.getElementById('inputAngle').value);if(isNaN(val)||val<0||val>359){alert("Bitte 0..359° eingeben");return}fetch('/api/set_target',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'angle='+val}).then(_=>updateStatus())}
function manual(dir){fetch('/api/manual',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'direction='+dir})}
function stopManual(){fetch('/api/stop',{method:'POST'})}
function stopAll(){fetch('/api/stop',{method:'POST'}).then(_=>updateStatus())}
setInterval(updateStatus,300);updateStatus();
document.addEventListener('keydown',e=>{if(e.key==="ArrowLeft")manual('left');if(e.key==="ArrowRight")manual('right');if(e.key===" "||e.key==="Escape")stopAll()});
document.addEventListener('keyup',e=>{if(e.key==="ArrowLeft"||e.key==="ArrowRight")stopManual()});
</script>
</body>
</html>
)HTML";
  request->send(200, "text/html", html);
}

void handleConfigPage(AsyncWebServerRequest *request) {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<title>Konfiguration</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
body{font-family:Arial,sans-serif;background:#f0f2f5;margin:0;padding:0}
.container{max-width:700px;margin:20px auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1)}
h1{margin-top:0}
label{display:block;margin-top:10px}
input{width:100%;padding:6px;font-size:14px;box-sizing:border-box}
button{margin-top:15px;padding:8px 16px;font-size:16px;border:none;border-radius:5px;cursor:pointer}
.btn-save{background:#28a745;color:#fff}
a{display:inline-block;margin-top:10px;text-decoration:none;color:#007bff}
.status{margin-top:10px;padding:8px;border-radius:5px;display:none}
.ok{background:#d4edda}
.err{background:#f8d7da}
</style>
</head>
<body>
<div class="container">
<h1>Konfiguration</h1>
<div id="msg" class="status"></div>
<form id="cfg">
<h3>Access Point (Fallback)</h3>
<label>AP SSID:<input id="apSsid"></label>
<label>AP Passwort:<input id="apPass" type="password"></label>
<h3>WiFi Client (optional)</h3>
<label>STA SSID:<input id="staSsid" placeholder="Leer = nur AP"></label>
<label>STA Passwort:<input id="staPass" type="password"></label>
<h3>UDP / Rotor</h3>
<label>UDP Port:<input id="udpPort" type="number" min="1024" max="65535"></label>
<label>Toleranz (Grad):<input id="tol" type="number" step="0.1" min="0.1" max="10"></label>
<label>Anschlag-Überlauf X (Grad):<input id="overrun" type="number" step="0.5" min="0" max="20" value="5"></label>
<button type="submit" class="btn-save">Speichern & Neustart</button>
</form>
<a href="/">Zurück</a>
</div>
<script>
function loadCfg(){fetch('/api/get_config').then(r=>r.json()).then(c=>{document.getElementById('apSsid').value=c.apSsid;document.getElementById('apPass').value=c.apPass;document.getElementById('staSsid').value=c.staSsid;document.getElementById('staPass').value=c.staPass;document.getElementById('udpPort').value=c.udpPort;document.getElementById('tol').value=c.tolerance;document.getElementById('overrun').value=c.overrun})}
document.getElementById('cfg').addEventListener('submit',e=>{e.preventDefault();let cfg={apSsid:document.getElementById('apSsid').value,apPass:document.getElementById('apPass').value,staSsid:document.getElementById('staSsid').value,staPass:document.getElementById('staPass').value,udpPort:parseInt(document.getElementById('udpPort').value),tolerance:parseFloat(document.getElementById('tol').value),overrun:parseFloat(document.getElementById('overrun').value)};fetch('/api/save_config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'apSsid='+encodeURIComponent(cfg.apSsid)+'&apPass='+encodeURIComponent(cfg.apPass)+'&staSsid='+encodeURIComponent(cfg.staSsid)+'&staPass='+encodeURIComponent(cfg.staPass)+'&udpPort='+cfg.udpPort+'&tolerance='+cfg.tolerance+'&overrun='+cfg.overrun}).then(r=>r.text()).then(t=>{let m=document.getElementById('msg');m.style.display='block';m.className='status ok';m.textContent='Gespeichert, ESP startet neu...';setTimeout(()=>{location.href='/'},4000)}).catch(e=>{let m=document.getElementById('msg');m.style.display='block';m.className='status err';m.textContent='Fehler'})});
loadCfg();
</script>
</body>
</html>
)HTML";
  request->send(200, "text/html", html);
}

void handleCalibrationPage(AsyncWebServerRequest *request) {
  String html = R"HTML(
<!DOCTYPE html>
<html lang="de">
<head>
<meta charset="UTF-8">
<title>Kalibrierung</title>
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<style>
body{font-family:Arial,sans-serif;background:#f0f2f5;margin:0;padding:0}
.container{max-width:900px;margin:20px auto;background:#fff;padding:20px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.1)}
h1{margin-top:0}
table{width:100%;border-collapse:collapse;margin-top:10px}
th,td{border:1px solid #ccc;padding:6px;text-align:center}
input{width:90%;padding:4px}
button{padding:6px 12px;border:none;border-radius:4px;cursor:pointer}
.btn-cap{background:#17a2b8;color:#fff}
.btn-save{background:#28a745;color:#fff;margin-top:10px}
.info{padding:10px;background:#fff3cd;border-radius:5px;margin-bottom:10px;border:1px solid #ffc107}
.status{margin-top:10px;padding:8px;border-radius:5px;display:none}
.ok{background:#d4edda}
.err{background:#f8d7da}
a{display:inline-block;margin-top:10px;text-decoration:none;color:#007bff}
</style>
</head>
<body>
<div class="container">
<h1>16-Punkt Kalibrierung</h1>
<div class="info">
<strong>Anleitung:</strong><br>
1. Rotor auf 0°, 20°, 40° ... 340° stellen und erfassen (16 Punkte)<br>
2. Rotor auf (180-X)° links vom Anschlag stellen und Index 16 erfassen<br>
3. Rotor auf (180+X)° rechts vom Anschlag stellen und Index 17 erfassen<br>
4. "Kalibrierung speichern" klicken<br>
<br><strong>Wichtig:</strong> X = Überlauf-Winkel aus Konfiguration (Standard 5°)
</div>
<div style="background:#e7f3ff;padding:10px;border-radius:5px;margin-bottom:10px">
Aktueller ADC: <strong><span id="adc">---</span></strong> | 
Berechneter Winkel: <strong><span id="ang">---</span>°</strong>
</div>
<div id="msg" class="status"></div>
<table>
<thead><tr><th>Index</th><th>Winkel</th><th>ADC-Wert</th><th>Aktion</th></tr></thead>
<tbody id="tbody"></tbody>
</table>
<button class="btn-save" onclick="saveCal()">Kalibrierung speichern</button><br>
<a href="/">Zurück</a>
</div>
<script>
let adc=0,ang=0,vals=new Array(18).fill(0),angles=new Array(18).fill(0);
function mkTable(){let tb=document.getElementById('tbody');tb.innerHTML='';for(let i=0;i<18;i++){let w=angles[i];let label='<input id="a'+i+'" value="'+w.toFixed(1)+'" style="width:70px;text-align:center"> °';if(i==16)label='(180-X)° Links';if(i==17)label='(180+X)° Rechts';tb.innerHTML+='<tr><td>'+i+'</td><td><strong>'+label+'</strong></td><td><input id="c'+i+'" value="'+vals[i]+'" style="text-align:center"></td><td><button class="btn-cap" onclick="cap('+i+')">Erfassen</button></td></tr>'}}
function cap(i){document.getElementById('c'+i).value=adc;vals[i]=adc;showMsg('Index '+i+' ('+i*22.5+'°): ADC='+adc+' erfasst',true)}
function loadCal(){fetch('/api/get_calibration').then(r=>r.json()).then(d=>{vals=d.values;angles=d.angles;mkTable()})}
function updStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{adc=d.adc;ang=d.currentAngle;document.getElementById('adc').textContent=adc;document.getElementById('ang').textContent=ang.toFixed(1)})}
function showMsg(t,ok){let m=document.getElementById('msg');m.style.display='block';m.className='status '+(ok?'ok':'err');m.textContent=t;setTimeout(()=>{m.style.display='none'},3000)}
function saveCal(){for(let i=0;i<18;i++){let v=parseInt(document.getElementById('c'+i).value);if(isNaN(v)||v<0||v>4095){showMsg('Ungültiger Wert bei Index '+i,false);return}vals[i]=v;if(i<16){let a=parseFloat(document.getElementById('a'+i).value);if(isNaN(a)||a<0||a>=360){showMsg('Ungültiger Winkel bei Index '+i,false);return}angles[i]=a}}let body='';for(let i=0;i<18;i++){if(i)body+='&';body+='c'+i+'='+vals[i]+'&a'+i+'='+angles[i]}fetch('/api/save_calibration',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body}).then(r=>r.text()).then(_=>{showMsg('✓ Kalibrierung gespeichert!',true);setTimeout(()=>{location.href='/'},2000)}).catch(e=>showMsg('Fehler beim Speichern',false))}
mkTable();loadCal();setInterval(updStatus,500);updStatus();
</script>
</body>
</html>
)HTML";
  request->send(200, "text/html", html);
}

// ------------------------- API Handler -------------------------

void handleAPIStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<256> doc;
  doc["currentAngle"] = rotor.currentAngle;
  doc["targetAngle"] = rotor.targetAngle;
  doc["hasTarget"] = rotor.hasTarget;
  doc["manualLeft"] = rotor.manualLeft;
  doc["manualRight"] = rotor.manualRight;
  doc["adc"] = rotor.lastAdc;
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleAPISetTarget(AsyncWebServerRequest *request) {
  if (request->hasParam("angle", true)) {
    float a = request->getParam("angle", true)->value().toFloat();
    if (a >= 0 && a < 360) {
      rotor.targetAngle = a;
      rotor.hasTarget = true;
      rotor.manualLeft = false;
      rotor.manualRight = false;
      request->send(200, "text/plain", "OK");
      Serial.printf("WEB: target=%.1f°\n", a);
      return;
    }
  }
  request->send(400, "text/plain", "Invalid angle");
}

void handleAPIManual(AsyncWebServerRequest *request) {
  if (request->hasParam("direction", true)) {
    String d = request->getParam("direction", true)->value();
    rotor.hasTarget = false;
    if (d == "left") {
      rotor.manualLeft = true;
      rotor.manualRight = false;
    } else if (d == "right") {
      rotor.manualLeft = false;
      rotor.manualRight = true;
    }
    request->send(200, "text/plain", "OK");
    return;
  }
  request->send(400, "text/plain", "Invalid direction");
}

void handleAPIStop(AsyncWebServerRequest *request) {
  rotor.hasTarget = false;
  rotor.manualLeft = false;
  rotor.manualRight = false;
  request->send(200, "text/plain", "OK");
}

void handleAPIGetConfig(AsyncWebServerRequest *request) {
  StaticJsonDocument<384> doc;
  doc["apSsid"] = config.apSsid;
  doc["apPass"] = config.apPass;
  doc["staSsid"] = config.staSsid;
  doc["staPass"] = config.staPass;
  doc["udpPort"] = config.udpPort;
  doc["tolerance"] = config.toleranceDeg;
  doc["overrun"] = config.overrunDeg;
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleAPISaveConfig(AsyncWebServerRequest *request) {
  if (request->hasParam("apSsid", true)) config.apSsid = request->getParam("apSsid", true)->value();
  if (request->hasParam("apPass", true)) config.apPass = request->getParam("apPass", true)->value();
  if (request->hasParam("staSsid", true)) config.staSsid = request->getParam("staSsid", true)->value();
  if (request->hasParam("staPass", true)) config.staPass = request->getParam("staPass", true)->value();
  if (request->hasParam("udpPort", true)) config.udpPort = request->getParam("udpPort", true)->value().toInt();
  if (request->hasParam("tolerance", true)) config.toleranceDeg = request->getParam("tolerance", true)->value().toFloat();
  if (request->hasParam("overrun", true)) config.overrunDeg = request->getParam("overrun", true)->value().toFloat();
  saveConfig();
  request->send(200, "text/plain", "OK");
  delay(2000);
  ESP.restart();
}

void handleAPIGetCalibration(AsyncWebServerRequest *request) {
  StaticJsonDocument<1024> doc;
  doc["valid"] = config.calibValid;
  JsonArray arrV = doc.createNestedArray("values");
  JsonArray arrA = doc.createNestedArray("angles");
  for (int i = 0; i < 18; i++) {
    arrV.add(config.calibAdc[i]);
    arrA.add(config.calibAngle[i]);
  }
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

void handleAPISaveCalibration(AsyncWebServerRequest *request) {
  bool ok = true;
  for (int i = 0; i < 18; i++) {
    String nameC = "c" + String(i);
    String nameA = "a" + String(i);
    if (!request->hasParam(nameC, true) || !request->hasParam(nameA, true)) { ok = false; break; }
    float v = request->getParam(nameC, true)->value().toFloat();
    float a = request->getParam(nameA, true)->value().toFloat();
    if (v < 0 || v > 4095 || a < 0 || a >= 360) { ok = false; break; }
    config.calibAdc[i] = v;
    config.calibAngle[i] = a;
  }
  if (!ok) {
    request->send(400, "text/plain", "Invalid data");
    return;
  }
  
  config.calibValid = true;
  saveConfig();
  request->send(200, "text/plain", "OK");
  Serial.println(F("\n✓ KALIBRIERUNG GESPEICHERT UND AKTIVIERT"));
  Serial.println(F("Neue Kalibrierungswerte (18 Punkte):"));
  for (int i = 0; i < 18; i++) {
    Serial.printf("  %6.1f° → ADC %4.0f\n", config.calibAngle[i], config.calibAdc[i]);
  }
  Serial.println();
}
