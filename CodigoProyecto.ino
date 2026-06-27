#include <avr/pgmspace.h>
#include <DHTStable.h>
#include <SoftwareSerial.h>

// --- Versión del programa ---
#define VERSION "v17.5.0-mega"  // Versión con MQ136 integrado

// --- Configuración WiFi y Ubidots (en PROGMEM) ---
const char WIFI_SSID[]      PROGMEM = "Nexxt_131C68";
const char WIFI_PASS[]      PROGMEM = "SalondeProfesoresI";

const char UBIDOTS_TOKEN[]  PROGMEM = "BBUS-Zj6jNCAwiJV1clMBPeTopX5kWdxx4H";
const char DEVICE_LABEL[]   PROGMEM = "estacion_meteo";
const char UBIDOTS_SERVER[] PROGMEM = "industrial.api.ubidots.com";

// --- Pines analógicos / digitales ---
#define DHTPIN     4
#define MQ7_PIN    A0
#define MQ135_PIN  A1
#define MQ131_PIN  A2
#define MQ136_PIN  A3   // NUEVO: Sensor de H2S (sulfuro de hidrógeno)

// --- UARTs hardware dedicadas ---
#define WIFI_SERIAL   Serial1   // ESP8266 en Serial1 -> RX1(19), TX1(18)
#define PMS_SERIAL    Serial2   // PMS5003 en Serial2 -> RX2(17), TX2(16)

// --- Objetos ---
DHTStable dht;

// --- Tiempos ---
#define SENSOR_INTERVAL       30000  // 30 s
#define WIFI_CHECK_INTERVAL   15000  // 15 s
#define DHT_READ_INTERVAL      3000  // 3 s
unsigned long prevSensorMillis, prevWifiMillis, prevDhtMillis;

// --- Estado ---
bool isWifiConnected = false;
bool dhtReadSuccess = false;
bool lastPMSReadSuccess = false;

// --- Variables de sensores ---
float temperatura, humedad;
float ppm_nh3, ppm_nox, ppm_co2;
float ppm_h2s;  // NUEVO: H2S en ppm (sulfuro de hidrógeno) del MQ136
int adc_mq7, adc_mq135, adc_mq131, adc_mq136;

// --- Variables de PMS5003 ---
struct PMSData {
  uint16_t pm1;
  uint16_t pm25;
  uint16_t pm10;
  bool valid = false;
};
PMSData pmsData;

// --- Buffer compartido ---
char buffer[400];  // Buffer principal compartido

// --- Prototipos ---
bool attemptAutoConnect();
bool checkWifiConnection();
bool sendCommand(const char* cmd, unsigned long timeout, const char* expectedResponse, bool printOutput = false);
void sendDataToUbidots();
bool readDhtWithRetry();
bool readPMSDataToGlobal();
void calculateMQ135Gases(int adc_value);
void calculateMQ136Gas(int adc_value);
void getStringFromProgmem(const char* progmemString, char* ramBuffer);

void setup() {
  Serial.begin(115200);

  // Inicializar los UARTs de hardware
  WIFI_SERIAL.begin(115200);  // ESP8266 tipicamente 115200
  PMS_SERIAL.begin(9600);     // PMS5003 es 9600 8N1 por defecto

  Serial.print(F("\n--- ARDUINO ESTACIÓN METEO ("));
  Serial.print(F(VERSION));
  Serial.println(F(" - Mega + PMS5003 + ESP8266 + MQ136) ---"));

  delay(500);

  // Probar comunicación básica con ESP8266 (Serial1)
  byte attempts = 0;
  bool espResponding = false;

  while (attempts < 3 && !espResponding) {
    Serial.print(F("Comprobando ESP8266 (intento "));
    Serial.print(attempts + 1);
    Serial.println(F(")..."));

    while (WIFI_SERIAL.available()) WIFI_SERIAL.read();

    WIFI_SERIAL.println("AT");
    delay(500);

    byte idx = 0;
    memset(buffer, 0, sizeof(buffer));
    unsigned long startCheck = millis();

    while (millis() - startCheck < 1000) {
      if (WIFI_SERIAL.available()) {
        char c = WIFI_SERIAL.read();
        if (idx < sizeof(buffer) - 1) buffer[idx++] = c;
        buffer[idx] = '\0';
      }
    }

    if (strstr(buffer, "OK") != NULL) {
      espResponding = true;
      Serial.println(F("ESP8266 respondiendo OK"));
    } else {
      Serial.print(F("No respuesta. Recibido: "));
      Serial.println(buffer);
      attempts++;
      delay(700);
    }
  }

  if (espResponding) {
    isWifiConnected = attemptAutoConnect();
    Serial.println(isWifiConnected ? F("WiFi conectado") : F("WiFi falló"));
  } else {
    Serial.println(F("ERROR: ESP8266 no responde. Verifique conexiones/niveles lógicos."));
  }

  // Lectura inicial DHT
  dhtReadSuccess = readDhtWithRetry();

  // Inicialización
  pmsData.valid = false;
  prevSensorMillis = prevWifiMillis = prevDhtMillis = millis();

  Serial.println(F("Nota: MQ136 requiere precalentamiento de al menos 5 minutos"));
  Serial.println(F("para obtener lecturas estables de H2S"));
}

void loop() {
  unsigned long currentMillis = millis();

  // --- Pasarela de depuración Serial <-> ESP8266 ---
  while (WIFI_SERIAL.available()) Serial.write(WIFI_SERIAL.read());
  if (Serial.available()) {
    byte bytesRead = Serial.readBytesUntil('\n', buffer, 127);
    if (bytesRead > 0) {
      if (buffer[bytesRead-1] == '\r') bytesRead--;
      buffer[bytesRead] = '\0';
      WIFI_SERIAL.println(buffer);
    }
  }

  // --- Lectura periódica del DHT ---
  if (currentMillis - prevDhtMillis >= DHT_READ_INTERVAL) {
    prevDhtMillis = currentMillis;
    dhtReadSuccess = readDhtWithRetry();
  }

  // --- Chequeo periódico de WiFi ---
  if (currentMillis - prevWifiMillis >= WIFI_CHECK_INTERVAL) {
    prevWifiMillis = currentMillis;

    bool prevWifiState = isWifiConnected;
    isWifiConnected = checkWifiConnection();

    if (prevWifiState != isWifiConnected) {
      Serial.print(F("Estado WiFi: "));
      Serial.println(isWifiConnected ? F("CONECTADO") : F("DESCONECTADO"));

      if (!isWifiConnected) {
        Serial.println(F("Intentando reconexión..."));
        isWifiConnected = attemptAutoConnect();
      }
    }
  }

  // --- Ciclo principal de lectura + envío ---
  if (currentMillis - prevSensorMillis >= SENSOR_INTERVAL) {
    prevSensorMillis = currentMillis;

    Serial.println(F("\n--- Ciclo de Lectura y Envío ---"));

    // Analógicos
    adc_mq7   = analogRead(MQ7_PIN);
    adc_mq135 = analogRead(MQ135_PIN);
    adc_mq131 = analogRead(MQ131_PIN);
    adc_mq136 = analogRead(MQ136_PIN);   // NUEVO: Lectura MQ136
    calculateMQ135Gases(adc_mq135);
    calculateMQ136Gas(adc_mq136);        // NUEVO: Cálculo H2S

    // Reintento DHT si falló
    if (!dhtReadSuccess) {
      Serial.println(F("Reintentando lectura DHT..."));
      dhtReadSuccess = readDhtWithRetry();
    }

    // Log sensores
    Serial.print(F("T:")); Serial.print(temperatura, 1);
    Serial.print(F("°C H:")); Serial.print(humedad, 1);
    Serial.print(F("%"));
    Serial.print(F(" MQ7:")); Serial.print(adc_mq7);
    Serial.print(F(" MQ135:")); Serial.print(adc_mq135);
    Serial.print(F(" MQ131:")); Serial.print(adc_mq131);
    Serial.print(F(" MQ136:")); Serial.println(adc_mq136);

    Serial.print(F("NH3:")); Serial.print(ppm_nh3, 1);
    Serial.print(F(" NOx:")); Serial.print(ppm_nox, 1);
    Serial.print(F(" CO2:")); Serial.print(ppm_co2, 1);
    Serial.print(F(" H2S:")); Serial.print(ppm_h2s, 1);
    Serial.println(F(" ppm"));

    // PMS5003
    Serial.println(F("Leyendo datos del sensor PMS5003..."));
    lastPMSReadSuccess = readPMSDataToGlobal();

    if (lastPMSReadSuccess) {
      Serial.print(F("PM1.0:")); Serial.print(pmsData.pm1);
      Serial.print(F(" PM2.5:")); Serial.print(pmsData.pm25);
      Serial.print(F(" PM10:")); Serial.print(pmsData.pm10);
      Serial.println(F(" μg/m³"));
    } else {
      Serial.println(F("ERROR: No se pudo leer datos de PMS5003"));
    }

    // Envío a Ubidots
    if (isWifiConnected) {
      Serial.println(F("Enviando datos a Ubidots..."));
      sendDataToUbidots();
    } else {
      Serial.println(F("WiFi desconectado - no se pueden enviar datos"));
      Serial.println(F("Intentando reconexión WiFi..."));
      isWifiConnected = attemptAutoConnect();
    }
  }
}

bool readPMSDataToGlobal() {
  // NOTA: En Mega NO detenemos el WiFi; cada UART es independiente.
  const unsigned long timeout = 3000; // 3 s
  unsigned long startTime = millis();

  // Vaciar buffer serial del PMS
  while (PMS_SERIAL.available()) PMS_SERIAL.read();

  Serial.println(F("Esperando datos del PMS5003..."));

  uint8_t pmsBuf[32];
  int index = 0;
  bool success = false;

  while (millis() - startTime < timeout && !success) {
    while (PMS_SERIAL.available()) {
      uint8_t c = PMS_SERIAL.read();

      // Cabecera 0x42 0x4D
      if (index == 0 && c != 0x42) continue;
      if (index == 1 && c != 0x4D) { index = 0; continue; }

      if (index < 32) pmsBuf[index++] = c;

      if (index == 32) {
        uint16_t checksum = 0;
        for (uint8_t i = 0; i < 30; i++) checksum += pmsBuf[i];

        uint16_t receivedChecksum = (pmsBuf[30] << 8) | pmsBuf[31];

        if (checksum == receivedChecksum) {
          pmsData.pm1 = (pmsBuf[4] << 8) | pmsBuf[5];
          pmsData.pm25 = (pmsBuf[6] << 8) | pmsBuf[7];
          pmsData.pm10 = (pmsBuf[8] << 8) | pmsBuf[9];
          pmsData.valid = true;
          success = true;
          Serial.println(F("Datos PMS5003 recibidos correctamente"));
        } else {
          Serial.println(F("Error de checksum PMS5003!"));
        }
        break;
      }
    }
    delay(5);
  }

  if (!success) {
    Serial.println(F("Timeout: PMS5003 no envió datos completos"));
  }
  return success;
}

bool readDhtWithRetry() {
  for (byte i = 0; i < 3; i++) {
    if (dht.read11(DHTPIN) == 0) {
      humedad = dht.getHumidity();
      temperatura = dht.getTemperature();
      if (temperatura > -40 && temperatura < 80 && humedad >= 0 && humedad <= 100) {
        return true;
      }
    }
    delay(400);
  }
  return false;
}

void calculateMQ135Gases(int adc_value) {
  float voltaje = adc_value * (5.0 / 1023.0);
  ppm_nh3 = constrain(voltaje * 10.0, 0, 200);
  ppm_nox = constrain(voltaje * 15.0, 0, 300);
  ppm_co2 = constrain(350.0 + (voltaje * 1000.0), 350, 10000);
}

// NUEVO: Cálculo aproximado de H2S para el sensor MQ136
// El factor 20.0 es una estimación inicial; para mediciones precisas
// se debe calibrar el sensor en aire limpio y ajustar según datasheet.
void calculateMQ136Gas(int adc_value) {
  float voltaje = adc_value * (5.0 / 1023.0);
  ppm_h2s = constrain(voltaje * 20.0, 0, 200);
}

void getStringFromProgmem(const char* progmemString, char* ramBuffer) {
  strcpy_P(ramBuffer, progmemString);
}

bool checkWifiConnection() {
  return sendCommand("AT+CWJAP?", 2000, "+CWJAP:", false);
}

bool attemptAutoConnect() {
  Serial.println(F("Reiniciando módulo WiFi..."));
  WIFI_SERIAL.println("AT+RST");
  delay(3000);  // ESP se reinicia completamente

  // Limpiar buffer del ESP
  while (WIFI_SERIAL.available()) WIFI_SERIAL.read();

  // Probar comunicación básica
  if (!sendCommand("AT", 2000, "OK", true)) {
    Serial.println(F("ERROR: ESP no responde después del reset"));
    return false;
  }

  // Configurar modo estación
  if (!sendCommand("AT+CWMODE=1", 2000, "OK", true)) {
    Serial.println(F("ERROR: No se pudo configurar modo estación"));
    return false;
  }

  // Desconectar de AP previos
  sendCommand("AT+CWQAP", 1000, "OK", false);

  // Copiar SSID y PASS desde PROGMEM a RAM
  char ssid[30], pass[30];
  getStringFromProgmem(WIFI_SSID, ssid);
  getStringFromProgmem(WIFI_PASS, pass);

  // Formar comando correctamente
  char cmd[80];
  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, pass);

  Serial.print(F("Conectando a red WiFi: "));
  Serial.println(ssid);

  // Intentar conexión (timeout 25s)
  bool connected = sendCommand(cmd, 25000, "WIFI GOT IP", true);

  Serial.println(connected ? F("WiFi conectado") : F("WiFi fallo"));
  return connected;
}

void sendDataToUbidots() {
  char serverStr[40], tokenStr[40], labelStr[40];
  char tempBuffer[350];  // Aumentado para incluir nuevas variables

  getStringFromProgmem(UBIDOTS_SERVER, serverStr);
  getStringFromProgmem(DEVICE_LABEL, labelStr);
  getStringFromProgmem(UBIDOTS_TOKEN, tokenStr);

  Serial.println(F("Verificando estado de conexión WiFi..."));
  if (!checkWifiConnection()) {
    Serial.println(F("Error: WiFi no conectado, abortando envío"));
    isWifiConnected = false;
    return;
  }

  char tempStr[8], humStr[8], coStr[8], o3Str[8];
  char nh3Str[8], noxStr[8], co2Str[8];
  char pm1Str[8], pm25Str[8], pm10Str[8];
  char h2sStr[8];  // NUEVO: string para H2S

  dtostrf(temperatura, 4, 1, tempStr);
  dtostrf(humedad, 4, 1, humStr);

  float ppm_co = adc_mq7   * (5.0 / 1023.0) * 20.0;
  float ppm_o3 = adc_mq131 * (5.0 / 1023.0) * 10.0;

  dtostrf(ppm_co, 4, 1, coStr);
  dtostrf(ppm_o3, 4, 1, o3Str);
  dtostrf(ppm_nh3, 4, 1, nh3Str);
  dtostrf(ppm_nox, 4, 1, noxStr);
  dtostrf(ppm_co2, 5, 1, co2Str);
  dtostrf(ppm_h2s, 4, 1, h2sStr);  // NUEVO: conversión H2S

  if (pmsData.valid) {
    itoa(pmsData.pm1,  pm1Str, 10);
    itoa(pmsData.pm25,  pm25Str, 10);
    itoa(pmsData.pm10, pm10Str, 10);
  } else {
    strcpy(pm1Str, "0"); strcpy(pm25Str, "0"); strcpy(pm10Str, "0");
  }

  Serial.println(F("Verificando respuesta del ESP8266..."));
  if (!sendCommand("AT", 1000, "OK", true)) {
    Serial.println(F("Error: ESP8266 no responde, abortando envío"));
    return;
  }

  Serial.println(F("Cerrando conexiones previas..."));
  sendCommand("AT+CIPCLOSE", 1000, NULL, false);

  Serial.print(F("Conectando a ")); Serial.print(serverStr); Serial.println(F("..."));
  while (WIFI_SERIAL.available()) WIFI_SERIAL.read();

  memset(buffer, 0, sizeof(buffer));
  snprintf(buffer, sizeof(buffer), "AT+CIPSTART=\"TCP\",\"%s\",80", serverStr);
  Serial.print(F("Enviando: ")); Serial.println(buffer);

  bool connected = false;
  for (byte attempt = 0; attempt < 3 && !connected; attempt++) {
    Serial.print(F("Intento ")); Serial.print(attempt + 1); Serial.print(F(" de 3... "));
    WIFI_SERIAL.println(buffer);

    unsigned long startTime = millis();
    memset(buffer, 0, sizeof(buffer));
    byte idx = 0;
    bool errorFound = false;
    bool connectFound = false;

    while (millis() - startTime < 10000 && !connectFound && !errorFound) {
      if (WIFI_SERIAL.available()) {
        char c = WIFI_SERIAL.read();
        if (idx < sizeof(buffer) - 1) { buffer[idx++] = c; buffer[idx] = '\0'; }
        if (strstr(buffer, "CONNECT") != NULL) connectFound = true;
        if (strstr(buffer, "ERROR")   != NULL) errorFound   = true;
      }
    }

    if (connectFound) {
      connected = true;
      Serial.println(F("\nConexión establecida!"));
    } else {
      Serial.println(F("\nFalló la conexión, reintento..."));
      delay(1200);
    }
  }

  if (!connected) {
    Serial.println(F("Error: No se pudo conectar al servidor"));
    return;
  }

  // Payload JSON - MODIFICADO: ahora incluye mq136_adc y h2s
  memset(tempBuffer, 0, sizeof(tempBuffer));
  snprintf(
    tempBuffer, sizeof(tempBuffer),
           "{\"temperatura\":%s,\"humedad\":%s,\"co\":%s,\"o3\":%s,\"mq135_adc\":%d,\"nh3\":%s,\"nox\":%s,\"co2\":%s,\"pm1\":%s,\"pm25\":%s,\"pm10\":%s,\"mq136_adc\":%d,\"h2s\":%s}",
           tempStr, humStr, coStr, o3Str, adc_mq135, nh3Str, noxStr, co2Str, pm1Str, pm25Str, pm10Str, adc_mq136, h2sStr
  );

  int payloadLen = strlen(tempBuffer);
  Serial.print(F("Payload JSON (")); Serial.print(payloadLen); Serial.println(F(" bytes)"));

  // Header HTTP
  memset(buffer, 0, sizeof(buffer));
  strcpy(buffer, "POST /api/v1.6/devices/");
  strcat(buffer, labelStr);
  strcat(buffer, " HTTP/1.1\r\nHost: ");
  strcat(buffer, serverStr);
  strcat(buffer, "\r\nContent-Type: application/json\r\nX-Auth-Token: ");
  strcat(buffer, tokenStr);
  strcat(buffer, "\r\nContent-Length: ");

  char lenStr[8];
  itoa(payloadLen, lenStr, 10);
  strcat(buffer, lenStr);
  strcat(buffer, "\r\n\r\n");

  int headerLen = strlen(buffer);
  int totalLen  = headerLen + payloadLen;

  Serial.print(F("Longitud total de solicitud: "));
  Serial.print(totalLen);
  Serial.println(F(" bytes"));

  Serial.println(F("Enviando comando CIPSEND..."));
  while (WIFI_SERIAL.available()) WIFI_SERIAL.read();

  char cmdLen[24];
  snprintf(cmdLen, sizeof(cmdLen), "AT+CIPSEND=%d", totalLen);
  Serial.print(F("Comando: ")); Serial.println(cmdLen);
  WIFI_SERIAL.println(cmdLen);

  unsigned long startTime = millis();
  bool gotPrompt = false;
  memset(buffer, 0, sizeof(buffer));
  byte idx = 0;

  while (millis() - startTime < 5000 && !gotPrompt) {
    if (WIFI_SERIAL.available()) {
      char c = WIFI_SERIAL.read();
      if (idx < sizeof(buffer) - 1) { buffer[idx++] = c; buffer[idx] = '\0'; }
      if (c == '>') gotPrompt = true;
    }
  }

  if (!gotPrompt) {
    Serial.println(F("\nError: No se recibió prompt '>'"));
    sendCommand("AT+CIPCLOSE", 1000, NULL, false);
    return;
  }

  Serial.println(F("\nEnviando datos HTTP..."));
  WIFI_SERIAL.print(
    String("POST /api/v1.6/devices/") + labelStr +
    " HTTP/1.1\r\nHost: " + serverStr +
    "\r\nContent-Type: application/json\r\nX-Auth-Token: " + tokenStr +
    "\r\nContent-Length: " + lenStr + "\r\n\r\n"
  );
  WIFI_SERIAL.print(tempBuffer);

  Serial.println(F("Esperando respuesta..."));
  startTime = millis();
  bool success = false;
  bool closed  = false;
  memset(buffer, 0, sizeof(buffer));
  idx = 0;

  while (millis() - startTime < 15000 && !closed) {
    if (WIFI_SERIAL.available()) {
      char c = WIFI_SERIAL.read();
      if (idx < sizeof(buffer) - 1) { buffer[idx++] = c; buffer[idx] = '\0'; }
      if (strstr(buffer, "HTTP/1.1 20") != NULL) success = true;
      if (strstr(buffer, "CLOSED")      != NULL) closed  = true;
    }
    if (idx > (int)(sizeof(buffer) * 0.8)) {
      int halfSize = sizeof(buffer) / 2;
      memmove(buffer, buffer + halfSize, idx - halfSize);
      idx -= halfSize;
      buffer[idx] = '\0';
    }
  }

  if (success) {
    Serial.println(F("\n--- Datos enviados correctamente ---"));
  } else {
    Serial.println(F("\n--- Error al enviar datos ---"));
    if (idx > 0) {
      Serial.println(F("Respuesta recibida:"));
      Serial.println(buffer);
    } else {
      Serial.println(F("No se recibió respuesta (timeout)"));
    }
  }

  if (!closed) {
    Serial.println(F("Cerrando conexión..."));
    sendCommand("AT+CIPCLOSE", 2000, NULL, false);
  }
}

bool sendCommand(const char* cmd, unsigned long timeout, const char* expectedResponse, bool printOutput) {
  while (WIFI_SERIAL.available()) WIFI_SERIAL.read();
  memset(buffer, 0, sizeof(buffer));

  if (printOutput) {
    Serial.print(F("CMD: "));
    Serial.println(cmd);
  }

  WIFI_SERIAL.println(cmd);
  unsigned long startTime = millis();
  byte responseIdx = 0;

  while (millis() - startTime < timeout) {
    while (WIFI_SERIAL.available()) {
      char c = WIFI_SERIAL.read();
      if (responseIdx < 79) buffer[responseIdx++] = c;
      buffer[responseIdx] = '\0';
      if (printOutput) Serial.write(c);
    }

    if (expectedResponse && strstr(buffer, expectedResponse) != NULL) {
      if (printOutput) {
        Serial.print(F("Respuesta OK: "));
        Serial.println(expectedResponse);
      }
      return true;
    }
    if (strstr(buffer, "ERROR") != NULL) {
      if (printOutput) Serial.println(F("ERROR detectado en respuesta"));
      return false;
    }
  }

  if (printOutput && expectedResponse) {
    Serial.print(F("Timeout esperando: "));
    Serial.println(expectedResponse);
    Serial.print(F("Recibido: "));
    Serial.println(buffer);
  }
  return expectedResponse == NULL ? true : false;
}
