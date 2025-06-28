#include <ESP8266WiFi.h>
#include <TinyGPS++.h>
#include <espnow.h>
#include <SoftwareSerial.h>
#include <math.h>

#define NUM_VERTICES 4  // Number of polygon vertices

// Initial location of RHS Oval (latitude, longitude)
const double OVAL_LAT = 14.567638907546986;
const double OVAL_LNG = 121.07517388358185;
double distanceMeters = 0;

// Define the polygon (quadrilateral) with latitude and longitude
float polyLat[NUM_VERTICES] = {14.567187, 14.567713, 14.568357, 14.567679};
float polyLon[NUM_VERTICES] = {121.075122, 121.074832, 121.075238, 121.075494};

// Haversine method to get distance between two coordinates
double haversine(double OVAL_LAT, double OVAL_LNG, double latitude, double longitude) {
    const double rEarth = 6371000.0; // in meters
    double lat1 = OVAL_LAT * M_PI / 180.0;  // convert to radians
    double lon1 = OVAL_LNG * M_PI / 180.0;  // convert to radians
    double lat2 = latitude * M_PI / 180.0;  // convert to radians
    double lon2 = longitude * M_PI / 180.0;  // convert to radians

    // Differences in coordinates
    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Haversine formula
    double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    double distance = rEarth * c;  // in meters

    return distance;  // return the distance in meters
}

// Variable for buzzer
int led = D7;
int buzzer = D6;

// Time tracking variables
unsigned long previousLedMillis = 0;
unsigned long previousBuzzerMillis = 0;

// Delays
const long ledDelay = 500;       // LED blink delay
const long buzzerDelay = 500;   // Buzzer sound delay

// GPS setup
SoftwareSerial ss(5, 4);  // TX -> GPIO5 (D1), RX -> GPIO4 (D2)
TinyGPSPlus gps;

uint8_t receiverMAC[] = {0x4E, 0xEB, 0xD6, 0x1F, 0x69, 0xB6};

// Struct message
typedef struct struct_message {
    float lat;
    float lon;
    float alt;
  } struct_message;
  
  struct_message myData;

void setup() {
    Serial.begin(9600);
    ss.begin(9600);
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != 0) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_add_peer(receiverMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
    esp_now_register_send_cb(onDataSent);
  
    pinMode(led, OUTPUT);
    pinMode(buzzer, OUTPUT);
}

// Ray-Casting Algorithm: Returns true if the point is inside the polygon
bool isInsidePolygon(float lat, float lon) {
    bool inside = false;
    int j = NUM_VERTICES - 1;  // Last vertex index

    for (int i = 0; i < NUM_VERTICES; i++) {
        if ((polyLon[i] > lon) != (polyLon[j] > lon) && 
            (lat < (polyLat[j] - polyLat[i]) * (lon - polyLon[i]) / (polyLon[j] - polyLon[i]) + polyLat[i])) {
            inside = !inside;  // Toggle state
        }
        j = i;  // Move to next edge
    }
    return inside;
}

void loop() {
    while (ss.available() > 0) {
        if (gps.encode(ss.read())) {
            if (gps.location.isValid()) {
                float lat = gps.location.lat();
                float lon = gps.location.lng();
                float alt = gps.altitude.meters();
                
                // Send data via ESP-NOW
                esp_now_send(receiverMAC, (uint8_t *)&myData, sizeof(myData));
                
                // Store in struct
                myData.lat = lat;
                myData.lon = lon;
                myData.alt = alt;
                if (isInsidePolygon(lat, lon)) {
                  Serial.println("Status from Oval: Inside");
                  Serial.print("Latitude: ");
                  Serial.println(lat, 6);
                  Serial.print("Longitude: ");
                  Serial.println(lon, 6);  
                  Serial.print("Altitude: ");
                  Serial.println(alt, 2);
                  digitalWrite(led, LOW);
                  noTone(buzzer);
                } 
                else {
                  Serial.println("Status from Oval: Outside");
                  Serial.print("Distance from Oval: ");
                  // Calculate distance to RHS Oval using Haversine formula
                  distanceMeters = haversine(OVAL_LAT, OVAL_LNG, lat, lon);
                  Serial.println(distanceMeters);
                  Serial.print("Latitude: ");
                  Serial.println(lat, 6);
                  Serial.print("Longitude: ");
                  Serial.println(lon, 6);  
                  Serial.print("Altitude: ");
                  Serial.println(alt, 2);
                  triggerLED();
                  triggerBuzzer();
                }
            }
        }
    }
}

void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
    delay(500);
    Serial.print("Send Status: ");
    Serial.println(sendStatus == 0 ? "Success" : "Failed");
}

  void triggerLED() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousLedMillis > ledDelay) {
      // Save the last time LED was updated
      previousLedMillis = currentMillis;
      // Toggle LED state
      int ledState = digitalRead(led);
      digitalWrite(led, !ledState);
    }
  }
  
  void triggerBuzzer() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousBuzzerMillis > buzzerDelay) {
      // Save the last time buzzer was updated
      previousBuzzerMillis = currentMillis;
      // Play the buzzer sound (only once)
      tone(buzzer, 1760, 250);  // Buzzer frequency and duration
    }
  }