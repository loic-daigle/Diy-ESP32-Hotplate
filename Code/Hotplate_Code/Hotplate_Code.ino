// The sketch uses 1183032 bytes (90%) of program storage space. The maximum is 1310720 bytes.

#include <OpenMenuOS.h> // Include the OpenMenuOS library
#include "images.h"
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <FastLED.h>
#include <numeric>
#include <vector>
#include <cmath>
#include <Wire.h>
#include <Adafruit_MAX31855.h>

// Create an instance of the OpenMenuOS class with button and display pins
OpenMenuOS menu(19, -1, 2); // btn_up, btn_down, btn_sel
// OpenMenuOS menu(19, -1, 2); // btn_up, btn_down, btn_sel

MenuScreen mainMenu("Main Menu");
CustomScreen reflowScreen;
CustomScreen heatScreen;
CustomScreen profilesScreen;
MenuScreen profilesMenu("Profile Menu");
CustomScreen calibrationScreen;

WiFiClient client;
HTTPClient http;
Preferences prefs;

// Define the pins for the MAX31855
int thermoCLK = 14; // Clock pin
int thermoCS = 13;  // Chip Select pin
int thermoDO = 15;  // Data Out pin

// Create the MAX31855 object
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);

// Rectangle dimensions
int profile_selection_Y = 0;
int profile_selection_X = 0;
const uint16_t rect_width = 155; // Width of the rectangle excluding the 4-pixel space on the right side
const uint16_t rect_height = 26;

// const int tft_width = 160;
// const int tft_height = 80;
const int tft_width = tft.width();
const int tft_height = tft.height();

// LED and Display Control
const int led_pin = 33;
const int buzzer_pin = 27;

int selected_mode = 0;

const float currentVersion = 1.00;
float availableVersion;

int up_button_clicked = 0;
int button_select_clicked_reflow = 0;

// Variables for button state tracking in the heat screen
unsigned long heatButtonPressStartTime = 0;
bool heatButtonWasPressed = false;
bool heatLongPressActive = false;

// WiFi credentials
String ssid1, password1, ssid2, password2;

bool updateAvailable = false;
const int heatingElementPin = 5; // Hotplate heating element pin
const int thermistorPin = 34;    // Thermistor pin
bool heating = false;
bool heatingStarted = false;
bool reflowing = false;
int targetTemp = 0;
int currentTemp = 0;
int selectedReflowProfile = 0;
int last_profile;

// Constants for reflow profiles are defined with the profile struct section below

unsigned long previousMillisTemperature = 0;
#define temperatureInterval 1000

const int MAXPOSSIBLETEMP = 300; // Max temp is 300°C

const int MINIMUMINPUTVOLTAGE = 12; // Minimum input voltage is 12v

// Timing Constants
unsigned long previousMillis = 0;     // Variable to store the time in milliseconds of the last execution
const unsigned long interval = 60000; // 60 seconds (in milliseconds)

const int numSteps = 20;

const int maxPwmValue = 255;
// LED Configuration
#define NUM_LEDS 6
CRGB leds[NUM_LEDS];

// -------------------- UYue 946-1010 control tuning (PID + SSR time-proportional) --------------------
// PID gains tuned conservatively for a large thermal mass preheater + zero-cross SSR
double Kp = 5.0;  // Proportional gain
double Ki = 0.03; // Integral gain (per second)
double Kd = 20.0; // Derivative gain

// Time-proportional control window for SSR (in milliseconds)
const unsigned long pidWindowSizeMs = 1000; // 1s window is gentle on AC SSRs

// PID state
double pidIntegral = 0.0;
double pidLastError = 0.0;
unsigned long pidLastTime = 0;
unsigned long pidWindowStart = 0;

// Forward declarations
void heaterWrite(bool on);
void resetPid();
void updateHeaterControl(float targetCelsius);
void resetReflowState();

// Reflow state
int reflowStep = 0;
unsigned long reflowStepStart = 0;

struct ReflowProfile
{
  const char *name;
  int steps[10][2]; // [time, temp] for all profile points (Preheat, Soak, Reflow, Cooldown, etc.)
};

const int reflowProfileCount = 3;
const int profileSteps = 10;
const int ReflowprofileSize = profileSteps;

ReflowProfile profiles[reflowProfileCount] = {
    {"Sn42Bi57Ag1",
     {{0, 25}, {30, 50}, {60, 80}, {90, 110}, {120, 125}, {150, 145}, {180, 180}, {210, 165}, {240, 95}, {270, 25}}},
    {"Test 1",
     {{0, 25}, {30, 60}, {60, 100}, {90, 120}, {120, 130}, {150, 150}, {180, 180}, {210, 170}, {240, 100}, {270, 25}}},
    {"Test 2",
     {{0, 25}, {30, 40}, {60, 70}, {90, 100}, {120, 110}, {150, 120}, {180, 130}, {210, 140}, {240, 150}, {270, 160}}}};
int selectedItem_profile;

// -------------------- Profile analysis and phase detection --------------------
enum ProfilePhase
{
  PreheatPhase,
  SoakPhase,
  ReflowPhase,
  CooldownPhase,
  DonePhase
};

struct ProfileAnalysis
{
  int soakStartTime = 0; // seconds
  int soakEndTime = 0;   // seconds
  int peakTime = 0;      // seconds at max temperature
  int endTime = 0;       // seconds at last step
  bool hasSoak = false;
  bool computed = false;
};

ProfileAnalysis profileAnalysis[reflowProfileCount];

// Heuristics for detecting soak plateau in steps
const float soakSlopeThreshold = 0.05f; // degC per second considered "flat" (~3 C/min)
const int minSoakDuration = 30;         // minimum plateau duration in seconds to count as soak

static inline int stepTime(int profileIdx, int i) { return profiles[profileIdx].steps[i][0]; }
static inline int stepTemp(int profileIdx, int i) { return profiles[profileIdx].steps[i][1]; }

void analyzeProfile(int profileIdx)
{
  ProfileAnalysis &pa = profileAnalysis[profileIdx];
  if (pa.computed)
    return;

  // Determine peak (max temp) and end time
  int maxIdx = 0;
  int maxT = stepTemp(profileIdx, 0);
  for (int i = 1; i < profileSteps; i++)
  {
    int t = stepTemp(profileIdx, i);
    if (t > maxT)
    {
      maxT = t;
      maxIdx = i;
    }
  }
  pa.peakTime = stepTime(profileIdx, maxIdx);
  pa.endTime = stepTime(profileIdx, profileSteps - 1);

  // Find a plateau (soak) region before the peak
  int plateauStartTime = -1;
  int plateauAccum = 0;
  for (int i = 0; i < maxIdx; i++)
  {
    int t0 = stepTime(profileIdx, i);
    int t1 = stepTime(profileIdx, i + 1);
    int dt = max(1, t1 - t0);
    float slope = float(stepTemp(profileIdx, i + 1) - stepTemp(profileIdx, i)) / float(dt);
    bool flat = fabs(slope) <= soakSlopeThreshold;
    if (flat)
    {
      if (plateauStartTime < 0)
        plateauStartTime = t0;
      plateauAccum += dt;
    }
    else
    {
      if (plateauStartTime >= 0 && plateauAccum >= minSoakDuration)
      {
        pa.hasSoak = true;
        pa.soakStartTime = plateauStartTime;
        pa.soakEndTime = plateauStartTime + plateauAccum;
        break; // take the first valid soak region
      }
      plateauStartTime = -1;
      plateauAccum = 0;
    }
  }
  // Handle plateau reaching exactly to the segment before peak
  if (!pa.hasSoak && plateauStartTime >= 0 && plateauAccum >= minSoakDuration)
  {
    pa.hasSoak = true;
    pa.soakStartTime = plateauStartTime;
    pa.soakEndTime = plateauStartTime + plateauAccum;
  }

  pa.computed = true;
}

int interpolateTargetTemp(int profileIdx, int elapsedSec)
{
  // Clamp before first and after last
  if (elapsedSec <= stepTime(profileIdx, 0))
    return stepTemp(profileIdx, 0);
  if (elapsedSec >= stepTime(profileIdx, profileSteps - 1))
    return stepTemp(profileIdx, profileSteps - 1);

  for (int i = 0; i < profileSteps - 1; i++)
  {
    int t0 = stepTime(profileIdx, i);
    int t1 = stepTime(profileIdx, i + 1);
    if (elapsedSec >= t0 && elapsedSec <= t1)
    {
      int y0 = stepTemp(profileIdx, i);
      int y1 = stepTemp(profileIdx, i + 1);
      int dt = max(1, t1 - t0);
      float alpha = float(elapsedSec - t0) / float(dt);
      return int(roundf(y0 + alpha * (y1 - y0)));
    }
  }
  return stepTemp(profileIdx, profileSteps - 1); // fallback
}

ProfilePhase getProfilePhase(int profileIdx, int elapsedSec)
{
  analyzeProfile(profileIdx);
  const ProfileAnalysis &pa = profileAnalysis[profileIdx];
  if (elapsedSec >= pa.endTime)
    return DonePhase;
  if (pa.hasSoak && elapsedSec < pa.soakStartTime)
    return PreheatPhase;
  if (pa.hasSoak && elapsedSec < pa.soakEndTime)
    return SoakPhase;
  if (elapsedSec < pa.peakTime)
    return ReflowPhase; // ramp to peak
  return CooldownPhase;
}

int getCurrentStepIndex(int profileIdx, int elapsedSec)
{
  if (elapsedSec < stepTime(profileIdx, 0))
    return 0;
  for (int i = 0; i < profileSteps - 1; i++)
  {
    int t0 = stepTime(profileIdx, i);
    int t1 = stepTime(profileIdx, i + 1);
    if (elapsedSec >= t0 && elapsedSec < t1)
      return i;
  }
  return profileSteps - 2; // last segment
}

// Helper function to parse "key:value" input
bool parseKeyValuePair(const String &input, char separator, String &key, String &value)
{
  int separatorIndex = input.indexOf(separator);
  if (separatorIndex == -1)
    return false;

  key = input.substring(0, separatorIndex);
  value = input.substring(separatorIndex + 1);
  return true;
}

// Function to handle WiFi setup commands
void handleWiFiCommand(const String &command)
{
  String ssid, password;
  if (parseKeyValuePair(command, ':', ssid, password))
  {
    if (ssid.startsWith("wifi1"))
    {
      if (parseKeyValuePair(password, ',', ssid1, password1))
      {
        Serial.println("SSID 1: " + ssid1);
        Serial.println("Password 1: " + password1);
        prefs.putString("ssid1", ssid1);
        prefs.putString("password1", password1);
        connectToWiFi();
      }
      else
      {
        Serial.println("Invalid format. Use 'wifi1:ssid1,password1'");
      }
    }
    else if (ssid.startsWith("wifi2"))
    {
      if (parseKeyValuePair(password, ',', ssid2, password2))
      {
        Serial.println("SSID 2: " + ssid2);
        Serial.println("Password 2: " + password2);
        prefs.putString("ssid2", ssid2);
        prefs.putString("password2", password2);
        connectToWiFi();
      }
      else
      {
        Serial.println("Invalid format. Use 'wifi2:ssid2,password2'");
      }
    }
    else
    {
      Serial.println("Invalid command. Available commands: 'wifi1:ssid1,password1', 'wifi2:ssid2,password2'");
    }
  }
  else
  {
    Serial.println("Invalid format. Use 'wifi1:ssid1,password1' or 'wifi2:ssid2,password2'");
  }
}

// Function to upload a reflow profile
void uploadProfile()
{
  Serial.println("Enter Profile Number (0 to 2):");
  while (!Serial.available())
    ; // Wait for input
  int profileIndex = Serial.parseInt();
  if (profileIndex < 0 || profileIndex >= reflowProfileCount)
  {
    Serial.println("Invalid Profile Number!");
    return;
  }

  Serial.println("Enter 10 steps as time-temperature pairs (e.g., 0,25):");
  for (int i = 0; i < profileSteps; i++)
  {
    Serial.println("Step " + String(i + 1) + ":");
    while (!Serial.available())
      ; // Wait for input
    String input = Serial.readStringUntil('\n');
    input.trim();

    String timeStr, tempStr;
    if (parseKeyValuePair(input, ',', timeStr, tempStr))
    {
      profiles[profileIndex].steps[i][0] = timeStr.toInt();
      profiles[profileIndex].steps[i][1] = tempStr.toInt();
    }
    else
    {
      Serial.println("Invalid input. Use format time,temperature.");
      i--; // Retry the same step
    }
  }

  Serial.println("Profile updated successfully!");
  // Invalidate cached analysis so it recomputes with new steps
  if (profileIndex >= 0 && profileIndex < reflowProfileCount)
  {
    profileAnalysis[profileIndex] = ProfileAnalysis();
  }
}

// Main serial command handler
void checkSerial()
{
  if (Serial.available() > 0)
  {
    String receivedInput = Serial.readStringUntil('\n');
    receivedInput.trim();

    if (receivedInput.startsWith("wifi"))
    {
      handleWiFiCommand(receivedInput);
    }
    else if (receivedInput.equalsIgnoreCase("upload"))
    {
      uploadProfile();
    }
    else
    {
      Serial.println("Unknown command. Available commands: 'wifi1:ssid1,password1', 'wifi2:ssid2,password2', 'upload'");
    }
  }
}
void connectToWiFi()
{
  // if (menu.menu_items_settings_bool[0] == true) {
  //   unsigned long currentMillis = millis();
  //   if (currentMillis - previousMillis >= interval) {
  //     previousMillis = currentMillis;
  //     if (WiFi.status() != WL_CONNECTED) {

  //       int numNetworks = WiFi.scanNetworks();
  //       if (numNetworks == 0) {
  //         Serial.println("No network found.");
  //         return;
  //       }

  //       for (int i = 0; i < numNetworks; i++) {
  //         String ssid = WiFi.SSID(i);
  //         int rssi = WiFi.RSSI(i);

  //         if (ssid.equals(ssid1)) {
  //           Serial.println("Connection to the first network...");
  //           WiFi.begin(ssid1, password1);
  //           while (WiFi.status() != WL_CONNECTED) {
  //             delay(1000);
  //             Serial.print(".");
  //           }

  //           if (WiFi.status() == WL_CONNECTED) {
  //             Serial.println("Connected to the first network!");
  //             return;
  //           }
  //         }

  //         if (ssid.equals(ssid2)) {
  //           Serial.println("Connection to the second network...");
  //           WiFi.begin(ssid2, password2);
  //           while (WiFi.status() != WL_CONNECTED) {
  //             delay(1000);
  //             Serial.print(".");
  //           }

  //           if (WiFi.status() == WL_CONNECTED) {
  //             Serial.println("Connected to the second network!");
  //             return;
  //           }
  //         }
  //       }

  //       Serial.println("Failed to connect to networks.");
  //       if (menu.menu_items_settings_bool[1] == true) {  // Change to a real settings item!!!
  //         connectToStrongestOpenWiFi();
  //       }
  //     }
  //   }
  // }
}
void connectToStrongestOpenWiFi()
{
  // Scan for available Wi-Fi networks
  int numNetworks = WiFi.scanNetworks();
  Serial.print("Found ");
  Serial.print(numNetworks);
  Serial.println(" networks");

  // Find the strongest open network
  int strongestSignal = -100; // Set a low initial value for signal strength
  int strongestIndex = -1;

  for (int i = 0; i < numNetworks; ++i)
  {
    if (WiFi.encryptionType(i) == WIFI_AUTH_OPEN)
    {
      int signal = WiFi.RSSI(i);
      Serial.print("Network: ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" | Signal strength: ");
      Serial.println(signal);

      if (signal > strongestSignal)
      {
        strongestSignal = signal;
        strongestIndex = i;
      }
    }
  }

  // Connect to the strongest open network if found
  if (strongestIndex != -1)
  {
    Serial.print("Connecting to: ");
    Serial.println(WiFi.SSID(strongestIndex));
    WiFi.begin(WiFi.SSID(strongestIndex).c_str());

    while (WiFi.status() != WL_CONNECTED)
    {
      delay(100);
      Serial.print(".");
    }

    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("No open Wi-Fi networks found");
  }
}
void drawReflowProfile(int mode)
{
  // Constants for graph layout
  const int marginLeft = 7;
  const int marginTop = 15;
  const int graphWidth = menu.getTftWidth() - 2 * marginLeft;
  const int graphHeight = menu.getTftHeight() - marginTop - 11;

  // Find the max time and temperature values
  int maxTime = 0;
  int maxTemp = 0;

  for (int i = 0; i < ReflowprofileSize; i++)
  {
    maxTime = max(maxTime, profiles[mode].steps[i][0]);
    maxTemp = max(maxTemp, profiles[mode].steps[i][1]);
  }

  // Calculate scaling factors
  float timeScale = static_cast<float>(graphWidth) / maxTime;
  float tempScale = static_cast<float>(graphHeight) / maxTemp;

  // Draw axes
  canvas.drawLine(0, marginTop + graphHeight, 160, marginTop + graphHeight, TFT_WHITE);

  // Draw temperature profile
  for (int i = 0; i < ReflowprofileSize - 1; i++)
  {
    int x1 = marginLeft + static_cast<int>(profiles[mode].steps[i][0] * timeScale);
    int y1 = marginTop + graphHeight - static_cast<int>(profiles[mode].steps[i][1] * tempScale);
    int x2 = marginLeft + static_cast<int>(profiles[mode].steps[i + 1][0] * timeScale);
    int y2 = marginTop + graphHeight - static_cast<int>(profiles[mode].steps[i + 1][1] * tempScale);

    canvas.drawLine(x1, y1, x2, y2, TFT_GREEN);
  }

  // Display temperature values and time labels
  for (int i = 0; i < ReflowprofileSize; i++)
  {
    int x = marginLeft + static_cast<int>(profiles[mode].steps[i][0] * timeScale);
    int y = marginTop + graphHeight - static_cast<int>(profiles[mode].steps[i][1] * tempScale);

    canvas.drawCircle(x, y, 2, TFT_RED);
    canvas.setCursor(x - 5, y - 10);
    canvas.setFreeFont(&TomThumb);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_RED);
    canvas.print(profiles[mode].steps[i][1]);

    // Display time labels at intervals of 30 seconds
    if (i == 0 || (profiles[mode].steps[i][0] - profiles[mode].steps[i - 1][0]) >= 30)
    {
      canvas.drawLine(x, marginTop - 4 + graphHeight + 2, x, menu.getTftHeight() - 10, TFT_WHITE); // Vertical line for time label
      if (x > 99)
      {
        canvas.setCursor(x - 5, 77);
      }
      else
      {
        canvas.setCursor(x - 3, 77);
      }
      canvas.setTextSize(1);
      canvas.setTextColor(TFT_WHITE);
      canvas.print(profiles[mode].steps[i][0]);
    }
  }
}
void checkForUpdate()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    http.begin("https://theyoungmaker.ddns.net/pages/Hotplate/api/hotplate_version.html");

    // Make HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);

      // Parse JSON data
      String payload = http.getString(); // Get the response payload as a String
      Serial.println(payload);

      StaticJsonDocument<48> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (error)
      {
        Serial.print("deserializeJson() failed: ");
        Serial.println(error.c_str());
        return;
      }

      availableVersion = doc["version"].as<float>();
      Serial.print("Current Version: ");
      Serial.println(currentVersion);
      Serial.print("Available Version: ");
      Serial.println(availableVersion);
    }
    else
    {
      Serial.print("HTTP Request failed: ");
      Serial.println(httpResponseCode);
    }

    http.end(); // Close connection
    if (availableVersion > currentVersion)
    {
      updateAvailable = true;
    }
    if (updateAvailable)
    {
      tft.fillScreen(TFT_BLACK);
      // tft.drawRGBBitmap(0, 0, (uint16_t*)Update_available, 160, 80);

      delay(7000);
      update();
    }
  }
}
void update()
{
  // if (menu.menu_items_settings_bool[5]) {
  //   tft.fillScreen(TFT_BLACK);
  //   tft.setFreeFont(&FreeMonoBold9pt7b);
  //   tft.setTextSize(1);
  //   tft.setTextColor(TFT_GREEN);
  //   tft.setCursor(22, 42);
  //   tft.print("Updating...");

  //   httpUpdate.rebootOnUpdate(false);

  //   Serial.println(F("Update start now!"));

  //   // t_httpUpdate_return ret = ESPhttpUpdate.update(client, "http://192.168.1.125:3000/firmware/httpUpdateNew.bin");
  //   // Or:
  //   t_httpUpdate_return ret = httpUpdate.update(client, "192.168.0.118", 5500, "/Hotplate.ino.bin");

  //   switch (ret) {
  //     case HTTP_UPDATE_FAILED:
  //       Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
  //       tft.fillScreen(TFT_BLACK);
  //       tft.setTextColor(TFT_RED);
  //       tft.setCursor(8, 42);
  //       tft.print("Update Failed");
  //       delay(4000);
  //       break;

  //     case HTTP_UPDATE_NO_UPDATES:
  //       Serial.println("HTTP_UPDATE_NO_UPDATES");
  //       break;

  //     case HTTP_UPDATE_OK:
  //       Serial.println("HTTP_UPDATE_OK");
  //       tft.fillScreen(TFT_BLACK);
  //       tft.setTextColor(TFT_GREEN);
  //       tft.setCursor(38, 42);
  //       tft.print("Updated!");
  //       delay(1000);  // Wait a second and restart
  //       ESP.restart();
  //       break;
  //   }
  // }
}
void checkTemperature()
{
  // //Only to temporarily test feature using temp
  // if (heating) {
  //   currentTemp += random(1, 3);
  // } else if (!heating && currentTemp > 0) {

  //   currentTemp -= 1;
  // }

  // Serial.println("Checking temp...");
  // int temp = analogRead(thermistorPin);
  // Serial.print("Temperature is:");
  // Serial.print(temp);

  // int ThermistorPin = 34;  // Pin connected to the thermistor
  // double adcMax = 4095.0;  // Maximum value of the ADC (analog-to-digital converter)
  // double Vs = 3.3;         // Supply voltage

  // double R1 = 10000.0;   // Voltage divider resistor value
  // double Beta = 3478.0;  // Beta value of the thermistor
  // double To = 298.15;    // Temperature in Kelvin for 25 degrees Celsius
  // double Ro = 10500.0;   // Resistance of the thermistor at 25 degrees Celsius

  // // Variables for calculations
  // double Vout, Rt = 0;
  // double T, Tc, Tf = 0;
  // double totalTemperature = 0;  // Accumulator for summing up temperature readings

  // // Read the temperature three times and calculate the average
  // for (int i = 0; i < 3; i++) {
  //   // Read analog input and convert it to voltage
  //   Vout = analogRead(ThermistorPin) * Vs / adcMax;

  //   // Calculate resistance of the thermistor
  //   Rt = R1 * Vout / (Vs - Vout);

  //   // Calculate temperature using the Steinhart-Hart equation
  //   T = 1 / (1 / To + log(Rt / Ro) / Beta);  // Temperature in Kelvin

  //   // Convert temperature to Celsius
  //   Tc = T - 273.15;  // Celsius

  //   // Check if temperature is below 0 Celsius
  //   if (Tc <= 0) {
  //     // Re-read the temperature if it's below or equal to 0 Celsius
  //     Vout = analogRead(ThermistorPin) * Vs / adcMax;
  //     Rt = R1 * Vout / (Vs - Vout);
  //     T = 1 / (1 / To + log(Rt / Ro) / Beta);  // Temperature in Kelvin
  //     Tc = T - 273.15;                         // Celsius
  //   } else {
  //     // Add current temperature reading to the accumulator
  //     totalTemperature += Tc;
  //   }
  // }

  // // Calculate the average temperature over the three readings
  // currentTemp = totalTemperature / 3;

  // // Print the temperature
  // Serial.print("Temperature: ");
  // Serial.println(Tc);
  // Serial.println(" °C");

  // Read the temperature from the thermocouple
  currentTemp = thermocouple.readCelsius();
  Serial.print("Internal Temp:");
  Serial.println(thermocouple.readInternal());
  // Check for errors
  if (isnan(currentTemp))
  {
    Serial.println("Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    if (e & MAX31855_FAULT_OPEN)
      Serial.println("FAULT: Thermocouple is open - no connections.");
    if (e & MAX31855_FAULT_SHORT_GND)
      Serial.println("FAULT: Thermocouple is short-circuited to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC)
      Serial.println("FAULT: Thermocouple is short-circuited to VCC.");
    // Safety: turn off heater on sensor fault
    heaterWrite(false);
    return;
  }

  // Safety: hard cutoff if above configured maximum
  if (currentTemp >= MAXPOSSIBLETEMP)
  {
    Serial.println("Safety cutoff: temperature exceeded maximum.");
    heaterWrite(false);
  }

  // Print the temperature
  Serial.print("Temperature: ");
  Serial.print(currentTemp);
  Serial.println(" °C");
}
void reflow(int profile)
{
  // Initialize timing
  if (reflowStepStart == 0)
  {
    reflowStepStart = millis();
  }
  unsigned long elapsedMs = millis() - reflowStepStart;
  int elapsedSec = int(elapsedMs / 1000UL);

  // Determine target temperature via interpolation
  targetTemp = interpolateTargetTemp(profile, elapsedSec);

  // Update SSR control via PID
  updateHeaterControl(targetTemp);

  // Determine phase for logging/UI
  ProfilePhase phase = getProfilePhase(profile, elapsedSec);
  switch (phase)
  {
  case PreheatPhase:
    Serial.println("Preheating");
    break;
  case SoakPhase:
    Serial.println("Soaking");
    break;
  case ReflowPhase:
    Serial.println("Reflowing");
    break;
  case CooldownPhase:
    Serial.println("Cooldown");
    break;
  case DonePhase:
    Serial.println("Done");
    break;
  }

  // Finish when past profile end time
  analyzeProfile(profile);
  if (elapsedSec >= profileAnalysis[profile].endTime)
  {
    heaterWrite(false);
    heatingStarted = false;
    reflowing = false;
    reflowStepStart = 0;
    ringBuzzer(3, 1, 2);
  }
}
void getHeatingTimeString(String &timeString)
{
  static unsigned long startTime = 0; // Static variable to store the start time
  unsigned long currentTime;
  unsigned long elapsedTime;
  unsigned int minutes;
  unsigned int seconds;

  if (reflowing)
  {
    if (startTime == 0)
    {
      // If heating becomes true and startTime is 0, set the start time
      startTime = millis();
    }

    currentTime = millis();
    elapsedTime = (currentTime - startTime) / 1000;
    minutes = elapsedTime / 60;
    seconds = elapsedTime % 60;

    // Format the time as "min:sec"
    timeString = String(minutes) + ":" + ((seconds < 10) ? "0" : "") + String(seconds);
  }
  else
  {
    // If heating is false, reset the start time
    startTime = 0;
    timeString = "0:00";
  }
}
void setNeopixelColor(int red, int green, int blue)
{
  for (int led = 0; led < NUM_LEDS; led++)
  {
    leds[led] = CRGB(red, green, blue);
    FastLED.show();
  }
}
void setLedsFromTemp()
{
  // Échelle de couleur en fonction de la température
  int blue = constrain(map(currentTemp, 0, 300, 255, 0), 0, 255);
  int red = 255 - blue;
  int green = 0;

  setNeopixelColor(red, green, blue);
}
void ringBuzzer(int time, int delay1, int delay2)
{ // Time = # of time to ring the buzzer; Delay1 = in seconds for how long the buzzer is ringing; Delay2 = in seconds for how long the delay between the rings of the buzzer (0 if buzzer is set to ring only one time)
  delay1 = delay1 * 1000;
  delay2 = delay2 * 1000;
  if (time == 1)
  {
    delay2 = 0;
  }
  for (int i = 0; i < time; i++)
  { // TODO : Modify so that it uses Non-Blocking methods (Millis)
    digitalWrite(buzzer_pin, HIGH);
    delay(delay1);
    digitalWrite(buzzer_pin, LOW);
    delay(delay2);
  }
}

// -------------------- Heater control helpers (SSR time-proportional) --------------------
void heaterWrite(bool on)
{
  digitalWrite(heatingElementPin, on ? HIGH : LOW);
  heating = on;
}

void resetPid()
{
  pidIntegral = 0.0;
  pidLastError = 0.0;
  pidLastTime = millis();
  pidWindowStart = millis();
}

void updateHeaterControl(float targetCelsius)
{
  // Call this frequently (e.g., every loop) after currentTemp is updated
  unsigned long now = millis();
  if (pidLastTime == 0)
    pidLastTime = now;
  double dt = (now - pidLastTime) / 1000.0; // seconds
  pidLastTime = now;

  // Compute PID
  double error = targetCelsius - currentTemp;
  pidIntegral += error * dt;
  // Anti-windup clamp
  pidIntegral = constrain(pidIntegral, -500.0, 500.0);
  double derivative = (error - pidLastError) / max(dt, 1e-3);
  pidLastError = error;

  double output = Kp * error + Ki * pidIntegral + Kd * derivative; // Arbitrary units
  // Map output to 0..1 duty
  double duty = constrain(output / 300.0, 0.0, 1.0); // Scale factor chosen for typical deltas

  // Time-proportional window
  if (now - pidWindowStart >= pidWindowSizeMs)
  {
    pidWindowStart += pidWindowSizeMs;
  }
  unsigned long onTime = (unsigned long)(duty * pidWindowSizeMs);
  unsigned long elapsedInWindow = now - pidWindowStart;
  bool on = elapsedInWindow < onTime;
  heaterWrite(on);
}

void resetReflowState()
{
  reflowStep = 0; // retained for compatibility but unused for phase now
  reflowStepStart = millis();
}

void checkHotplateWear()
{
  // Variables initiales
  float initialHeatingTime = 100.0;    // Temps initial en secondes
  float initialMaxTemperature = 300.0; // Température maximale initiale en °C

  // Variables actuelles
  float currentHeatingTime = 120.0;    // Temps actuel en secondes
  float currentMaxTemperature = 270.0; // Température maximale actuelle en °C

  // Pondération des facteurs
  float heatingTimeWeight = 0.3;    // Poids du temps de chauffe (30 %)
  float maxTemperatureWeight = 0.7; // Poids de la température max (70 %)

  // Calcul des variations
  float heatingTimeIncreasePercentage = ((currentHeatingTime - initialHeatingTime) / initialHeatingTime) * 100;
  float maxTemperatureDecreasePercentage = ((initialMaxTemperature - currentMaxTemperature) / initialMaxTemperature) * 100;

  // Calcul de l'usure globale
  float wearPercentage = (heatingTimeIncreasePercentage * heatingTimeWeight) + (maxTemperatureDecreasePercentage * maxTemperatureWeight);

  // Affichage des résultats
  Serial.print("Augmentation du temps de chauffe : ");
  Serial.print(heatingTimeIncreasePercentage);
  Serial.println("%");

  Serial.print("Diminution de la température maximale : ");
  Serial.print(maxTemperatureDecreasePercentage);
  Serial.println("%");

  Serial.print("Usure globale estimée : ");
  Serial.print(wearPercentage);
  Serial.println("%");
}
void addProfileMenuItems()
{
  static bool itemsAdded = false; // Static variable to ensure items are added only once

  if (!itemsAdded)
  {
    for (int i = 0; i < reflowProfileCount; i++)
    {
      // Adding a menu item for each profile, passing a reference to the corresponding screen
      profilesMenu.addItem(profiles[i].name, &profilesScreen);
    }

    itemsAdded = true; // Set the flag to true after items are added
  }

  menu.redirectToScreen(&profilesMenu);
}

void setupScreens()
{
  reflowScreen.customDraw = []()
  {
    Serial.print("currentTemp:");
    Serial.print(currentTemp);
    Serial.print(",");
    Serial.print("targetTemp:");
    Serial.println(targetTemp);
    setLedsFromTemp();
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillisTemperature >= temperatureInterval)
    {
      checkTemperature();
      previousMillisTemperature = currentMillis;
    }

    if (!reflowing && (digitalRead(menu.UpButton()) == HIGH) && (up_button_clicked == 0))
    {
      // Change reflow profile
      selectedReflowProfile++;
      if (selectedReflowProfile >= reflowProfileCount)
      {
        selectedReflowProfile = 0; // Wrap around to the first profile
      }
      up_button_clicked = 1; // Mark button as clicked
    }

    // Reset button state (debouncing logic)
    if (digitalRead(menu.UpButton()) == LOW)
    {
      up_button_clicked = 0; // Reset button state for the next click
    }

    // Save profile only during reflow
    if (reflowing && selectedReflowProfile != last_profile)
    {
      Serial.print("saving profile");
      prefs.putInt("last_profile", selectedReflowProfile); // Save when reflowing
      last_profile = selectedReflowProfile;                // Update in-memory variable to avoid redundant saves
    }

    static unsigned long select_button_press_time_reflow = 0;

    if (digitalRead(menu.SelectButton()) == HIGH)
    {
      if (button_select_clicked_reflow == 0)
      {
        select_button_press_time_reflow = millis(); // Start measuring button press time
        button_select_clicked_reflow = 1;
      }
    }
    else if (digitalRead(menu.SelectButton()) == LOW)
    {
      if (button_select_clicked_reflow == 1)
      {
        // Only perform action if it was a short press (less than 250ms)
        if ((millis() - select_button_press_time_reflow) < 250)
        {
          heatingStarted = !heatingStarted;
          reflowing = !reflowing;
          static bool first = true;
          if (heatingStarted)
          {
            resetPid();
            resetReflowState();
            first = true; // allow re-init inside draw loop code path
          }
          else
          {
            heaterWrite(false);
            first = true;
          }
        }
        button_select_clicked_reflow = 0; // Reset the button state
      }
    }

    String formattedTime;
    getHeatingTimeString(formattedTime);
    int16_t time_width = canvas.textWidth(formattedTime);
    canvas.pushImage(tft_width - time_width - 17, 7, 11, 12, Clock); // Distance between text and icon is 15 + 2 (2 for the padding of the text)
    canvas.setTextColor(0x4EF0);
    canvas.setTextSize(1);
    canvas.setFreeFont();
    canvas.drawString(formattedTime, tft_width - time_width - 2, 9); // The 2 is for padding
    String heatTempStr = String(targetTemp) + " C";
    int16_t target_temp_width = canvas.textWidth(heatTempStr);
    // Calculate x position dynamically
    int x_target_temp = tft_width - target_temp_width - 2;
    // Draw target temperature circle and rectangle
    canvas.pushImage(x_target_temp - 10 - 5, 32, 12, 12, Target);

    canvas.setTextColor(0xFFFF);
    canvas.drawString(heatTempStr, x_target_temp, 34);
    String currentTempStr = String(currentTemp);

    canvas.setTextSize(2);
    int16_t current_temp_width = canvas.textWidth(currentTempStr);

    canvas.drawString(currentTempStr, 24, 30);
    canvas.pushImage(24 + current_temp_width + 2, 30, 20, 15, c_icon);
    canvas.setTextColor(0x653F);
    canvas.setTextSize(1);
    // menu.scrollTextHorizontal(22, 9, profiles[selectedReflowProfile].name, TFT_WHITE, TFT_BLACK, 1, 50, 100); // Does not work correctly du to the library
    canvas.drawString(profiles[selectedReflowProfile].name, 22, 9);
    canvas.pushImage(6, 7, 12, 12, Reflow);
    canvas.pushImage(8, 32, 9, 11, Fire);

    // Progress based on elapsed time vs profile end time
    analyzeProfile(selectedReflowProfile);
    int endSec = profileAnalysis[selectedReflowProfile].endTime;
    int elapsedSec = (reflowStepStart == 0) ? 0 : int((millis() - reflowStepStart) / 1000UL);
    int progressPx = (endSec > 0) ? constrain(int((float)elapsedSec / (float)endSec * 150.0f), 0, 150) : 0;
    canvas.fillSmoothRoundRect(5, 60, 150, 10, 10 / 2, 0x320A, TFT_BLACK);        // Progress bar background
    canvas.fillSmoothRoundRect(5, 60, progressPx, 10, 10 / 2, 0xFC87, TFT_BLACK); // Progress bar

    if (heatingStarted)
    {
      // Initialize timing when starting
      static bool first = true;
      if (first)
      {
        resetPid();
        first = false;
      }
      reflow(selectedReflowProfile);

      // Optional: show current phase text
      int phaseSec = (reflowStepStart == 0) ? 0 : int((millis() - reflowStepStart) / 1000UL);
      ProfilePhase phase = getProfilePhase(selectedReflowProfile, phaseSec);
      const char *phaseText = "";
      switch (phase)
      {
      case PreheatPhase:
        phaseText = "Preheat";
        break;
      case SoakPhase:
        phaseText = "Soak";
        break;
      case ReflowPhase:
        phaseText = "Reflow";
        break;
      case CooldownPhase:
        phaseText = "Cooldown";
        break;
      case DonePhase:
        phaseText = "Done";
        break;
      }
      canvas.setTextColor(0xFFFF);
      canvas.setTextSize(1);
      canvas.drawString(phaseText, 6, 48);
    }
  };
  heatScreen.customDraw = []()
  {
    setLedsFromTemp();
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillisTemperature >= temperatureInterval)
    {
      checkTemperature();
      previousMillisTemperature = currentMillis;
    }

    bool buttonIsPressed = (digitalRead(menu.UpButton()) == HIGH);

    // Button just pressed
    if (buttonIsPressed && !heatButtonWasPressed)
    {
      heatButtonPressStartTime = millis();
      heatButtonWasPressed = true;
    }

    // Button is being held
    else if (buttonIsPressed && heatButtonWasPressed)
    {
      unsigned long pressDuration = millis() - heatButtonPressStartTime;

      // Long press detected
      if (pressDuration >= 600)
      { // Time in ms to trigger long press
        heatLongPressActive = true;

        // Increase temperature faster during long press
        if (targetTemp <= MAXPOSSIBLETEMP)
        {
          targetTemp = min(targetTemp + 5, MAXPOSSIBLETEMP);
          delay(100); // Delay between increments during long press
        }
      }
    }

    // Button released
    else if (!buttonIsPressed && heatButtonWasPressed)
    {
      // If it wasn't a long press, handle as single click
      if (!heatLongPressActive && targetTemp <= MAXPOSSIBLETEMP)
      {
        targetTemp = min(targetTemp + 5, MAXPOSSIBLETEMP);
      }

      // Reset button state
      heatButtonWasPressed = false;
      heatLongPressActive = false;
    }
    // Update SSR control via PID
    updateHeaterControl(targetTemp);

    String formattedTime;
    getHeatingTimeString(formattedTime);

    // Calculate dimensions for time display
    int16_t time_width = canvas.textWidth(formattedTime);
    int16_t time_x_pos = tft_width - time_width - 2;

    // Draw time icon and formatted time
    canvas.pushImage(tft_width - time_width - 17, 7, 11, 12, Clock); // Distance between text and icon is 17 (15 + 2 padding)
    canvas.setTextColor(0x4EF0);
    canvas.setTextSize(1);
    canvas.setFreeFont();
    canvas.drawString(formattedTime, time_x_pos, 9); // Padding of 2

    // Draw target temperature
    canvas.setTextSize(2);
    String heatTempStr = String(targetTemp);
    int16_t target_temp_width = canvas.textWidth(heatTempStr);
    int16_t x_target_temp = tft_width - target_temp_width - 22; // Dynamic position for target temp

    canvas.pushImage(tft_width - 22, 30, 20, 15, c_icon);     // Icon
    canvas.pushImage(x_target_temp - 15, 32, 12, 12, Target); // Adjusted for padding
    canvas.setTextColor(0xFFFF);
    canvas.drawString(heatTempStr, x_target_temp, 30); // Target temp text

    // Draw current temperature
    String currentTempStr = String(currentTemp);
    int16_t current_temp_width = canvas.textWidth(currentTempStr);

    canvas.drawString(currentTempStr, 24, 30);                         // Current temp text
    canvas.pushImage(24 + current_temp_width + 2, 30, 20, 15, c_icon); // Current temp icon

    // Draw manual heat text and fire icon
    canvas.setTextColor(0x653F);
    canvas.setTextSize(1);
    canvas.drawString("Manual Heat", 2, 9);
    canvas.pushImage(8, 32, 9, 11, Fire);

    // Calculate and draw progress bar
    int progress = (targetTemp > 0)
                       ? static_cast<int>((static_cast<float>(currentTemp) / targetTemp) * 150)
                       : 0;

    canvas.fillSmoothRoundRect(5, 60, 150, 10, 10 / 2, 0x320A, TFT_BLACK);      // Background
    canvas.fillSmoothRoundRect(5, 60, progress, 10, 10 / 2, 0xFC87, TFT_BLACK); // Progress bar
  };
  profilesScreen.customDraw = []()
  {
    Serial.print(profiles[profilesMenu.getIndex()].name);
    canvas.fillScreen(TFT_BLACK);
    drawReflowProfile(profilesMenu.getIndex());
  };
  // calibrationScreen.customDraw = []() {
  //   canvas.drawSmoothRoundRect(1, 1, 4, 4, 100, 10, TFT_WHITE, TFT_ORANGE);
  //   canvas.drawString(menu.getLibraryVersion(), 10, 50);
  // };
}
void setup()
{
  // --- Serial initialization ---
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== BOOTING OpenHotplate ===");
  Serial.flush();

  // --- Step 1: Create Settings screen ---
  Serial.println("[1] Creating SettingsScreen...");
  SettingsScreen *settingsScreen = new SettingsScreen("Settings");
  if (!settingsScreen) {
    Serial.println("ERROR: Failed to allocate SettingsScreen!");
  } else {
    settingsScreen->addBooleanSetting("Auto Update", true);
    settingsScreen->addBooleanSetting("Leds", true);
    settingsScreen->addBooleanSetting("Wifi", true);
    Serial.println("OK: SettingsScreen created and populated.");
  }
  Serial.flush();

  // --- Step 2: Build main menu ---
  Serial.println("[2] Building mainMenu items...");
  mainMenu.addItem("Reflow", &reflowScreen, nullptr, (const uint16_t *)Menu_icon_1);
  mainMenu.addItem("Heat", &heatScreen, nullptr, (const uint16_t *)Menu_icon_2);
  mainMenu.addItem("Profiles", nullptr, addProfileMenuItems, (const uint16_t *)Menu_icon_3);
  mainMenu.addItem("Settings", settingsScreen, nullptr, (const uint16_t *)Menu_icon_4);
  Serial.println("OK: mainMenu built.");
  Serial.flush();

  // --- Step 3: Setup screens ---
  Serial.println("[3] Running setupScreens()...");
  setupScreens();
  Serial.println("OK: setupScreens done.");
  Serial.flush();

  // --- Step 4: Configure menu appearance ---
  Serial.println("[4] Configuring menu visual...");
  menu.setMenuStyle(1);
  menu.setSelectionBorderColor(0xfa60);
  menu.setSelectionFillColor(0xfa60);
  menu.setScrollbarStyle(1);
  menu.setScrollbar(true);
  Serial.println("OK: Menu visual config done.");
  Serial.flush();

  // --- Step 5: Initialize menu system ---
  Serial.println("[5] menu.begin()...");
    digitalWrite(13, HIGH); // Device ignores SPI commands
    digitalWrite(10, LOW); // Device ignores SPI commands

  menu.begin(&mainMenu);
  Serial.println("OK: menu.begin() complete.");
  menu.setButtonsMode("high");
  Serial.println("OK: Buttons mode set.");
  Serial.flush();

  // --- Step 6: Configure input pins ---
  Serial.println("[6] Configuring button pins...");
  pinMode(menu.UpButton(), INPUT_PULLDOWN);
  pinMode(menu.SelectButton(), INPUT_PULLDOWN);
  Serial.println("OK: Button pins ready.");
  Serial.flush();

  // --- Step 7: Initialize LEDs ---
  Serial.println("[7] Initializing LEDs...");
  FastLED.addLeds<WS2812, led_pin, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(200);
  Serial.println("OK: LEDs initialized.");
  Serial.flush();

  // --- Step 8: Configure other I/O ---
  Serial.println("[8] Configuring other I/O pins...");
  // pinMode(buzzer_pin, OUTPUT);
  // digitalWrite(buzzer_pin, LOW);
  // pinMode(thermistorPin, INPUT);
  // pinMode(heatingElementPin, OUTPUT);
  Serial.println("OK: I/O pins configured.");
  Serial.flush();

  // --- Step 9: Firmware update check ---
  Serial.println("[9] Checking for firmware updates...");
  checkForUpdate();
  Serial.println("OK: Firmware update check done.");
  Serial.flush();

  // --- Step 10: Preferences initialization ---
  Serial.println("[10] Opening preferences storage...");
  if (!prefs.begin("OpenHotplate", false)) {
    Serial.println("ERROR: Failed to open preferences!");
    return;
  }
  Serial.println("OK: Preferences opened.");
  Serial.flush();

  // --- Step 11: Initialize heater + PID ---
  Serial.println("[11] Initializing heater and PID...");
  heaterWrite(false);
  resetPid();
  Serial.println("OK: Heater off, PID reset.");
  Serial.flush();

  // --- Step 12: Test thermocouple communication ---
  digitalWrite(13, HIGH); // Device ignores SPI commands

  Serial.println("[12] Testing MAX31855...");
  double testTemp = thermocouple.readCelsius();
  if (isnan(testTemp)) {
    Serial.println("ERROR: Thermocouple fault(s) detected!");
    uint8_t e = thermocouple.readError();
    if (e & MAX31855_FAULT_OPEN)
      Serial.println("  → FAULT: Thermocouple is open - no connection.");
    if (e & MAX31855_FAULT_SHORT_GND)
      Serial.println("  → FAULT: Thermocouple shorted to GND.");
    if (e & MAX31855_FAULT_SHORT_VCC)
      Serial.println("  → FAULT: Thermocouple shorted to VCC.");
  } else {
    Serial.print("OK: MAX31855 initialized. Test temp = ");
    Serial.print(testTemp);
    Serial.println(" °C");
  }
  Serial.flush();

  // --- Step 13: Load last profile ---
  Serial.println("[13] Loading last profile...");
  selectedReflowProfile = prefs.getInt("last_profile", 0);
  Serial.print("OK: Last profile index = ");
  Serial.println(selectedReflowProfile);
  Serial.flush();

  // --- Setup complete ---
  Serial.println("=== SETUP COMPLETE ===");
  Serial.flush();
}


void loop()
{
  menu.loop();     // Handle menu logic
  checkSerial();   // Check for serial input
  connectToWiFi(); // Attempt to connect to WiFi if enabled
}