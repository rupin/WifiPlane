//***************************************************
// Differential Thrust Flight Controller (Access Point Mode)
//***************************************************

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define P_ID 1
#define ST_LED  2
#define L_MOTOR 5
#define R_MOTOR 4

#define DC_RSSI 1500  // Time in mS for sending telemetry
#define DC_RX   900   // Time in mS for tx inactivity (App timeout Failsafe)

ADC_MODE(ADC_VCC);

// State tracking variables
unsigned long premillis_rssi = 0;
unsigned long premillis_rx   = 0;
bool motors_active = false;
bool app_was_connected = false;

// Network variables
unsigned int localPort = 6000;    // local port to listen on
unsigned int remotPort = 2390;    // local port to talk on
IPAddress remotIp;                // Will be set to broadcast IP

char packetBuffer[32]; 
char replyBuffer[]={P_ID,0x01,0x01,0x00}; // telemetry payload
WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n\n--- Differential Thrust Flight Controller ---");
  
  // ==========================================
  // HARDWARE SETUP
  // ==========================================
  analogWriteRange(255);
  
  pinMode(L_MOTOR, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  analogWrite(L_MOTOR, 0);
  analogWrite(R_MOTOR, 0);
  
  pinMode(ST_LED, OUTPUT);
  digitalWrite(ST_LED, HIGH); // Turn LED off initially (assuming active-LOW)

  // ==========================================
  // DYNAMIC ACCESS POINT SETUP
  // ==========================================
  Serial.println("Configuring Access Point...");
  
  // 1. Set ESP8266 to Access Point mode
  WiFi.mode(WIFI_AP);
  
  // 2. Fetch the unique MAC address of this specific chip
  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  
  // 3. Generate a unique network name (e.g., "Plane_A1B2")
  char dynamic_ssid[20];
  sprintf(dynamic_ssid, "Plane_%02X%02X", mac[4], mac[5]); 
  
  // 4. Start the hotspot with an open network (no password)
  WiFi.softAP(dynamic_ssid, ""); 
  
  Serial.print("Access Point Started! Connect phone to: ");
  Serial.println(dynamic_ssid);
  
  // In AP Mode, the ESP's IP address is always 192.168.4.1
  Serial.print("ESP IP Address: ");
  Serial.println(WiFi.softAPIP()); 

  // ==========================================
  // UDP & NETWORK CONFIGURATION
  // ==========================================
  // Set the telemetry to broadcast to any device connected to the plane's network
  remotIp = IPAddress(192, 168, 4, 255); 
  
  Udp.begin(localPort);
  Serial.print("Listening for App Control Data on UDP Port: ");
  Serial.println(localPort);
  
  // Turn LED on to indicate the system is fully ready
  digitalWrite(ST_LED, LOW); 
  
  Serial.println("System Ready. Waiting for App Data...");
  Serial.println("----------------------------------------");
}

void loop() {
  delay(5); // Critical for ESP8266 background Wi-Fi tasks
  
  // ==========================================
  // LAYER 1: WI-FI RANGE FAILSAFE (AP MODE)
  // ==========================================
  // In AP mode, we check how many devices (stations) are connected to the ESP.
  // If it's greater than 0, a phone is connected to the plane.
  int clients_connected = WiFi.softAPgetStationNum();
  
  if(clients_connected > 0)
  {
    if (!app_was_connected) {
      Serial.println("SUCCESS: Phone Connected to Plane!");
      app_was_connected = true;
    }
    
    digitalWrite(ST_LED, LOW); // Solid LED means connected
    
    // ==========================================
    // DATA PARSING & MIXING ALGORITHM
    // ==========================================
    int packetSize = Udp.parsePacket();
    if (packetSize) 
    {
      int len = Udp.read(packetBuffer, 31);
      if (len > 0) 
      {
        packetBuffer[len] = '\0'; 

        Serial.print("Received Data: ");
        Serial.println(packetBuffer);
        
        int parsed_pid, parsed_throttle, parsed_yaw;
        int parsed_count = sscanf(packetBuffer, "[%d, %d, %d]", &parsed_pid, &parsed_throttle, &parsed_yaw);
        
        if (parsed_count == 3 && parsed_pid == P_ID)
        {
          // 1. Establish Base Speed from Throttle (0-128 converted to 0-255)
          int base_speed = parsed_throttle * 2;
          if (base_speed > 255) base_speed = 255;
          if (base_speed < 0) base_speed = 0;

          // 2. Initialize both motors to base speed
          int out_l = base_speed;
          int out_r = base_speed;

          parsed_yaw = constrain(parsed_yaw, 0, 128);          

          // 3. Apply Differential Thrust (Slowing the inner motor)
          if (parsed_yaw < 64) {
            // Turning Left: Slow down the Left Motor proportionally
            out_l = (base_speed * parsed_yaw) / 64;
          } 
          else if (parsed_yaw > 64) {
            // Turning Right: Slow down the Right Motor proportionally
            out_r = (base_speed * (128 - parsed_yaw)) / 64;
          }

          // Constrain final outputs
          out_l = constrain(out_l, 0, 255);
          out_r = constrain(out_r, 0, 255);

          // 4. Send signals to hardware
          analogWrite(L_MOTOR, out_l);
          analogWrite(R_MOTOR, out_r);
          
          motors_active = true;
          premillis_rx = millis(); // Reset the data failsafe timer

          // Uncomment below to debug mixing results in real-time
          /*
          Serial.print("APP -> Thr: "); Serial.print(parsed_throttle);
          Serial.print(" | Yaw: "); Serial.print(parsed_yaw);
          Serial.print("  ||  MOTORS -> L: "); Serial.print(out_l);
          Serial.print(" | R: "); Serial.println(out_r);
          */
        }
      }
    }
    
    // ==========================================
    // LAYER 2: APP TIMEOUT FAILSAFE
    // ==========================================
    // Even if the phone is connected to Wi-Fi, the app might have crashed or minimized.
    // If no UDP data is received for 900ms, kill the motors.
    if(millis() - premillis_rx > DC_RX)
    {
       if (motors_active) {
         analogWrite(L_MOTOR, 0);
         analogWrite(R_MOTOR, 0);
         motors_active = false;
         Serial.println("!!! FAILSAFE: No App Data for 900ms. Motors STOPPED. !!!");
       }
    }

    // ==========================================
    // TELEMETRY BROADCAST
    // ==========================================
    if(millis() - premillis_rssi > DC_RSSI)
    {
       premillis_rssi = millis();
       
       // Measure the internal voltage
       float vcc = (((float)ESP.getVcc()/(float)1024.0)+0.75f);
       
       // Convert the float to a pure string, e.g., "3.85"
       char telemetryStr[10];
       sprintf(telemetryStr, "%.2f", vcc);
       
       // Send to App
       Udp.beginPacket(remotIp, remotPort);
       Udp.write(telemetryStr); 
       Udp.endPacket();
     }
  }
  else
  {
    // ==========================================
    // NO PHONE CONNECTED BEHAVIOR
    // ==========================================
    if (app_was_connected || motors_active) {
      analogWrite(L_MOTOR, 0);
      analogWrite(R_MOTOR, 0);
      motors_active = false;
      app_was_connected = false;
      Serial.println("!!! CRITICAL FAILSAFE: Phone Disconnected! Motors STOPPED. !!!");
    }
    
    // Blink LED to indicate waiting for connection
    digitalWrite(ST_LED, LOW);
    delay(60);
    digitalWrite(ST_LED, HIGH);
    delay(1000);
  }
}
