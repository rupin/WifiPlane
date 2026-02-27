//***************************************************
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

#define P_ID 1
#define ST_LED  2
#define L_MOTOR 5
#define R_MOTOR 4
#define DC_RSSI 1500  // Time in mS for send RSSI
#define DC_RX   900   // Time in mS for tx inactivity (Failsafe Timeout)

ADC_MODE(ADC_VCC);

// State tracking variables
unsigned long premillis_rssi = 0;
unsigned long premillis_rx   = 0;
bool motors_active = false;
bool wifi_was_connected = false;

char ssid[] = "wifiplane";        // your network SSID (name)
char pass[] = "wifiplane1234";    // your network password 
IPAddress remotIp;
unsigned int localPort = 6000;    // local port to listen on
unsigned int remotPort = 2390;    // local port to talk on

char packetBuffer[32]; 
char replyBuffer[]={P_ID,0x01,0x01,0x00}; // telemetry payload
WiFiUDP Udp;

void setup() {
  Serial.begin(115200);
  delay(10);
  Serial.println("\n\n--- Differential Thrust Flight Controller ---");
  
  WiFi.mode(WIFI_STA);
  analogWriteRange(255);
  
  pinMode(L_MOTOR, OUTPUT);
  pinMode(R_MOTOR, OUTPUT);
  analogWrite(L_MOTOR, 0);
  analogWrite(R_MOTOR, 0);
  
  pinMode(ST_LED, OUTPUT);
  digitalWrite(ST_LED, HIGH);
  
  Serial.print("Connecting to Hotspot: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) 
  {
    digitalWrite(ST_LED, LOW);
    delay(60);
    digitalWrite(ST_LED, HIGH);
    delay(1000);
    Serial.print(".");
  }
  
  wifi_was_connected = true;
  Serial.println("\nWiFi Connected!");
  Serial.print("Assigned IP Address: ");
  Serial.println(WiFi.localIP());
  
  remotIp = WiFi.localIP();
  remotIp[3] = 255; 
  Udp.begin(localPort);
  Serial.println("System Ready. Waiting for App Data...");
  Serial.println("----------------------------------------");
}

void loop() {
  delay(5);
  
  // ==========================================
  // LAYER 1: WI-FI RANGE FAILSAFE
  // ==========================================
  if(WiFi.status() == WL_CONNECTED)
  {
    if (!wifi_was_connected) {
      Serial.println("SUCCESS: WiFi Reconnected!");
      wifi_was_connected = true;
    }
    
    digitalWrite(ST_LED, LOW);
    int packetSize = Udp.parsePacket();
    
    // ==========================================
    // DATA PARSING & MIXING ALGORITHM
    // ==========================================
    if (packetSize) 
    {
      int len = Udp.read(packetBuffer, 31);
      if (len > 0) 
      {
        packetBuffer[len] = '\0'; 
        
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

          // 3. Apply Differential Thrust (Slowing the inner motor)
          if (parsed_yaw < 64) {
            // Turning Left: Slow down the Left Motor proportionally
            // Example: Yaw 0 = 0% speed. Yaw 32 = 50% speed.
            out_l = (base_speed * parsed_yaw) / 64;
          } 
          else if (parsed_yaw > 64) {
            // Turning Right: Slow down the Right Motor proportionally
            // Example: Yaw 128 = 0% speed. Yaw 96 = 50% speed.
            out_r = (base_speed * (128 - parsed_yaw)) / 64;
          }

          // Constrain final outputs just to be absolutely safe
          out_l = constrain(out_l, 0, 255);
          out_r = constrain(out_r, 0, 255);

          // 4. Send signals to hardware
          analogWrite(L_MOTOR, out_l);
          analogWrite(R_MOTOR, out_r);
          
          motors_active = true;
          premillis_rx = millis(); // Reset the data failsafe timer

          // DEBUG: Print the mixing results
          Serial.print("APP -> Thr: "); Serial.print(parsed_throttle);
          Serial.print(" | Yaw: "); Serial.print(parsed_yaw);
          Serial.print("  ||  MOTORS -> L: "); Serial.print(out_l);
          Serial.print(" | R: "); Serial.println(out_r);
        }
      }
    }
    
    // ==========================================
    // LAYER 2: APP DISCONNECT / TIMEOUT FAILSAFE
    // ==========================================
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
       long rssi = abs(WiFi.RSSI());
       float vcc = (((float)ESP.getVcc()/(float)1024.0)+0.75f)*10;
       replyBuffer[1] = (unsigned char)rssi;
       replyBuffer[2] = (unsigned char)vcc;
       
       Udp.beginPacket(remotIp, remotPort);
       Udp.write(replyBuffer);
       Udp.endPacket();
     }
  }
  else
  {
    // ==========================================
    // OUT OF RANGE BEHAVIOR
    // ==========================================
    if (wifi_was_connected || motors_active) {
      analogWrite(L_MOTOR, 0);
      analogWrite(R_MOTOR, 0);
      motors_active = false;
      wifi_was_connected = false;
      Serial.println("!!! CRITICAL FAILSAFE: Wi-Fi Connection Lost! Motors STOPPED. !!!");
    }
    
    // Blink LED to indicate we are trying to reconnect
    digitalWrite(ST_LED, LOW);
    delay(60);
    digitalWrite(ST_LED, HIGH);
    delay(1000);
  }
}
