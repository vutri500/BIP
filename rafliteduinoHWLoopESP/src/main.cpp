/* Copyright (c) 2021  Paulo Costa
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#include <Arduino.h>
#include "channels.h"
#include "IRLine.h"
#include "robot.h"
#include "PID.h"
#include "proj_types.h"
#include "SPI.h"
#include "ArduinoNvs.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

byte UsingSimulator, testIR;
byte go;

String ssid, password;

int ssid_idx, pass_idx;
int ssid_done, pass_done;

//IPAddress SendIP(192, 168, 1, 172);
int udp_on, ip_on;

WiFiUDP Udp;
unsigned int localUdpPort = 4224;  // local port to listen on

#define UDP_MAX_SIZE 512
uint8_t UdpInPacket[UDP_MAX_SIZE];  // buffer for incoming packets
uint8_t UdpOutPacket[UDP_MAX_SIZE];  // buffer for outgoing packets
int UdpBufferSize = UDP_MAX_SIZE;

//#include <Wire.h>
//#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
//static Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

//Adafruit_DCMotor* M1;
//Adafruit_DCMotor* M2;
//Adafruit_DCMotor* M3;

//#include <VL53L0X.h>

//VL53L0X sensor;
//float distance, prev_distance;

void sim_loop(void);
void real_loop(void);

#define NUM_PARS 16

float Pars[NUM_PARS + 1];
uint8_t send_par_index, recv_par_index;

void process_serial_packet(char channel, uint32_t value, channels_t& obj);


channels_t serial_channels, udp_channels;

IRLine_t IRLine;

hw_timer_t * timer_enc = NULL;


// Encoders: supply GND and 3V3
#define ENC1_A 27  //green  Right motor encoder
#define ENC1_B 14  //blue 
#define ENC2_A 19  // green  Left motor encoder
#define ENC2_B 23  // blue

// MOTOR DRIVER: supply to GND and Vin
#define ML_A 26   // A-1A  Left motor +
#define ML_B 25   // A-1B  Left Motor -
#define MR_A 16   // B-1B  Right Motor -
#define MR_B 17   // B-1A  Right Motor +

#define SOLENOID_PIN 13  // connect to Magnet board (supply the board with GND and +5V)
#define TOUCHSW_pin 12  // switch it to ground

//IR line sensor: Connect to GND and 3V3
//Blue   IR1 IO33
//Yellow IR2 IO32
//Orange IR3 IO39
//Green  IR4 IO36
//White  IR5 IO34



const int freq = 500;
const int ML1Channel = 0;
const int ML2Channel = 1;
const int MR1Channel = 2;
const int MR2Channel = 3;
const int resolution = 8;

robot_t robot;



void setMotorLPWM(int PWM);
void setMotorRPWM(int PWM);




void setSolenoid(byte value);



void setSolenoidState()
{
  if (robot.solenoid_state) setSolenoid(1);
  else setSolenoid(0);
}

byte readTouchSwitch(void)
{
  return !digitalRead(TOUCHSW_pin);
}

void control(robot_t& robot);

#include <ESP32Encoder.h>

ESP32Encoder encoder1, encoder2;

//uint32_t delta, current, previous, interval = 40000UL;
schedule_t schedule;

void setMotorLPWM(int PWM)
{
  int PWM_max = 100;
  if (PWM >  PWM_max) PWM =  PWM_max;
  if (PWM < -PWM_max) PWM = -PWM_max;
  if (PWM >= 0)
  {
     ledcWrite(ML1Channel,PWM);
     ledcWrite(ML2Channel,0);     
  }
  if (PWM < 0)
  {
     ledcWrite(ML1Channel,0);
     ledcWrite(ML2Channel,-PWM);     

  }
}

void setMotorRPWM(int PWM)
{
  int PWM_max = 100;
  if (PWM >  PWM_max) PWM =  PWM_max;
  if (PWM < -PWM_max) PWM = -PWM_max;
   if (PWM >= 0)
  {
     ledcWrite(MR1Channel,PWM);
     ledcWrite(MR2Channel,0);     
  }
  if (PWM < 0)
  {
     ledcWrite(MR1Channel,0);
     ledcWrite(MR2Channel,-PWM);     
  }
}



void setMotorsPWM(int PWM1, int PWM2)
{
  setMotorLPWM(PWM2);
  setMotorRPWM(PWM1);
}


void setSolenoid(byte value)
{
  digitalWrite(SOLENOID_PIN, value);
}


void serial_write(uint8_t b)
{
  Serial.write(b);
}


#define TX_UDP_BUF_SIZE 256
uint8_t TX_buffer[TX_UDP_BUF_SIZE];
int TX_count;

void serial_send_buffer(void)
{
  if (TX_count != 0) Serial.write(TX_buffer, TX_count);
}

void udp_send_buffer(void)
{
  //Serial.write(TX_buffer, TX_count);
  if (TX_count != 0 && udp_on) {
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(TX_buffer, TX_count);
    //Serial.print("Sent="); Serial.println(Udp.endPacket());
    Udp.endPacket();  
  } 
  TX_count = 0;
  Serial.write('.');
}


void udp_write(uint8_t b)
{
  //Serial.write(b);

  if (TX_count < TX_UDP_BUF_SIZE) {
    TX_buffer[TX_count] = b;
    TX_count++;
  }
  if (TX_count == TX_UDP_BUF_SIZE) {
    udp_send_buffer();
  }
}



void wifi_init(void)
{
  if (!(ssid_done && pass_done)) return;
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.print("Connecting to WiFi ..");
  //while (WiFi.status() != WL_CONNECTED) {
  //  Serial.print('.');
  //  delay(1000);
  //}
  Serial.println(WiFi.localIP());
}


void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  ip_on = Udp.begin(localUdpPort);
  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);
  
  ArduinoOTA.begin(); 
}

void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Reason: ");
  Serial.println(info.prov_fail_reason);
  udp_on = 0;
  ip_on = 0;
  wifi_init();
  //Serial.println("Trying to Reconnect");
  //WiFi.begin(ssid, password);
}


void fill_sane_pars(void)
{
  int i;
  for (i = 0; i < NUM_PARS; i++) {
    Pars[i] = 0;  
  } 

  Pars[0] = 10.5;  // Kp
  Pars[1] = 22;    // Ki
  Pars[2] = 2;     // Kd
  Pars[3] = 9.85;  // Kf

}

 #define RXD2 13
 #define TXD2 12
 #define IR_CARRIER 5
 
void setup()
{
  // Set the pins as input or output as needed
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  pinMode(TOUCHSW_pin, INPUT_PULLUP);

  //pinMode(IR_CARRIER, OUTPUT);
  pinMode(ML_A, OUTPUT);
  pinMode(MR_A, OUTPUT);
  pinMode(ML_B, OUTPUT);
  pinMode(MR_B, OUTPUT);
  pinMode(SOLENOID_PIN, OUTPUT);

  ledcSetup(ML1Channel, freq, resolution);
  ledcSetup(MR1Channel, freq, resolution);
  ledcSetup(ML2Channel, freq, resolution);
  ledcSetup(MR2Channel, freq, resolution);
  ledcAttachPin(ML_A, ML1Channel);
  ledcAttachPin(ML_B, ML2Channel);
  ledcAttachPin(MR_A, MR1Channel);
  ledcAttachPin(MR_B, MR2Channel);  
  // attach the channel to the GPIO to be controlled
 

 

  UsingSimulator = 1;
  //testIR = 0;

  schedule.interval = 40000UL;
  robot.dt = 1e-6 * schedule.interval; // in seconds

  fill_sane_pars();

  // Reserve SSID and password buffers
  //ssid.reserve(32);
  //password.reserve(32);
  ssid_idx = 0;
  pass_idx = 0;
  ssid_done = 0;
  pass_done = 0;

  NVS.begin();

  NVS.setString("SSID", "BIPRAFL");
  NVS.setString("PASS", "BIPRAFL23");
  
  ssid = NVS.getString("SSID");
  if (ssid.length() > 0) ssid_done = 1;

  password = NVS.getString("PASS");
  if (password.length() > 0) pass_done = 1;

  wifi_init();

  Serial.begin(115200);
  serial_channels.init(process_serial_packet, serial_write);
  Serial.println("Init Begin!");
  
  //Serial2.begin(2400, SERIAL_8N1, RXD2, TXD2);

  if (!UsingSimulator) {
  

//#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
//  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
//  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
//  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
//#endif

//#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
//  sensor.setMeasurementTimingBudget(20000);
//#endif    
  }

  // Enable the weak pull up resistors
	ESP32Encoder::useInternalWeakPullResistors=UP;

  // use pin ENC1_A and ENC1_B for the first encoder
  encoder1.attachFullQuad(ENC1_A, ENC1_B);
  
  // use pin ENC2_A and ENC2_B for the second encoder (reverse order to correct signal)
  encoder2.attachFullQuad(ENC2_B, ENC2_A);
      
  // clear the encoder's raw count and set the tracked count to zero
  encoder1.clearCount();
  encoder2.clearCount();  

  // AD configuration
  analogReadResolution(10);
  analogSetAttenuation(ADC_0db);

  udp_channels.init(process_serial_packet, udp_write);

  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  //if (WiFi.status() == WL_CONNECTED) {
  //  Udp.begin(localUdpPort);
  //}
  //Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
      //ESP.restart();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  //ArduinoOTA.begin();  

  // Robot Parameters
  //robot.b = 0.137 / 2;
  robot.r1 = 0.07 / 2;
  robot.r2 = 0.07 / 2;

  robot.dv_max = 5 * robot.dt;  // Linear velocity change per control period
  robot.dw_max = 10 * robot.dt; // Angular velocity change per control period

  robot.state = 0;
  robot.solenoid_state = 1;
  Serial.println("\nSetup Finished!");
}


void readEncoders(void)
{
  cli();  // Must be done with the interrupts disabled: minimize lost count betwween read and clear
  robot.enc1 = encoder1.getCount();
  encoder1.clearCount();

  robot.enc2 = encoder2.getCount();
  encoder2.clearCount();
  sei();

  robot.Senc1 += robot.enc1;
  robot.Senc2 += robot.enc2;
}

int analog_pins[5] = {33, 32, 39, 36, 34};



void readIRSensors(void)
{
  byte c;  // Read the five IR sensors using the AD converter
  for (c = 0; c < 5; c++) {
    IRLine.IR_values[c] = 1023 - analogRead(analog_pins[c]);
  }
}

uint32_t encodeIRSensors(void)
{
  byte c;  // Encode five IR sensors with 6 bits for each sensor
  uint32_t result = IRLine.IR_values[0] >> 4; // From 10 bits to 6 bits
  for (c = 1; c < 5; c++) {
    result = (result << 6) | (IRLine.IR_values[c] >> 4);
  }
  return result;
}



void process_serial_packet(char channel, uint32_t value, channels_t& obj)
{
  channels_u val;
  val.u = value;

  if (channel == 'o') {  // Clear acumulated encoder
    robot.Senc1 = 0;
    robot.Senc2 = 0;

  } else if (channel == 'J')  {  // Start Pars
    if ((value & 0xFF) < NUM_PARS) {
      recv_par_index = value & 0xFF;
    }      

  } else if (channel == 'H')  {  // Each Par
    if (recv_par_index >= 0) {
      Pars[recv_par_index] = val.f;
      recv_par_index++;
      if (recv_par_index >= NUM_PARS) recv_par_index = 0;
    }      
  
  } else if (channel == 'R')  { 
    robot.enc1 = int16_t((value >> 16) & 0xFFFF);
    robot.enc2 = int16_t((value >> 00) & 0xFFFF);

    robot.Senc1 += robot.enc1;
    robot.Senc2 += robot.enc2;

  } else if (channel == 'I')  {   // IR Sensors + Touch
    uint8_t c;
    for (c = 0; c < 5; c++) {
      IRLine.IR_values[c] = 16 * ((value >> (c * 6)) & 0x3F);
    } 
    robot.TouchSwitch = ((value >> 31) & 1);  
 
  } else if (channel == 'G')  {  // Control
    go = 1;

  } else if (channel == 'v') {  // Set robot linear speed reference
    robot.v_req = val.f;   

  } else if (channel == 'w') {  // Set robot angular speed reference
    robot.w_req = val.f;   

  } else if (channel == 'x') {  // Set robot x position
    robot.x = val.f;   

  } else if (channel == 'y') {  // Set robot y position
    robot.y = val.f;   

  } else if (channel == 't') {  // Set robot theta angle
    robot.theta = val.f;   

  } else if (channel == 's') {  // Set robot state
    robot.setState(value);

  } else if (channel == 'O') {  // Set Requested PWM
    robot.PWM_1_req = int16_t((value >> 16) & 0xFFFF);
    robot.PWM_2_req = int16_t((value >> 0) & 0xFFFF);

  } else if (channel == 'T') {  // Set general parameters T1 and T2
    robot.T1 = (value >> 16) & 0xFFFF;
    robot.T2 = (value >> 0) & 0xFFFF;

  } else if (channel == 'p') { // Set PID parameter
    Pars[0] = val.f; 
    robot.PID1.Kp = val.f;
    robot.PID2.Kp = val.f;

  } else if (channel == 'i') { // Set PID parameter
    if (fabs(val.f) > 1e-2 ) {
      robot.PID1.Se = robot.PID1.Se * robot.PID1.Ki / val.f;
      robot.PID2.Se = robot.PID2.Se * robot.PID2.Ki / val.f;
    }
    Pars[1] = val.f; 
    robot.PID1.Ki = val.f;
    robot.PID2.Ki = val.f;

  } else if (channel == 'm') { // Set PID parameter
    Pars[2] = val.f; 
    robot.PID1.Kd = val.f;
    robot.PID2.Kd = val.f;

  } else if (channel == 'n') { // Set PID parameter
    Pars[3] = val.f; 
    robot.PID1.Kf = val.f;
    robot.PID2.Kf = val.f;

  } else if (channel == 'z') { // Set Solenoid
    robot.solenoid_state = value;

  }
}


void testIR_loop(void)
{
  uint32_t t;
  byte b;
  
  if (Serial.available()) {
    b = Serial.read();
    Serial2.write(b);
  }  

  if (Serial2.available()) {
    b = Serial2.read();
    Serial.write(b);
  }  
}

void loop(void)
{
  if (UsingSimulator) {
    sim_loop();
  } else if (testIR) {
    testIR_loop();
  } else {
    real_loop();
  }

  if (ip_on) {
    ArduinoOTA.handle();

    int packetSize = Udp.parsePacket();
    if (packetSize) {
      int i;
      udp_on = 1;
      // receive incoming UDP packets

      //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
      int len = Udp.read(UdpInPacket, UdpBufferSize - 1);
      if (len > 0) {
        UdpInPacket[len] = 0;
      }
      //Serial.printf("UDP packet contents (as string): %s\n", UdpInPacket);

      for (i = 0; i < len; i++) {
        udp_channels.StateMachine(UdpInPacket[i]);
        //Serial.write(UdpInPacket[i]);
      }
    }      
  
  }
}


void serial_print_format(int value, byte space)
{
  byte b, c;
  b = Serial.print(value);
  for (c = 0; c < space - b; c++) {
     Serial.print(" ");
  }
}

void serial_print_format(float value, byte space)
{
  Serial.printf("%4g", value);
}

void send_udp_channels(void)
{
  udp_channels.send('I', encodeIRSensors() | (robot.solenoid_state << 30) | (robot.TouchSwitch << 31));
  udp_channels.send('U', robot.PWM_1, robot.PWM_2);
  udp_channels.send('R', robot.enc1, robot.enc2);
  udp_channels.send('S', robot.Senc1, robot.Senc2);
  udp_channels.send('T', robot.T1, robot.T2);
  
  udp_channels.sendFloat('v', robot.ve);
  udp_channels.sendFloat('w', robot.we);
  
  udp_channels.sendFloat('x', robot.x);
  udp_channels.sendFloat('y', robot.y);
  //udp_channels.sendFloat('x', IRLine.pos_right);
  //udp_channels.sendFloat('y', IRLine.pos_left);
  udp_channels.sendFloat('t', robot.theta);

  //udp_channels.sendFloat('p', robot.PID1.Kp);
  //udp_channels.sendFloat('i', robot.PID1.Ki);
  //udp_channels.sendFloat('m', robot.PID1.Kd);
  //udp_channels.sendFloat('n', robot.PID1.Kf);

  udp_channels.send('l', byte(IRLine.total / 20), IRLine.cross_count, IRLine.crosses, byte(IRLine.blacks * 10));

  // send Pars Table sequentially (num_pars_to_send for each loop)
  int num_pars_to_send = 4;
  udp_channels.send('J', num_pars_to_send, send_par_index);
  for (int i = 0; i < num_pars_to_send; i++) {
    udp_channels.sendFloat('H', Pars[send_par_index]);
    send_par_index++;
    if (send_par_index >= NUM_PARS) send_par_index = 0;
  }

  udp_channels.send('P', robot.state, schedule.delta / 100);
  udp_send_buffer();
}


void real_loop(void)
{
  uint32_t t;
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    if (b == '+') robot.solenoid_state = 1;
    if (b == '-') robot.solenoid_state = 0;
    //if (b == '(') {robot.v1_PWM = 50;}
    if (b == '(') {robot.v += 0.1; robot.w = 0; robot.PWM_1_req += 10; robot.PWM_2_req += 10;}
    if (b == '/') {robot.v = 0; robot.w = 1.5; robot.PWM_1_req += 10; robot.PWM_2_req = 0;}
    if (b == '=') {robot.v = 0; robot.w =-1.5; robot.PWM_1_req = 0; robot.PWM_2_req += 10;}
    //if (b == ')') {robot.v2_PWM = 50;}
    if (b == ')') {robot.v -= -0.1; robot.w = 0;  robot.w = 0; robot.PWM_1_req -= 10; robot.PWM_2_req -= 10;}
    //if (b == '?') {robot.v1_PWM = 0; robot.v2_PWM = 0;} 
    if (b == '?') {robot.v = 0; robot.w = 0; robot.PWM_1_req = 0; robot.PWM_2_req = 0;}
    if (b == '\\') robot.state = 200;
    if (b == '*') robot.state = 201;
    serial_channels.StateMachine(b);
  }


  schedule.current = micros();
  schedule.delta = schedule.current - schedule.previous;

  if (schedule.delta >= schedule.interval) {
    schedule.previous = schedule.current;

    readEncoders();

    t = micros();
    readIRSensors();
    t = micros() - t;

    // Start new distance measure
    //sensor.startReadRangeMillimeters();

    robot.LastTouchSwitch = robot.TouchSwitch;
    robot.TouchSwitch = readTouchSwitch();

    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();

    robot.odometry();
    control(robot);

    // Later: read the real battery voltage
    robot.battery_voltage = 7.4;

    setSolenoidState();

    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;

    // Auto Control mode selection:
    // States for 0 to 199 are for PID control
    // States for 200 to 255 are for direct Voltage control
    robot.VWToMotorsVoltage();

    setMotorsPWM(robot.PWM_1, robot.PWM_2);
    //setMotorsPWM(50, 150);

    IPAddress ip = WiFi.localIP();

    serial_channels.send('i', ip[0], ip[1], ip [2], ip[3]);
    
    if(udp_on) send_udp_channels();
    
    Serial.print(F(" "));
    Serial.print(ip.toString());

    Serial.print(F(" E1: "));
    serial_print_format(robot.enc1, 4);

    Serial.print(F(" E2: "));
    serial_print_format(robot.enc2, 4);

    byte c;
    for (c = 0; c < 5; c++) {
       Serial.print(" ");
       Serial.print(IRLine.IR_values[c]);
    }

    Serial.print(" S: ");
    serial_print_format(robot.state, 0);    

    Serial.print(" T: ");
    serial_print_format(robot.TouchSwitch, 0);
     
    Serial.print(" M: ");
    serial_print_format(robot.solenoid_state, 0);    

    Serial.print(" PWM1: ");
    serial_print_format(robot.PWM_1, 0);    
    Serial.print(" PWM2: ");
    serial_print_format(robot.PWM_2, 0);    


    /*
    Serial.print(F(" V: "));
    serial_print_format(robot.v, 4);
    Serial.print(F(" W: "));
    serial_print_format(robot.w, 4);

    Serial.print(F(" Ve: "));
    serial_print_format(robot.ve, 4);
    Serial.print(F(" We: "));
    serial_print_format(robot.we, 4);    
    */

    Serial.println();
  }

}


void sim_loop(void)
{
  byte b;
  if (Serial.available()) {
    b = Serial.read();
    serial_channels.StateMachine(b);
    if (b == '?')
      Serial.print(WiFi.localIP().toString());   
  }

  if (go) {
    schedule.previous = schedule.current;
    schedule.current = micros();
    schedule.delta = schedule.current - schedule.previous;
    go = 0;

    IRLine.calcIRLineEdgeLeft();
    IRLine.calcIRLineEdgeRight();
    IRLine.calcCrosses();

    robot.odometry();
    control(robot);
    
    //robot.accelerationLimit();
    robot.v = robot.v_req;
    robot.w = robot.w_req;

    robot.VWToMotorsVoltage();
    
    robot.battery_voltage = 7.4;
    robot.PWM_1 = robot.u1 / robot.battery_voltage * 255;
    robot.PWM_2 = robot.u2 / robot.battery_voltage * 255;
    
    //Serial.print(WiFi.localIP().toString());   
    //Serial.print(F(" "));
 
    serial_channels.send('s',  robot.state);
    serial_channels.send('M',  round(robot.PWM_1));
    serial_channels.send('Q',  round(robot.PWM_2));
    serial_channels.send('L',  robot.solenoid_state);
    serial_channels.send('X',  IRLine.crosses);
    //serial_channels.send('Y',  IRLine.pos_left);
    //serial_channels.send('Z',  IRLine.pos_right);

    if(udp_on) send_udp_channels();

  }
  
  /*schedule.current = micros();
  schedule.delta = schedule.current - schedule.previous;

  if (schedule.delta >= schedule.interval) {
    schedule.previous = schedule.current;

    if(udp_on) send_udp_channels();
  }*/
}


