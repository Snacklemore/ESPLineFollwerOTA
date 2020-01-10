#include <WiFiLink.h>
#include <WiFiUdp.h>
#include "SoftwareSerial.h"
//**************************************************************************
//SENSORPINSETUP
//**************************************************************************
//GABELLICHTSCHRANKE ZU LANGSAM! JE HÖHER DIE DREHZAHL DESTO WENIGER SEGMENTE WERDEN GEZÄHLT WAS DAZU FÜHRT DAS DIE GESCHWINDIGKEIT IMMER WEITER STEIGT
//LICHTSCHRANKE LINKS FUNKTIONIERT EINWANDFREI, LICHTSCHRANKE RECHTS IST NICHT BAUGLEICH DA ERSATZTEIL!

int ruptPinLeft  = A4;
int ruptPinRight = A5;
int ruptPinValLeft = 0;
int ruptPinValRight = 0;
int qtrSensorLeft = A3;
int qtrSensorRight = A2;
int qtrSensorValLeft = 0;
int qtrSensorValRight = 0;

//**************************************************************************
//MOTORPINSETUP
//**************************************************************************

//**************************************************
//1st motor
//**************************************************
int enA = 11;
int in1 = 8;
int in2 = 9;

//****************************************************
//2nd motor
//**************************************************
int enB = 10;
int in3 = 7;
int in4 = 6;

//**************************************************************************
//**************************************************************************
//TIMER
int curStateLeft = 0; //states for RPM Control
int prevStateLeft = 0;
int curStateRight =0;
int prevStateRight = 0;
const int computeWheelInterval = 1000; //timings for RPM Control
const int computeWheelIntervalMedian = 100;
const int updateWheelPWMInterval = 1000;
unsigned long lastWheelIntervalMedianTime =0;
unsigned long lastUpdateWheelPWMRefreshTime = 0;
unsigned long lastWheelRefreshTime = 0;
bool speedIsControlled = false; //flag whether to control RPM or not
bool isStopped = true; //flag to stop execution of automation loop
int lastAction = 0; // 0 start 1 links 2 rechts, lastAction performed eg. turn right etc.
//**************************************************************************
//**************************************************************************


SoftwareSerial Serialwifi(2, 3);// RX, TX, Emulated hardwareSerial port, for communication with ESP8266

unsigned int currentPWMA = 47; // PWM Values for motors
unsigned int currentPWMB = 50;

unsigned int localPort = 2390;      // local port to listen on to receive commands over WiFi

char packetBuffer[255]; //buffer to hold incoming packet
char ReplyBuffer[] = "acknowledged";
float velocity = 1.0; // velocity the RPM Control aims to maintain
float SegmentCounterRight = 0;// Counter variables to count segments of decoder per second
float SegmentCounterLeft = 0;



float vRight = 0;// actual velocity calculated per motor
float vLeft = 0;

WiFiUDP Udp; //WiFi UDP object, needed to run an UDPServer
unsigned int state = 0;// state to control what is executed
char* charBuffer;
void ledDebug(int time){ // debugFunction 
    digitalWrite(LED_BUILTIN, HIGH);   
    delay(time);                       
    digitalWrite(LED_BUILTIN, LOW);    
     
  
}
//**************************************************************************
//**************************************************************************

void resetEsp()
{
  WiFi.resetESP(); //ESP Reset to maintain connectivity 
}
void driveForward(int pwmSpeedA,int pwmSpeedB)
{
  speedIsControlled = true;
  //Motor A forward
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
  analogWrite(enA,pwmSpeedA);
  analogWrite(enB,pwmSpeedB);

  //Motor B forward
  
  
}
void setMotorRightForward()
{
  digitalWrite(in3,HIGH);
  digitalWrite(in4,LOW);
}
void setMotorLeftForward()
{
  digitalWrite(in1,HIGH);
  digitalWrite(in2,LOW);
}
void setMotorRightBackwards()
{
  digitalWrite(in3,LOW);
  digitalWrite(in4,HIGH);
}
void setMotorLeftBackwards()
{
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
}


void stopLeftMotor()
{
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
}
void stopRightMotor()
{
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
}
void stopMotors()
{
  digitalWrite(in1,LOW);
  digitalWrite(in2,LOW);
  digitalWrite(in3,LOW);
  digitalWrite(in4,LOW);
  speedIsControlled = false;
  
  
}
//**************************************************************************
//**************************************************************************

int readLeftWheel(){ // function to read photointerrupter optisolator(values are analog, we need them digital to count the state changes)
  
  if (analogRead(ruptPinLeft) > 0)
  {
    return 0;
  }
  if (analogRead(ruptPinLeft) == 0)
  {
    return 1;
  }
  
}
int readRightWheel(){
  
  if (analogRead(ruptPinRight) > 0)
  {
    return 0;
  }
  if (analogRead(ruptPinRight) == 0)
  {
    return 1;
  }
  
}
int readLeftSensor(){ //read IR sensor Values
  return analogRead(qtrSensorLeft);
  
}
int readRightSensor(){
  return analogRead(qtrSensorRight); 

  
}
//**************************************************************************
//**************************************************************************

void detectSegments(){ //function to count segments
      
      curStateRight = analogRead(ruptPinRight);     
      if (curStateRight == 0)
      {
   
        SegmentCounterRight++;
        
      }else if (curStateRight > 0)
      {
        
      }
      
      curStateLeft  = readLeftWheel();
      if (curStateLeft != prevStateLeft)
      {
   
        SegmentCounterLeft++;
        prevStateLeft = curStateLeft;
      }
      
     
}
//**************************************************************************
//**************************************************************************
//called every second
void computeWheelSpeed() //compute wheelspeed every 1s out of counted segments of decoder
{
  
  if (millis() - lastWheelRefreshTime >= computeWheelInterval)
  {
    detectSegments();
    
    float radius = 0.16;
    vLeft = (SegmentCounterLeft * (2.0*3.14159*radius))/40.0; //ms/s
    vRight = (SegmentCounterRight * (2.0*3.14159*radius))/40.0;
    charBroadCast(string2char(String(vRight)));



    SegmentCounterLeft = 0;
    SegmentCounterRight = 0;
    
    controlSpeed();
   
    lastWheelRefreshTime += computeWheelInterval;
    
  }
  
  
}
//**************************************************************************
//**************************************************************************
void slowDownVLeft() // functions to change PWM so aimed speed gets maintained
{
  currentPWMA -=1;
  if (currentPWMA < 40) 
    currentPWMA = 41;
  analogWrite(enA,currentPWMA);
}
void slowDownVRight()
{
  currentPWMB -=1;
  if (currentPWMB < 40) 
    currentPWMB = 41;
  analogWrite(enB,currentPWMB);
}
void speedUpVLeft()
{
  currentPWMA +=1;
  
  analogWrite(enA,currentPWMA);
}
void speedUpVRight()
{
  currentPWMB +=1;
  
  analogWrite(enB,currentPWMB);
}
//**************************************************************************
//**************************************************************************
//called every 1s
void controlSpeed(){ //function to get speed on target
  detectSegments();
  if ( speedIsControlled)
  {
     
        detectSegments();  
        if (vRight > velocity)
          {
            slowDownVRight();
          }
          else if (vRight < velocity)
          {
            speedUpVRight();
          }
          if(vLeft > velocity)
          {
          slowDownVLeft();
          }
          else if (vLeft < velocity)
          {
          speedUpVLeft();
          }
  }
  detectSegments();
  
  
  
}
//**************************************************************************
//**************************************************************************
//provides qtrSensorValLeft and Right with values 
void checkLineSensors()
{
  qtrSensorValLeft = readLeftSensor();
  qtrSensorValRight = readRightSensor();
}
//**************************************************************************
//**************************************************************************
//adjusting motorspeed according to sensor readings
void adjustMotors()
{
  if (!isStopped)
  {
    if (qtrSensorValLeft > 100)
    {
      
      stopLeftMotor();
      
      setMotorRightForward();
      lastAction = 2;
    }
    if (qtrSensorValRight > 100)
    {
      
      stopRightMotor();
      
      setMotorLeftForward();
      lastAction = 1;
    }
    if ( qtrSensorValLeft > 100 && qtrSensorValRight > 100)
    {
      if (lastAction == 1)
      {
        
        setMotorLeftForward();
      }
      else if ( lastAction == 2)
      {
        
        setMotorRightForward();
      }
    }
  }
  
}
//**************************************************************************
//**************************************************************************

//actual Automation loop 
//loop is elevated from main loop to keep execution time of one loop iteration as low as possible
void startAutomation()
{
  bool isRunning = true;
  while(isRunning)
  {
    
    
    checkLineSensors();
    adjustMotors();
    int packetSize = Udp.parsePacket(); // reading incoming packets
    if (packetSize)
    {
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) packetBuffer[len] = 0;
      str1 = String(packetBuffer);
      if (str1 == String("mstop")) // stop automation loop, and fall back to main loop
      {
        ledDebug(100);
        isStopped = true;  
        qtrSensorValLeft = 0;
        qtrSensorValRight = 0;
        stopMotors();
        isRunning = false;
      }
      if(str1 == String("start"))
      {
        isStopped = false;
        driveForward(currentPWMA,currentPWMB);
      }
      if (str1 == String("stop"))
      {
        isStopped = true; // stop Robot but stay in automation 
        qtrSensorValLeft = 0;
        qtrSensorValRight = 0;
        stopMotors();
      }
     
    }
    
    
  }
}
//**************************************************************************
//**************************************************************************

void setup() {
  //MOTORPINPINSETUP
  pinMode(enA,OUTPUT);
  pinMode(enB,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  pinMode(in3,OUTPUT);
  pinMode(in4,OUTPUT);
  pinMode(ruptPinRight,INPUT);
  pinMode(ruptPinLeft,INPUT);
  pinMode(qtrSensorLeft,INPUT);
  pinMode(qtrSensorRight,INPUT);
  
  
  

  
  pinMode(LED_BUILTIN, OUTPUT);
  
  //delay to give ESP8266 time to connect to smartPhone HotSpot
  ledDebug(2000);
  Serialwifi.begin(9600); // speed must match with BAUDRATE_COMMUNICATION setting in firmware config.h
  WiFi.init(&Serialwifi);
    
  WiFi.resetESP(); // to clear 'sockets' after sketch upload
  
  delay(3000); //wait while WiFiLink firmware connects to WiFi with Web Panel settings
  while (WiFi.status() != WL_CONNECTED) {
    delay(10);
    ledDebug(40);

  }
  ledDebug(2500); // Almost Ready signal
  printWifiStatus();

  
  Udp.begin(localPort); // starting the UDPServer to Receive Commands
   
}

void loop() {
 
  
  
  
  
  
  
  //check states first to determine what to do
  switch(state){
    case 0:
        {
          
          break;
        }
          
    case 1:
        {
          ledDebug(1000);
          resetEsp();
          break;
        }
        
    case 2:
    {
          ledDebug(500); // TestCase no real usage
           break; 
    }
    case 3:
    {
          startAutomation(); // entry point of Automation Loop
          state = 0;
          break;
    }
    case 4:
    {
      break;
      
    }
    case 5:
    {
      stopMotors();
      state = 0;
      break;
    }
          
  }
 
 
 
  
  
 int packetSize = Udp.parsePacket(); // read incoming UDP Packets
  if (packetSize)
  {
    ledDebug(500);
    
    IPAddress remoteIp = Udp.remoteIP(); // get the Commandsenders IP
   

    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) packetBuffer[len] = 0;
    //SerialH.println(packetBuffer);
    str1 = String(packetBuffer);
    if (str1 == String("rst")){ // read send string and determine what to do 
      state = 1;
    } else if (str1 == String("stop"))
    {
      state = 0;
    } else if (str1 == String("fastblink"))
    {
      state = 2;
    } else if (str1 == String("automate"))
    {
      state = 3;
    } else if (str1 == String("ipprint"))
    {
      state = 4;
    }else if (str1 == String("mstop"))
    {
      state = 5;
    }
    str1 = "";

    // send a reply, to the IP address and port that sent us the packet we received
    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(ReplyBuffer);
    Udp.endPacket();
    
  }
  
}
void charBroadCast(const char* arr) // broadcast to send sensor values 
{
   IPAddress broadcastIp(255,255,255,255);
   Udp.beginPacket(broadcastIp, 2391);
    Udp.write(arr); //SerialH.print(remoteIp);
    //SerialH.print(", port ");
    //SerialH.println(Udp.remotePort());
    Udp.endPacket();
  //free(arr);
}
char* string2char(String command){ // helper function, only c_strings are broadcastable
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}

void printWifiStatus() {
 IPAddress ip = WiFi.localIP();
 long rssi = WiFi.RSSI();
  
}

/*void computeRpmMedian()
{
//called every 1/2s
//failed RPMMedianCalculation not used
  if ( millis()- lastRPMMedianRefreshTime >= computeRPMMedianInterval)
  {
    int segmentCountSumLeft = 0;
    int segmentCountSumRight = 0;
    int i;
    int lossR =0;
    for(i =0;i<5;i++)
    {
      int segmentCount = SegmentCounterLeftArr[i];
      
      if (segmentCount > 0)
      {
        segmentCountSumLeft += segmentCount;
      }else
      {
        lossR++;
      }
        
      
    }
    int lossL = 0;
    int k;
    for(k=0;k<5;k++)
    {
      int segmentCount = SegmentCounterLeftArr[k];
      
      if (segmentCount > 0){
        segmentCountSumRight += segmentCount;
      }
      else
      {
        lossL++;
      }
        
      
    }
    SegmentCounterLeftMedian  = segmentCountSumLeft  / (5-lossL);
    SegmentCounterRightMedian = segmentCountSumRight / (5-lossR);
    charBroadCast(string2char(String(SegmentCounterRightMedian)));
    

    lastRPMMedianRefreshTime += computeRPMMedianInterval;
  }
}*/
