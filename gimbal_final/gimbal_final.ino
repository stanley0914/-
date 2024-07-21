#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>
#include <Servo.h>

//定義MPU用到的變數
bool dmpready=false;//DMP初始化成功->true
uint8_t mpuintstatus;
uint8_t devstatus;
uint16_t packetsize;
uint16_t fifocount;
uint8_t fifobuffer[64];
int16_t rawvalue[6];
int yawvalue,pitchvalue,rollvalue;

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuinterrupt=false;
void dmpdataready(){
  mpuinterrupt=true;
}
#define INTERRUPT_PIN 2
MPU6050 accelgyro;

Servo syaw;
Servo spitch;
Servo sroll;
void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(38400);

  Serial.println(F("Initializing I2C devices"));
  accelgyro.initialize();
  pinMode(INTERRUPT_PIN,INPUT);

  Serial.println(F("Initializing DMP"));
  unsigned long startTime=millis();
  devstatus=accelgyro.dmpInitialize();

  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(2048);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  Serial.print(F("wait"));

  if(devstatus==0){
    accelgyro.CalibrateAccel(6);
    accelgyro.CalibrateGyro(6);
    accelgyro.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    accelgyro.setDMPEnabled(true);

    Serial.print(F("enabling inerrupt detection(arduino external interrupt)"));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN),dmpdataready,RISING );
    mpuintstatus =accelgyro.getIntStatus();

    Serial.println(F("dmp ready. Wating for first interrupt..."));
    dmpready=true;
    
    packetsize=accelgyro.dmpGetFIFOPacketSize();
    unsigned long endTime=millis();
    Serial.print(F("DMP initialization time:"));
    Serial.print(endTime-startTime);
  }
  else{
    Serial.print(F("DMP initialization failed(code"));
    Serial.print(devstatus);
    Serial.println(F(")"));
    
  }

  syaw.attach(10);
  spitch.attach(9);
  sroll.attach(8);
}
void loop() {
  // put your main code here, to run repeatedly:
  if(!dmpready){
    return;
  }
  if(accelgyro.dmpGetCurrentFIFOPacket(fifobuffer)){
    accelgyro.dmpGetQuaternion(&q,fifobuffer);
    accelgyro.dmpGetGravity(&gravity,&q);
    
    accelgyro.dmpGetYawPitchRoll(ypr,&q,&gravity);
    accelgyro.getMotion6(&rawvalue[0],&rawvalue[1],&rawvalue[2],&rawvalue[3],&rawvalue[4],&rawvalue[5]);

    Serial.println("\n感測器校正後");
    Serial.println("===================");
    //Serial.print("az:");
    //Serial.print(rawvalue[2]);
    //Serial.print("\n");
    Serial.print("gx:");
    Serial.print(rawvalue[3]);
    Serial.print("\t");
    Serial.print("gy:");
    Serial.print(rawvalue[4]);
    Serial.print("\t");
    Serial.print("gz:");
    Serial.print(rawvalue[5]);
    Serial.print("\n");

    //radium->degree
    ypr[0]=ypr[0]*180/M_PI;
    ypr[1]=ypr[1]*180/M_PI;
    ypr[2]=ypr[2]*180/M_PI;
    //將90~-90轉為0~180給伺服馬達
    int yawvalue=map(ypr[0],-90,90,0,180);
    int pitchvalue=map(ypr[1],90,-90,0,180);
    int rollvalue=map(ypr[2],-90,90,0,180);

    syaw.write(yawvalue);
    spitch.write(pitchvalue);
    sroll.write(rollvalue);

  }

}
