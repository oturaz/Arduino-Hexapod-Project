#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include<ServoTimer2.h>
#include<math.h>
#include <VirtualWire.h>

byte message[VW_MAX_MESSAGE_LEN]; // a buffer to store the incoming messages 
byte messageLength = VW_MAX_MESSAGE_LEN; // the size of the message


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN0  115 
#define SERVOMIN1  125
#define SERVOMIN2  125 
#define SERVOMIN3  115
#define SERVOMIN4  125 
#define SERVOMIN5  125
#define SERVOMIN6  120 
#define SERVOMIN7  120
#define SERVOMIN8  125 
#define SERVOMIN9  120
#define SERVOMIN10  120 
#define SERVOMIN11  155
#define SERVOMIN12  120 
#define SERVOMIN13  125
#define SERVOMIN14  115 
#define SERVOMIN15  125

#define SERVOMAX0  575
#define SERVOMAX1  590
#define SERVOMAX2  580
#define SERVOMAX3  565
#define SERVOMAX4  590
#define SERVOMAX5  575
#define SERVOMAX6  580
#define SERVOMAX7  550
#define SERVOMAX8  570
#define SERVOMAX9  565
#define SERVOMAX10  580
#define SERVOMAX11  585
#define SERVOMAX12  570
#define SERVOMAX13  570
#define SERVOMAX14  535
#define SERVOMAX15  590

#define SERVOMIN16  140
#define SERVOMAX16  450
#define SERVOMIN17  150
#define SERVOMAX17  450

#ifndef M_PI 
#define M_PI 3.1415926535 
#endif

long int timecount=0;
int movecheck=1;

double constanta =4.8828125;
ServoTimer2 myservo16;
ServoTimer2 myservo17;
float Tibia=5.9,Femur=4.1,Coxa=5.2;
struct point
{
  float x,y,z;
};
struct leg
{
  float u1,u2,u3,angle1,angle2,angle3;
  point location,pos;
  float Zoffset,L,L1,Alpha,Alpha1,Alpha2,Beta,Gamma=0;
}leg[7];
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float getGamma(float x,float y)
{
  return atan(y/x);
}
float getAlpha1(float zoffset,float l)
{
  return acos(zoffset/l);
}
float getAlpha2(float l)
{
  return acos((Tibia*Tibia-Femur*Femur-l*l)/(-2*Femur*l));
}
float getBeta(float l)
{
  return acos((l*l-Tibia*Tibia-Femur*Femur)/(-2*Tibia*Femur));
}
float getL(float zoffset,float l1)
{
  return sqrtf(zoffset*zoffset+((l1-Coxa)*(l1-Coxa)));
}
float getL1(float x1,float x2,float y1,float y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
float toDegrees(float radian) {
    return radian * (180.0 / M_PI);
}
float toRadians(float degree)
{
  return (degree * M_PI  / 180.0) ;
}

void calculateAngles(int i)
{
    leg[i].Gamma=getGamma(leg[i].pos.x-leg[i].location.x,leg[i].pos.y-leg[i].location.y);
    leg[i].Zoffset=leg[i].pos.z-leg[i].location.z;
    leg[i].L1=getL1(leg[i].location.x,leg[i].pos.x,leg[i].location.y,leg[i].pos.y);
    leg[i].L=getL(leg[i].Zoffset,leg[i].L1);
    leg[i].Alpha1=getAlpha1(leg[i].Zoffset,leg[i].L);
    leg[i].Alpha2=getAlpha2(leg[i].L);
    leg[i].Alpha=leg[i].Alpha1+leg[i].Alpha2;
    leg[i].Beta=getBeta(leg[i].L);
    
    leg[i].Gamma=toDegrees(leg[i].Gamma);
    leg[i].Alpha=toDegrees(leg[i].Alpha);
    leg[i].Beta =toDegrees(leg[i].Beta);
    switch(i)
  {
    case 1:
    leg[i].Gamma-=60;
    break;
    case 2:
    
    break;
    case 3:
    leg[i].Gamma+=60;
    break;
    case 4:
    leg[i].Gamma-=60;
    break;
    case 5:
    
    break;
    case 6:
    leg[i].Gamma+=60;
    break;
  }
    //Serial.println(leg[i].Gamma);
    
    if(leg[i].Gamma>0) leg[i].u1=map(leg[i].Gamma,0,47,90,40);
    else leg[i].u1=map(leg[i].Gamma,0,-42,90,180);
  
    leg[i].u2=map(leg[i].Alpha,12,94,180,0);
  
    if(leg[i].Beta<90) leg[i].u3=map(leg[i].Beta,0,90,180,78);
    else leg[i].u3=map(leg[i].Beta,90,138,78,0);
}

void initLeg(int i)
{
  switch(i)
  {
    case 1:
    leg[i].pos.x=13.93675;
    leg[i].pos.y=9.498148;
    leg[i].pos.z=7.101;

    leg[i].location.x=17.725;
    leg[i].location.y=16;
    leg[i].location.z=0;
    break;
    
    case 2:
    leg[i].pos.x=7.8735;
    leg[i].pos.y=20;
    leg[i].pos.z=7.101;

    leg[i].location.x=15.4;
    leg[i].location.y=20;
    leg[i].location.z=0;
    break;

    case 3:
    leg[i].pos.x=13.93675;
    leg[i].pos.y=30.501852;
    leg[i].pos.z=7.101;

    leg[i].location.x=17.7255;
    leg[i].location.y=24;
    leg[i].location.z=0;
    break;

    case 4:
    leg[i].pos.x=26.06325;
    leg[i].pos.y=30.501852;
    leg[i].pos.z=7.101;

    leg[i].location.x=22.275;
    leg[i].location.y=24;
    leg[i].location.z=0;
    break;
    
    case 5:
    //myservo16.attach(9,SERVOMIN16*constanta,SERVOMAX16*constanta);
    //myservo17.attach(10,SERVOMIN17*constanta,SERVOMAX17*constanta);
    myservo16.attach(9);
    myservo17.attach(10);
    
    leg[i].pos.x=32.1265;
    leg[i].pos.y=20;
    leg[i].pos.z=7.101;

    leg[i].location.x=24.6;
    leg[i].location.y=20;
    leg[i].location.z=0;
    break;

    case 6:
    leg[i].pos.x=26.06325;
    leg[i].pos.y=9.498148;
    leg[i].pos.z=7.101;

    leg[i].location.x=22.275;
    leg[i].location.y=16;
    leg[i].location.z=0;
    break;
  }
  calculateLeg(i);
}

void calculateLeg(int i)
{
  switch(i)
  {
    case 1:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN3, SERVOMAX3);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN4, SERVOMAX4);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN5, SERVOMAX5);
    break;
    
    case 2:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN6, SERVOMAX6);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN7, SERVOMAX7);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN8, SERVOMAX8);
    break;

    case 3:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN9, SERVOMAX9);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN10, SERVOMAX10);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN11, SERVOMAX11);
    break;
    
    case 4:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN12, SERVOMAX12);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN13, SERVOMAX13);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN14, SERVOMAX14);
    break;
    
    case 5:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN15, SERVOMAX15);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN16*constanta,SERVOMAX16*constanta);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN17*constanta,SERVOMAX17*constanta);
    break;

    case 6:
    calculateAngles(i);
    leg[i].angle3 = map(leg[i].u3, 0, 180, SERVOMIN0, SERVOMAX0);
    leg[i].angle2 = map(leg[i].u2, 0, 180, SERVOMIN1, SERVOMAX1);
    leg[i].angle1 = map(leg[i].u1, 0, 180, SERVOMIN2, SERVOMAX2);
    break;
  }
}
void setLeg(int i)
{
  switch(i)
  {
    case 1:
    pwm.setPWM(3, 0, leg[i].angle3);
    pwm.setPWM(4, 0, leg[i].angle2);
    pwm.setPWM(5, 0, leg[i].angle1);
    break;
    case 2:
    pwm.setPWM(6, 0, leg[i].angle3);
    pwm.setPWM(7, 0, leg[i].angle2);
    pwm.setPWM(8, 0, leg[i].angle1);
    break;
    case 3:
    pwm.setPWM(9, 0, leg[i].angle3);
    pwm.setPWM(10, 0, leg[i].angle2);
    pwm.setPWM(11, 0, leg[i].angle1);
    break;
    case 4:
    pwm.setPWM(12, 0, leg[i].angle3);
    pwm.setPWM(13, 0, leg[i].angle2);
    pwm.setPWM(14, 0, leg[i].angle1);
    break;
    case 5:
    pwm.setPWM(15, 0, leg[i].angle3);
    myservo16.write(  leg[i].angle2);
    myservo17.write(  leg[i].angle1);
    break;
    case 6:
    pwm.setPWM(0, 0, leg[i].angle3);
    pwm.setPWM(1, 0, leg[i].angle2);
    pwm.setPWM(2, 0, leg[i].angle1);
    break;
  }
}

void moveLeg(int i,float rot,float d,float z)
{
  float x,y;
  x=sin(toRadians(rot))*d;
  y=cos(toRadians(rot))*d;
  //Serial.println(x);
  //Serial.println(y);
  leg[i].pos.x+=x;
  leg[i].pos.y+=y;
  leg[i].pos.z+=z;
  calculateLeg(i);
}

void moveBody(float rot,float d,float z)
{
      moveLeg(1,rot,-d/2,0);
      moveLeg(3,rot,-d/2,0);
      moveLeg(5,rot,-d/2,0);
   
      moveLeg(2,rot,d/2,-2);
      moveLeg(4,rot,d/2,-2);
      moveLeg(6,rot,d/2,-2);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(100);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(250);

      moveLeg(1,rot,-d/2,0);
      moveLeg(3,rot,-d/2,0);
      moveLeg(5,rot,-d/2,0);
   
      moveLeg(2,rot,d/2,2);
      moveLeg(4,rot,d/2,2);
      moveLeg(6,rot,d/2,2);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(100);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(250);
    
      moveLeg(1,rot,d,0);
      moveLeg(3,rot,d,0);
      moveLeg(5,rot,d,0);
   
      moveLeg(2,rot,-d,0);
      moveLeg(4,rot,-d,0);
      moveLeg(6,rot,-d,0);

  //1,3,5 forward
      moveLeg(2,rot,-d/2,0);
      moveLeg(4,rot,-d/2,0);
      moveLeg(6,rot,-d/2,0);
   
      moveLeg(1,rot,d/2,-2);
      moveLeg(3,rot,d/2,-2);
      moveLeg(5,rot,d/2,-2);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(100);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(250);
      moveLeg(2,rot,-d/2,0);
      moveLeg(4,rot,-d/2,0);
      moveLeg(6,rot,-d/2,0);
   
      moveLeg(1,rot,d/2,2);
      moveLeg(3,rot,d/2,2);
      moveLeg(5,rot,d/2,2);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(100);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(250);
    
      moveLeg(2,rot,d,0);
      moveLeg(4,rot,d,0);
      moveLeg(6,rot,d,0);
   
      moveLeg(1,rot,-d,0);
      moveLeg(3,rot,-d,0);
      moveLeg(5,rot,-d,0);
}

void moveBody2(float rot,float d,float z)
{
  for(int i=0;i<3;i++)
  {
      moveLeg(1,rot,-d/6.0,0);
      moveLeg(3,rot,-d/6.0,0);
      moveLeg(5,rot,-d/6.0,0);
   
      moveLeg(2,rot,d/6.0,-z/3.0);
      moveLeg(4,rot,d/6.0,-z/3.0);
      moveLeg(6,rot,d/6.0,-z/3.0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
  for(int i=0;i<3;i++)
  {
      moveLeg(1,rot,-d/6.0,0);
      moveLeg(3,rot,-d/6.0,0);
      moveLeg(5,rot,-d/6.0,0);
   
      moveLeg(2,rot,d/6.0,z/3.0);
      moveLeg(4,rot,d/6.0,z/3.0);
      moveLeg(6,rot,d/6.0,z/3.0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
    
      moveLeg(1,rot,d,0);
      moveLeg(3,rot,d,0);
      moveLeg(5,rot,d,0);
   
      moveLeg(2,rot,-d,0);
      moveLeg(4,rot,-d,0);
      moveLeg(6,rot,-d,0);

  //1,3,5 forward
  for(int i=0;i<3;i++)
  {
      moveLeg(2,rot,-d/6.0,0);
      moveLeg(4,rot,-d/6.0,0);
      moveLeg(6,rot,-d/6.0,0);
   
      moveLeg(1,rot,d/6.0,-z/3.0);
      moveLeg(3,rot,d/6.0,-z/3.0);
      moveLeg(5,rot,d/6.0,-z/3.0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
  for(int i=0;i<3;i++)
  {
      moveLeg(2,rot,-d/6.0,0);
      moveLeg(4,rot,-d/6.0,0);
      moveLeg(6,rot,-d/6.0,0);
   
      moveLeg(1,rot,d/6.0,z/3.0);
      moveLeg(3,rot,d/6.0,z/3.0);
      moveLeg(5,rot,d/6.0,z/3.0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
    
      moveLeg(2,rot,d,0);
      moveLeg(4,rot,d,0);
      moveLeg(6,rot,d,0);
   
      moveLeg(1,rot,-d,0);
      moveLeg(3,rot,-d,0);
      moveLeg(5,rot,-d,0);
}

void moveBody3(float rot,float d,float z)
{
      moveLeg(2,0,0,-z);
      moveLeg(4,0,0,-z);
      moveLeg(6,0,0,-z);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  for(int i=0;i<6;i++)
  {
      moveLeg(1,rot,-d/6.0,0);
      moveLeg(3,rot,-d/6.0,0);
      moveLeg(5,rot,-d/6.0,0);
   
      moveLeg(2,rot,d/6.0,0);
      moveLeg(4,rot,d/6.0,0);
      moveLeg(6,rot,d/6.0,0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
      moveLeg(2,0,0,z);
      moveLeg(4,0,0,z);
      moveLeg(6,0,0,z);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
    
      moveLeg(1,rot,d,0);
      moveLeg(3,rot,d,0);
      moveLeg(5,rot,d,0);
   
      moveLeg(2,rot,-d,0);
      moveLeg(4,rot,-d,0);
      moveLeg(6,rot,-d,0);

  //1,3,5 forward
      moveLeg(1,0,0,-z);
      moveLeg(3,0,0,-z);
      moveLeg(5,0,0,-z);
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    
  for(int i=0;i<6;i++)
  {
      moveLeg(2,rot,-d/6.0,0);
      moveLeg(4,rot,-d/6.0,0);
      moveLeg(6,rot,-d/6.0,0);
   
      moveLeg(1,rot,d/6.0,0);
      moveLeg(3,rot,d/6.0,0);
      moveLeg(5,rot,d/6.0,0);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(30);
  }
      moveLeg(1,0,0,z);
      moveLeg(3,0,0,z);
      moveLeg(5,0,0,z);
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(30);
    
      moveLeg(2,rot,d,0);
      moveLeg(4,rot,d,0);
      moveLeg(6,rot,d,0);
   
      moveLeg(1,rot,-d,0);
      moveLeg(3,rot,-d,0);
      moveLeg(5,rot,-d,0);
}

void moveHeight(float z)
{
  
      moveLeg(1,0,0,z);
      moveLeg(3,0,0,z);
      moveLeg(5,0,0,z);
      moveLeg(2,0,0,z);
      moveLeg(4,0,0,z);
      moveLeg(6,0,0,z);
    
    setLeg(1);
    setLeg(3);
    setLeg(5);
    delay(200);
    setLeg(2);
    setLeg(4);
    setLeg(6);
    delay(200);
      moveLeg(1,0,0,-z);
      moveLeg(3,0,0,-z);
      moveLeg(5,0,0,-z);
      moveLeg(2,0,0,-z);
      moveLeg(4,0,0,-z);
      moveLeg(6,0,0,-z);
}

void resetBody(float inaltime)
{
        moveLeg(1,0,0,-inaltime);
        moveLeg(3,0,0,-inaltime);
        moveLeg(5,0,0,-inaltime);
        setLeg(1);
        setLeg(3);
        setLeg(5);
        delay(200);
        
        moveLeg(1,0,0,inaltime);
        moveLeg(3,0,0,inaltime);
        moveLeg(5,0,0,inaltime);
        setLeg(1);
        setLeg(3);
        setLeg(5);
        delay(200);
        
        moveLeg(2,0,0,-inaltime);
        moveLeg(4,0,0,-inaltime);
        moveLeg(6,0,0,-inaltime);
        setLeg(2);
        setLeg(4);
        setLeg(6);
        delay(200);
        
        moveLeg(2,0,0,inaltime);
        moveLeg(4,0,0,inaltime);
        moveLeg(6,0,0,inaltime);
        setLeg(2);
        setLeg(4);
        setLeg(6);
        delay(200);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() 
{
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");

  initLeg(1);
  initLeg(2);
  initLeg(3);
  initLeg(4);
  initLeg(5);
  initLeg(6);

  // Initialize the IO and ISR 
    vw_setup(2000); // Bits per sec

    vw_rx_start(); // Start the receiver 
    
  //float anglex,angley;
  //myservo16.attach(9);
  //myservo17.attach(10);
  //anglex=map(60, 0, 180, SERVOMIN16*constanta, SERVOMAX16*constanta);
  //angley=map(90, 0, 180, SERVOMIN15, SERVOMAX15);
  //angle=map(u3, 0, 180, SERVOMIN17*constanta, SERVOMAX17*constanta);
  //myservo16.write(  anglex);
  //pwm.setPWM(15, 0, angley);
  //myservo17.write(  SERVOMIN17*constanta);
  
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
  yield();
}

void setServoPulse(uint8_t n, double pulse) 
{
  double pulselength;
  pulselength = 1000000;   // 1,000,000 us per second
  pulselength /= 60;   // 60 Hz
  Serial.print(pulselength); Serial.println(" us per period"); 
  pulselength /= 4096;  // 12 bits of resolution
  Serial.print(pulselength); Serial.println(" us per bit"); 
  pulse *= 1000;
  pulse /= pulselength;
  Serial.println(pulse);
  pwm.setPWM(n, 0, pulse);
}

void loop() 
{
  int rotatie=0,distanta=3,inaltime=2;
  
    if (vw_get_message(message, &messageLength)) // Non-blocking 
    { 
      int mesaj,dist,height;
        Serial.print("Received: ");
        dist=(int)message[0]-48;
        height=(int)message[1]-48;
        mesaj=((int)message[2]-48)*100+((int)message[3]-48)*10+((int)message[4]-48);

        if(height==1) {moveHeight(2);}
        if(height==2) {moveHeight(-2);}
        
        rotatie=mesaj;
        if(dist>0) moveBody2(rotatie,distanta,inaltime);
        timecount=0;
        movecheck=1;
        Serial.print(dist);
        Serial.print(height);
        Serial.print(mesaj);
        Serial.println(); 
    }
    else
    {
      timecount+=1;
      if(timecount==100000) 
      {
        Serial.print("No message\n");
        if(movecheck==1) resetBody(inaltime);
        movecheck=0;
        timecount=0;
        }
    }
    
      //delay(2000);
      //moveHeight(inaltime);
      //delay(2000);
      //moveHeight(-inaltime);
   
      
}
