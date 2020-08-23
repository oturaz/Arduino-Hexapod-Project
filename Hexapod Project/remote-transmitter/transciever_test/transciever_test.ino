#include <VirtualWire.h>
#include<math.h>

const int SW_pin=2;
const int X_pin=0;
const int Y_pin=1;
const int SW_pin2=3;
const int X_pin2=2;
const int Y_pin2=3;
#ifndef M_PI
#define M_PI 3.1415926535
#endif

float toDegrees(float radian)
{
  return radian*(180.0/M_PI);
}

void setup() 
{   
    pinMode(SW_pin,INPUT);
    pinMode(SW_pin2,INPUT);
    digitalWrite(SW_pin,HIGH);
    digitalWrite(SW_pin2,HIGH);
    Serial.begin(9600);
    // Initialize the IO and ISR 
    vw_setup(2000); // Bits per sec 
}

void loop() 
{ 
    

    String str;
    int height=0,distance=0;
    int ypin=analogRead(Y_pin),xpin=analogRead(X_pin);
    int ypin2=analogRead(Y_pin2)/100,xpin2=analogRead(X_pin2)/100;
    float angle;
    int dangle;

    if(xpin2==0) height=1;
    if(xpin2==10) height=2;
    
    ypin=map(ypin,0,1023,1000,0);
    ypin/=10;
    xpin=map(xpin,0,1023,0,1000);
    xpin/=10;

    angle=atan((xpin-49)/(ypin-52));
    dangle=toDegrees(angle);
    if (ypin<52&&xpin>=49) dangle=dangle+180;
    if (ypin<52&&xpin<49) dangle=dangle+180;
    if (ypin>52&&xpin<49) dangle=dangle+360;
    if (ypin==52&&xpin==0) dangle=270;
    if (ypin==52&&xpin==100) dangle=90;
    if(dangle==360) dangle=0;

    if(dangle>=0) distance=2;
    
    str=String(distance);
    str+=String(height);
    if(dangle<0)str+=String("0");
    if(dangle<10)str+=String("0");
    if(dangle<100)str+=String("0");
    str+=String(dangle);
    str+=String("0000");
    int i=str.length()+1;
    char chr[i];
    
    str.toCharArray(chr,i);
    
    
    Serial.print(chr);
    Serial.print("\n");
    Serial.print("Switch: ");
    Serial.print(digitalRead(SW_pin2));
    Serial.print("\n");
    Serial.print("X-axis: ");
    Serial.print(xpin2);
    Serial.print("\n");
    Serial.print("Y-axis: ");
    Serial.print(ypin2);
    Serial.print("\n");
    Serial.print("\n");

    if(dangle>=0||height>0) send(chr);
    delay(100);
    
}

void send (char *message) 
{ 
    vw_send((uint8_t *)message, strlen(message)); 
    vw_wait_tx(); // Wait until the whole message is gone 
}
