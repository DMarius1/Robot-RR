#include "Arduino.h"
#include "math.h"
#include "AX12A.h"
#include <Servo.h> ///importare librarii
#define DirectionPin   (10u)
#define BaudRate      (1000000ul)
#define motor1        (1)
#define motor2        (14)
#define motor3        (18) ///definire motoare
#define UP 450u
#define DOWN 507u

float q1=0;
float q2=0;

float l1= 9.3;
float l2=15;


void setup() {
  ax12a.begin(BaudRate, DirectionPin, &Serial); 

}

void CI (float xe, float ye)
{
    float L = sqrt(xe*xe + ye*ye);
    q1 = (int)ceil((acos((L*L - (l1*l1) - (l2*l2)) / (2 * l1 * l2))) * 57.2957795);
    q2 = (int)ceil((atan(ye/xe) - acos((l2*l2 - L*L - l1*l1) / (-2 * L * l1))) * 57.2957795);
    
    /*if (q1 < 0 || q2 < 0 || q1 > 1024 || q2 > 1024) {
    q1 = -q1;
    q2 = -q2;
  }*/

    //cout<<"q1= "<<q1<<endl;
    //cout<<"q2= "<<q2<<endl;

}

void goToPoint(float x, float y)
{
    CI(x,y);
    q1= 1023*q1/300;
    q2= 1023*q2/300;
    //Serial.println(q1);
    //Serial.println(q2);
    //cout<<"q1= "<<q1<<endl;
    //cout<<"q2= "<<q2<<endl;
    int m1=307+q1;
    int m2=512+q2;
    Serial.println(m1);
    delay(10);
    Serial.println(m2);
    delay(10);
    ax12a.moveSpeed(14, m1, 30);
    ax12a.moveSpeed(18, m2, 30);
}

void drawLine(float xs,float ys, float xf, float yf)
{
    float dx= xf-xs;
    float dy= yf-ys;

    float pasX=dx/10;
    float pasY=dy/10;
    pencilUp();
    goToPoint(xs,ys);
    delay(3000);
    pencilDown();
    for(int i=0;i<=10;i++)
    {
        //cout<<"Iteratia:"<<i<<endl;
        goToPoint(xs+pasX*i, ys+pasY*i);
        delay(100);
    }
    pencilUp();
    
}

void pencilUp() {
  ax12a.moveSpeed(1, DOWN, 300);
  unsigned int presentPosition = 0;
  delay(300);
}
void pencilDown() {
  ax12a.moveSpeed(1, UP, 300);
  unsigned int presentPosition = 0;
  delay(300);
}

void reset()
{
  ax12a.moveSpeed(18, 307, 60);
  ax12a.moveSpeed(14, 512, 60);
  ax12a.moveSpeed(1, 512, 60);
}
void loop() {

    //q1=30;
    //q2=50;
    
    //q1= 1023*q1/300;
    //q2= 1023*q2/300;
    //Serial.println(q1);
    //Serial.println(q2);
    //cout<<"q1= "<<q1<<endl;
    //cout<<"q2= "<<q2<<endl;
    /*int m1=307+q1;
    int m2=512+q2;
    Serial.println(m1);
    delay(10);
    Serial.println(m2);
    delay(10);
    ax12a.moveSpeed(12, m1, 60);
    ax12a.moveSpeed(13, m2, 60);
*/

   
   //reset();
   //delay(2000);
   //ax12a.moveSpeed(14, 495, 60);
  //drawLine(3.3732, 21.8963, -4.99181, 19.5447);
  //delay(1000);
  //goToPoint(-23.4369, -4.98985);
  //drawLine(10.6588, 19.4221, 19.5447, 4.99181);
  //delay(3000);
  drawLine(19.5447, 4.99181, 18.381, 8.30988);
  delay(2000);
  //drawLine(18.381, 8.30988, 15.381, 5.30988);
  //delay(2000);
  //drawLine(-4.99181, 19.5447, 22.1494, 0.480302);
  //delay(3000);
  //reset();
  //delay(3000);
}



//////////////////////////////////
