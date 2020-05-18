//Import librarii============================================
#include "math.h"
#include "AX12A.h"

//Definire constante==========================================
#define DirectionPin  (10u)
#define BaudRate      (1000000ul)

#define motor1        (12u)   //Motoare
#define motor2        (13u)
#define motor3        (14u) 

#define L1            (0.0935f)   //Lungimi brat
#define L2            (0.166f)
#define Offset_L1     (205u)      //Offset pozitie initiala
#define Offset_L2     (512u)

#define Rad_Grad      (57.2957f)    //Transformare radieni in grade
#define Grad_Digi     (3.4133f)     //Transformare grade in valoare digitala motor

#define Discretizare  (10u)         //Numar de pasi intre 2 puncte       
#define Space         (0.002f)      //Spatiul intre 2 caractere
#define Letter_size   (0.01f)       //Marimea caracterului
#define Scale         (1.5f)        //Scara caracterului
#define Scale2        (3.5f)        //Scara pentru Check box

#define Toleranta     (5u)          //Toleranta de pozitionare

//Structura robot 
struct Postura
{
  int q1;     //cupla 1
  int q2;     //cupla 2
};

//Structura coordonate punct
struct Punct
{
  float x;  
  float y;
};

void setup() {
  ax12a.begin(BaudRate, DirectionPin, &Serial);   //Initializare motoare
}

//resetare pozitie
void reset_position()
{
  ax12a.moveSpeed(motor1, 512, 100);
  ax12a.moveSpeed(motor2, 512, 100);
  ax12a.moveSpeed(motor3, 600, 100);
  delay(3000);
}

//Cinematica inversa ce returneaza valorile din cele 2 cuple 
Postura Cin_Inv(float xe,float ye)    //Primeste coordonatele punctului
{
  Postura tempPost;                                                                                   //postura temporara utilizand structura Postura
  float L = sqrt(xe*xe + ye*ye);                                                                      //Lungimea de la origine la punct
  tempPost.q1 = (int)((atan(ye/xe) - acos((L2*L2 - L*L - L1*L1)/(-2.f*L*L1)))*Rad_Grad*Grad_Digi);    //Rotatie cupla 1 in radieni -> grade -> val digitala motor
  tempPost.q2 = (int)(acos((L*L - L1*L1 - L2*L2) / (2.f * L1 * L2))*Rad_Grad*Grad_Digi);              //Rotatie cupla 2 in radieni -> grade -> val digitala motor
  tempPost.q1 += Offset_L1;                                                                           //Offset pozitie initiala
  tempPost.q2 += Offset_L2;                                                                           //Offset pozitie initiala
  return tempPost;                                                                                    //Returnare valori cuple
}

//Revenire la 0 daca valorile sunt in afara vol de lucru
void goZERO()
{
  Postura baza;                                                                                       //baza de tipul Postura
  baza = Cin_Inv(L1+L2,0);                                                                            //Calcul cinematica inversa
  ax12a.moveSpeed(motor1, baza.q1, 100);                                                              //Miscare motor 1
  ax12a.moveSpeed(motor2, baza.q2, 100);                                                              //Miscare motor 2
  ax12a.moveSpeed(motor3, 700, 100);                                                                  //Miscare motor 3
  delay(3000);
}

//Mergi la punctul specificat
void goToPoint(float xe, float ye)    //Primeste coordonatele punctului
{
  Postura punct;      //Variabila temporara de tip Postura 
  punct = Cin_Inv(xe, ye);        //Calcul cinematica inversa
  ax12a.moveSpeed(motor1, punct.q1, 100);     //Miscare motor 1
  ax12a.moveSpeed(motor2, punct.q2, 100);     //Miscare motor 2
  delay(1000);
}

//Coborare creion
void pencilDOWN()
{
  ax12a.moveSpeed(motor3, 512 , 100);
  delay(500);
}

//Ridicare creion
void pencilUP()
{
  ax12a.moveSpeed(motor3, 600, 100);
  delay(500);
}

//Functie de citire a pozitiei curente 
Postura ReadPosture()
{
  Postura tempPost;
  tempPost.q1 = ax12a.readPosition(motor1);
  tempPost.q2 = ax12a.readPosition(motor2);
  return tempPost;      //Returneaza pozitia curenta intr-o variabila de tip Postura
}

//Functie de comparare pozitie curenta reala cu cea dorita
bool pointReach(int realQ1, int realQ2, int tintaQ1, int tintaQ2)
{
  bool reach = false;
  if( realQ1>=(tintaQ1-Toleranta) && realQ1<=(tintaQ1+Toleranta) && realQ2>=(tintaQ2-Toleranta) && realQ2<=(tintaQ2+Toleranta) )
  {
    reach = true;     //Daca pozitia curenta se incadreaza in intervalul tinta primeste 'true'
  }

   return reach;    //Returneaza 'true' daca pozitia curenta corespunde cu pozitia tinta si 'false' in caz contrar
}


//Functie pentru a desena o linie
void drawLINE(float xs, float ys, float xf, float yf)   //Primeste coordonatele punctului de inceput si de sfarsit
{
  goToPoint(xs,ys);   //Mergi la punctul de start
  pencilDOWN();       //Creion jos
  Punct traj[Discretizare];     //Vector traiectorie de tip Punct, cu marimea discretizarii
  
  for(int i=0; i<=Discretizare; i++)    //for cu numarul pasilor dati de discretizare
  {
    //Distanta dintre punctul final si initial 
    float dx = xf-xs;   //Pe axa X
    float dy = yf-ys;   //Pe axa Y
    //Impartire distanta in pasi dati de discretizare
    float pasX = dx/ (float)Discretizare;   //Pe axa X
    float pasY = dy/ (float)Discretizare;   //Pe axa Y
    //Traiectoria primeste pe rand pasii
    traj[i].x = xs + pasX*i;    //Pe axa X
    traj[i].y = ys + pasY*i;    //Pe axa Y

    Postura tempPost;       //Postura temporara 
    tempPost = Cin_Inv(traj[i].x,  traj[i].y);    //Calcul cinematica inversa pt postura temporara cu traiectoria curenta
    ax12a.moveSpeed(motor1, tempPost.q1, 80);     //Miscare motoare
    ax12a.moveSpeed(motor2, tempPost.q2, 80);
    delay (100);
  }
  pencilUP();   //Ridicare Creion
}

//Definire litere prin linii
void drawLETTER(char c, Punct &baza)    //Primeste litera si variabila baza de tip Punct
{
  switch (c)    //Verificare litera primita si alegere caz potrivit
  {
    case 'A':   //Fiecare caz are cate o litera a alfabetului definita prin linii
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.005*Scale, baza.y + 0.01*Scale);      //Functie de scriere a unei linii de la un punct la altul
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.00*Scale);    //Punctele sunt descrise de coordonate
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.005*Scale);    //Coordonatele sunt alcatuite din
        break;                                                                                                              //baza.x/y = pct de plecare robot
    case 'B':                                                                                                               //+ coordonata punctului*scara 
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.008*Scale, baza.y + 0.005*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.005*Scale, baza.x + 0.0*Scale, baza.y + 0.005*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        break;
    case 'C':
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.008*Scale, baza.x + 0.008*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.01*Scale, baza.x + 0.002*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.008*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.008*Scale, baza.x + 0.0*Scale, baza.y + 0.002*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.002*Scale, baza.x + 0.002*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.0*Scale, baza.x + 0.008*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.002*Scale);
        break;
    case 'D':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.006*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.008*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.008*Scale, baza.x + 0.01*Scale, baza.y + 0.002*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.002*Scale, baza.x + 0.006*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        break;
    case 'E':
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.006*Scale, baza.y + 0.005*Scale);
        break;
    case 'F':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.006*Scale, baza.y + 0.005*Scale);
        break;
    case 'G':
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.008*Scale, baza.x + 0.008*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.01*Scale, baza.x + 0.002*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.008*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.008*Scale, baza.x + 0.0*Scale, baza.y + 0.002*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.002*Scale, baza.x + 0.002*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.0*Scale, baza.x + 0.008*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.002*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.002*Scale, baza.x + 0.01*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.004*Scale, baza.x + 0.006*Scale, baza.y + 0.004*Scale);
        break;
    case 'H':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.005*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'I':
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.01*Scale, baza.x + 0.008*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.0*Scale, baza.x + 0.008*Scale, baza.y + 0.0*Scale);
        break;
    case 'J':
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.01*Scale, baza.x + 0.008*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.0*Scale, baza.x + 0.002*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.002*Scale);
        break;
    case 'K':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case 'L':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case 'M':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.006*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case 'N':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'O':
        drawLINE(baza.x + 0.004*Scale, baza.y + 0.01*Scale, baza.x + 0.006*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.006*Scale, baza.x + 0.01*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.004*Scale, baza.x + 0.006*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.0*Scale, baza.x + 0.004*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.004*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.004*Scale, baza.x + 0.0*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.006*Scale, baza.x + 0.004*Scale, baza.y + 0.01*Scale);
        break;
    case 'P':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.006*Scale, baza.x + 0.002*Scale, baza.y + 0.006*Scale);
        break;
    case 'Q':
        drawLINE(baza.x + 0.004*Scale, baza.y + 0.01*Scale, baza.x + 0.006*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.006*Scale, baza.x + 0.01*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.004*Scale, baza.x + 0.006*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.0*Scale, baza.x + 0.004*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.004*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.004*Scale, baza.x + 0.0*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.006*Scale, baza.x + 0.004*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.006*Scale, baza.y + 0.004*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case 'R':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.006*Scale, baza.x + 0.002*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.006*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case 'S':
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.005*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.005*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.005*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        break;
    case 'T':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.0*Scale);
        break;
    case 'U':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'V':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'W':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.002*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.002*Scale, baza.y + 0.0*Scale, baza.x + 0.005*Scale, baza.y + 0.004*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.004*Scale, baza.x + 0.008*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.008*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'X':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        break;
    case 'Y':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.005*Scale, baza.y + 0.006*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.006*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.005*Scale, baza.y + 0.006*Scale, baza.x + 0.05*Scale, baza.y + 0.0*Scale);
        break;
    case 'Z':
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.01*Scale, baza.x + 0.01*Scale, baza.y + 0.01*Scale);
        drawLINE(baza.x + 0.01*Scale, baza.y + 0.01*Scale, baza.x + 0.0*Scale, baza.y + 0.0*Scale);
        drawLINE(baza.x + 0.0*Scale, baza.y + 0.0*Scale, baza.x + 0.01*Scale, baza.y + 0.0*Scale);
        break;
    case ' ': //Caz pentru spatiu
        //baza.x = baza.x + Scale*(Letter_size + Space); 
        break;
    case '|': //Caz pentru check box-ul optiunii speciale
        //Check box
        drawLINE(baza.x + 0.0*Scale2, baza.y + 0.0*Scale2, baza.x + 0.0*Scale2, baza.y + 0.01*Scale2);
        drawLINE(baza.x + 0.0*Scale2, baza.y + 0.01*Scale2, baza.x + 0.01*Scale2, baza.y + 0.01*Scale2);
        drawLINE(baza.x + 0.01*Scale2, baza.y + 0.01*Scale2, baza.x + 0.01*Scale2, baza.y + 0.0*Scale2);
        drawLINE(baza.x + 0.01*Scale2, baza.y + 0.0*Scale2, baza.x + 0.0*Scale2, baza.y + 0.0*Scale2);

        //Tick mark
        drawLINE(baza.x + 0.0*Scale2, baza.y + 0.005*Scale2, baza.x + 0.003*Scale2, baza.y + 0.0*Scale2);
        drawLINE(baza.x + 0.003*Scale2, baza.y + 0.0*Scale2, baza.x + 0.01*Scale2, baza.y + 0.01*Scale2);
        break;
    default:
        break;
  }
  baza.x = baza.x + Scale*(Letter_size + Space);        //Redefinire baza pe axa X pentru urmatoarea litera
}

//Functie de scriere a cuvintelor 
void drawWORD (char cuvant[30], Punct &baza)    //Primeste ca parametri cuvantul de tip char si o structura de tip Punct
{
  for(int i=0; i<strlen(cuvant); i++)   //for de lungimea cuvantului
  {
    drawLETTER (cuvant[i], baza);   //Functia de scriere a literei specificate
  }
}

//In loop se defineste punctul de plecare al robotului si functia de scriere a literei dorite
void loop() 
{
  //goToPoint(baza.x, baza.y);
  //Serial.println(ReadPosture().q1);
  //Serial.println(ReadPosture().q2);

  
  //Coordonate pentru primul rand
  Punct baza;
  baza.x = 0.0;
  baza.y = 0.134;
  
  //Optiune speciala
  //char cuvant1[30]="I AM NOT";
  //char cuvant2[30]="A ROBOT |";

  char cuvant1[30] = "ABC";
  char cuvant2[30] = "DEF";
  //drawLETTER ('R', baza);
  drawWORD(cuvant1, baza);
  
  //Coordonate pentru al doilea rand
  baza.x = 0.01;
  baza.y = 0.11;
  drawWORD(cuvant2, baza);
  //drawLETTER ('T', baza);
  delay (5000);
}
