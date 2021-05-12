#include<Wire.h>//Arduino header file
#include<DynamixelSerial1.h>//Header file that comes with Dynamixel motor
//Both the header files should work in C as well as C++, also the code, without making any changes 

//Radius of base plate = 17.75cm
const float b1[3] = {17.75, 0, 4.65};
const float b2[3] = {8.875, 15.371, 4.65};
const float b3[3] = {-8.875, 15.371, 4.65};
const float b4[3] = {-17.75, 0, 4.65};
const float b5[3] = {-8.875, -15.371, 4.65};
const float b6[3] = {8.875, -15.375, 4.65};

//Radius of top plate = 18cm
const float p1[3] = {18, 0, 21.941};
const float p2[3] = {9, 15.588, 21.941};
const float p3[3] = {-9, 15.588, 21.941};
const float p4[3] = {-18, 0, 21.941};
const float p5[3] = {-9, -15.588, 21.941};
const float p6[3] = {9, -15.588, 21.941};

float pn1[3], pn2[3], pn3[3], pn4[3], pn5[3], pn6[3];
const int MPU_addr = 0x68;
int16_t AcX,AcY,AcZ;

int minVal = 265;
int maxVal = 402;
int data = 0;
double x, y, z;
float rttn[3][3];
float t[3] = {0, 0, 0};
float correction_x = 4; //Correction in pitch in degrees
float correction_y = 0; //Correction in roll in degrees
float correction_z = 0; //Correction in yaw in degrees
float rad = 1.591 * DEG_TO_RAD, RLL = 0, PTCH = 0;
bool flag = true, hm = false;

//Function definitions
float distance(float p[3], float rotate[3][3], float translate[3], float p_new[3], float base[3])
{
  float ans = 0;
  int i, j;
  
  //Rotation
  for(i=0; i<3; i++)
  {
    p_new[i] = 0;
    for(j=0;j<3;j++)
    p_new[i] += p[j] * rotate[j][i];
  }
  
  //Translation
  for(i=0;i<3;i++)
    p_new[i] -= translate[i];
  
  for(i=0; i<3; i++)
    ans += pow((p_new[i] - base[i]),2);
  return sqrt(ans);
}

void setup()
{
  Serial.begin(115200);
  Serial2.begin(9600);
  Dynamixel.begin(1000000,2);
  Dynamixel.setMaxTorque(254,640);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void loop()
{
  float l1,l2,l3,l4,l5,l6;
  float L1,L2,L3,L4,L5,L6;
  float M1,M2,M3,M4,M5,M6;
  float N1,N2,N3,N4,N5,N6;
  float lf1,lf2,lf3,lf4,lf5,lf6;
  
  if(Serial2.available() > 0)
  {
    data = Serial2.read();
    if(data == 78)
    { 
      flag = false;
      hm = false;
    }
    
    if(data == 70)
      flag = true;
  }
  
  if(flag == false)
  {
    if(hm == false)
    {
      Dynamixel.move(254,512);
      delay(50);
      hm = true;
      lf1 = 0;
      lf2 = 0;
      lf3 = 0;
      lf4 = 0;
      lf5 = 0;
      lf6 = 0;
    }

    //Bluetooth control
    if(Serial2.available() > 0)
    {
      data = Serial2.read();
      switch(data)
      {
        case 80 : PTCH += rad;
                  break;
        case 112 : PTCH -= rad;
                  break;
        case 82 : RLL += rad;
                  break;
        case 114 : RLL -= rad;
                  break;
        case 84 : t[2] -= 0.5;
                  break;
        case 116 : t[2] += 0.5;
                  break;
        case 72 : Dynamixel.move(254,512);
                  PTCH = 0;
                  RLL = 0;
                  lf1 = 0;
                  lf2 = 0;
                  lf3 = 0;
                  lf4 = 0;
                  lf5 = 0;
                  lf6 = 0;
                  t[2] = 0;
                  break;
      }
    }
      
    rttn[0][0] = cos(-RLL);
    rttn[0][1] = sin(-RLL) * sin(PTCH);
    rttn[0][2] = - sin(-RLL) * cos(PTCH);
    rttn[1][0] = 0;
    rttn[1][1] = cos(PTCH);
    rttn[1][2] = sin(PTCH);
    rttn[2][0] = sin(-RLL);
    rttn[2][1] = - cos(-RLL) * sin(PTCH);
    rttn[2][2] = cos(-RLL) * cos(PTCH);

    l1 = distance(p1,rttn,t,pn1,b1);
    l2 = distance(p2,rttn,t,pn2,b2);
    l3 = distance(p3,rttn,t,pn3,b3);
    l4 = distance(p4,rttn,t,pn4,b4);
    l5 = distance(p5,rttn,t,pn5,b5);
    l6 = distance(p6,rttn,t,pn6,b6);

    L1 = pow(l1,2) - 299;
    L2 = pow(l2,2) - 299;
    L3 = pow(l3,2) - 299;
    L4 = pow(l4,2) - 299;
    L5 = pow(l5,2) - 299;
    L6 = pow(l6,2) - 299;

    M1 = 10*(pn1[2]-b1[2]);
    M2 = 10*(pn2[2]-b2[2]);
    M3 = 10*(pn3[2]-b3[2]);
    M4 = 10*(pn4[2]-b4[2]);
    M5 = 10*(pn5[2]-b5[2]);
    M6 = 10*(pn6[2]-b6[2]);

    N1 = -10*(cos(0) *(pn1[1]-b1[1]) - sin(0) *(pn1[0]-b1[0]));
    N2 = 10*(cos(PI/3) *(pn2[1]-b2[1]) - sin(PI/3) *(pn2[0]-b2[0]));
    N3 = -10*(cos(2*PI/3) *(pn3[1]-b3[1]) - sin(2*PI/3) *(pn3[0]-b3[0]));
    N4 = 10*(cos(PI) *(pn4[1]-b4[1]) - sin(PI) *(pn4[0]-b4[0]));
    N5 = -10*(cos(4*PI/3) *(pn5[1]-b5[1]) - sin(4*PI/3) *(pn5[0]-b5[0]));
    N6 = 10*(cos(5*PI/3) *(pn6[1]-b6[1]) - sin(5*PI/3) *(pn6[0]-b6[0]));

    //lf is the angle alfa with 2.844 as convertion factor from degrees to domain of 0 - 1023.
    lf1 += (asin(L1/sqrt(pow(M1,2)+pow(N1,2)))-atan(N1/M1)) * RAD_TO_DEG * 2.844;
    lf2 += (asin(L2/sqrt(pow(M2,2)+pow(N2,2)))-atan(N2/M2)) * RAD_TO_DEG * 2.844;
    lf3 += (asin(L3/sqrt(pow(M3,2)+pow(N3,2)))-atan(N3/M3)) * RAD_TO_DEG * 2.844;
    lf4 += (asin(L4/sqrt(pow(M4,2)+pow(N4,2)))-atan(N4/M4)) * RAD_TO_DEG * 2.844;
    lf5 += (asin(L5/sqrt(pow(M5,2)+pow(N5,2)))-atan(N5/M5)) * RAD_TO_DEG * 2.844;
    lf6 += (asin(L6/sqrt(pow(M6,2)+pow(N6,2)))-atan(N6/M6)) * RAD_TO_DEG * 2.844;

    if(L1 / sqrt(pow(M1, 2) + pow(N1, 2)) >= (-1) && L1 / sqrt(pow(M1, 2) + pow(N1, 2)) <= 1)
      Dynamixel.move(1, (512 - lf1));
    delay(50);//50 ms delay
    if(L2 / sqrt(pow(M2, 2) + pow(N2, 2)) >= (-1) && L2 / sqrt(pow(M2, 2) + pow(N2, 2)) <= 1)
      Dynamixel.move(2, (512 + lf2));
    delay(50);
    if(L3 / sqrt(pow(M3, 2) + pow(N3, 2)) >= (-1) && L3 / sqrt(pow(M3, 2)+pow(N3, 2)) <= 1)
      Dynamixel.move(3, (512 - lf3));
    delay(50);
    if(L4 / sqrt(pow(M4, 2) + pow(N4, 2)) >= (-1) && L4 / sqrt(pow(M4, 2) + pow(N4, 2)) <= 1)
      Dynamixel.move(4, (512 + lf4));
    delay(50);
    if(L5 / sqrt(pow(M5, 2) + pow(N5, 2)) >= (-1) && L5 / sqrt(pow(M5, 2) + pow(N5, 2)) <= 1)
      Dynamixel.move(5, (512 - lf5));
    delay(50);
    if(L6 / sqrt(pow(M6, 2) + pow(N6, 2)) >= (-1) && L6 / sqrt(pow(M6, 2) + pow(N6, 2)) <= 1)
      Dynamixel.move(6, (512 + lf6))
    delay(50);
  }
  
  if(flag == true)
  {
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true);
    AcX=Wire.read() << 8|Wire.read();
    AcY=Wire.read() << 8|Wire.read();
    AcZ=Wire.read() << 8|Wire.read();
    int xAng = map(AcX, minVal, maxVal, -90, 90);
    int yAng = map(AcY, minVal, maxVal, -90, 90);
    int zAng = map(AcZ, minVal, maxVal, -90, 90);
    x= (atan2(-yAng, -zAng) + PI) + correction_x * DEG_TO_RAD;
    y= - (atan2(-xAng, -zAng) + PI) + correction_y * DEG_TO_RAD;
    z= (atan2(-yAng, -xAng) + PI) + correction_z * DEG_TO_RAD;//x, y, z values are fed by MPU6050

    rttn[0][0] = cos(y);
    rttn[0][1] = 0;
    rttn[0][2] = sin(y);
    rttn[1][0] = sin(y) * sin(x);
    rttn[1][1] = cos(x);
    rttn[1][2] = - cos(y) * sin(x);
    rttn[2][0] = - sin(y) * cos(x);
    rttn[2][1] = sin(x);
    rttn[2][2] = cos(y) * cos(x);
    t[0] = 4.65 * sin(x);
    t[1] = 4.65 * sin(-y);
    t[2] = 0;
    
    l1 = distance(p1, rttn, t, pn1, b1);
    l2 = distance(p2, rttn, t, pn2, b2);
    l3 = distance(p3, rttn, t, pn3, b3);
    l4 = distance(p4, rttn, t, pn4, b4);
    l5 = distance(p5, rttn, t, pn5, b5);
    l6 = distance(p6, rttn, t, pn6, b6);
    L1 = pow(l1, 2) - 299;
    L2 = pow(l2, 2) - 299;
    L3 = pow(l3, 2) - 299;
    L4 = pow(l4, 2) - 299;
    L5 = pow(l5, 2) - 299;
    L6 = pow(l6, 2) - 299;
    M1 = 10 * (pn1[2] - b1[2]);
    M2 = 10 * (pn2[2] - b2[2]);
    M3 = 10 * (pn3[2] - b3[2]);
    M4 = 10 * (pn4[2] - b4[2]);
    M5 = 10 * (pn5[2] - b5[2]);
    M6 = 10 * (pn6[2] - b6[2]);
    N1 = -10 * (cos(0) * (pn1[1] - b1[1]) - sin(0) * (pn1[0] - b1[0]));
    N2 = 10 * (cos(PI / 3) * (pn2[1] - b2[1]) - sin(PI / 3) *(pn2[0] - b2[0]));
    N3 = -10 * (cos(2 * PI / 3) * (pn3[1] - b3[1]) - sin(2 * PI / 3) * (pn3[0] - b3[0]));
    N4 = 10 * (cos(PI) * (pn4[1] - b4[1]) - sin(PI) * (pn4[0] - b4[0]));
    N5 = -10 * (cos(4 * PI / 3) * (pn5[1] - b5[1]) - sin(4 * PI / 3) * (pn5[0] - b5[0]));
    N6 = 10 * (cos(5 * PI / 3) * (pn6[1] - b6[1]) - sin(5 * PI / 3) *(pn6[0] - b6[0]));

    lf1 = (asin(L1 / sqrt(pow(M1, 2) + pow(N1, 2))) - atan(N1 / M1)) * RAD_TO_DEG * 2.844;
    lf2 = (asin(L2 / sqrt(pow(M2, 2) + pow(N2, 2))) - atan(N2 / M2)) * RAD_TO_DEG * 2.844;
    lf3 = (asin(L3 / sqrt(pow(M3, 2) + pow(N3, 2))) - atan(N3 / M3)) * RAD_TO_DEG * 2.844;
    lf4 = (asin(L4 / sqrt(pow(M4, 2) + pow(N4, 2))) - atan(N4 / M4)) * RAD_TO_DEG * 2.844;
    lf5 = (asin(L5 / sqrt(pow(M5, 2) + pow(N5, 2))) - atan(N5 / M5)) * RAD_TO_DEG * 2.844;
    lf6 = (asin(L6 / sqrt(pow(M6, 2) + pow(N6, 2))) - atan(N6 / M6)) * RAD_TO_DEG * 2.844;
    if(L1 / sqrt(pow(M1, 2) + pow(N1, 2)) >= (-1) && L1 / sqrt(pow(M1, 2) + pow(N1, 2)) <= 1)
      Dynamixel.move(1, (512 - lf1));
    if(L2 / sqrt(pow(M2, 2) + pow(N2, 2)) >= (-1) && L2 / sqrt(pow(M2, 2) + pow(N2, 2)) <= 1)
      Dynamixel.move(2, (512 + lf2));
    if(L3 / sqrt(pow(M3, 2) + pow(N3, 2)) >= (-1) && L3 / sqrt(pow(M3, 2) + pow(N3, 2)) <= 1)
      Dynamixel.move(3, (512 - lf3));
    if(L4 / sqrt(pow(M4, 2) + pow(N4, 2)) >= (-1) && L4 / sqrt(pow(M4, 2) + pow(N4, 2)) <= 1)
      Dynamixel.move(4, (512 + lf4));
    if(L5 / sqrt(pow(M5, 2) + pow(N5, 2)) >= (-1) && L5 / sqrt(pow(M5, 2) + pow(N5, 2)) <= 1)
      Dynamixel.move(5, (512 - lf5));
    if(L6 / sqrt(pow(M6, 2) + pow(N6, 2)) >= (-1) && L6 / sqrt(pow(M6, 2) + pow(N6, 2)) <= 1)
      Dynamixel.move(6, (512 + lf6));
  }
}
