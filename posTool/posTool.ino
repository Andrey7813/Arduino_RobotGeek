#include <ax12.h>
#include <BioloidController.h>

//PROGMEM prog_uint16_t Home[] =   {  8, 507, 380, 644, 360, 664, 861, 205, 450};
PROGMEM prog_uint16_t Home[] =       { 8, 517, 630, 394, 401, 623, 740, 205, 450 };


BioloidController bioloid = BioloidController(1000000);

const int SERVOCOUNT = 8;

int serv1 = Home[1];
int serv2 = Home[2];
int serv3 = Home[3];
int serv4 = Home[4];
int serv5 = Home[5];
int serv6 = Home[6];
int serv7 = Home[7];
int serv8 = Home[8];

int curServ1 = 0;
int curServ2 = 0;
int curMenu = 0;

int incomingByte = 0;


void setup(){
  

   Serial.begin(9600);
   delay (500); 
   Serial.println("Arbotix-M Positioning tool V2. Andrey Teplyakov ");  
   Serial.println("###########################");    
   Serial.println("Serial Communication Established.");
   mainMenu();
   doHome();
   
}

void loop(){

  delay(100);
 
 
  if (Serial.available() > 0) {
                
    incomingByte = Serial.read();
    Serial.println(incomingByte);
     if(curMenu == 0)
     {     
        switch(incomingByte)
        {
          case '1': curServ1 = 1; curServ2 = 0;Serial.println("Current servos: ROTATION");incomingByte = 0;curMenu = 1; moveMenu(); break;
          case '2': curServ1 = 2; curServ2 = 3;Serial.println("Current servos: SHOULDER");incomingByte = 0;curMenu = 1;moveMenu();break;
          case '3': curServ1 = 4; curServ2 = 5;Serial.println("Current servos: ELBOW");incomingByte = 0;curMenu = 1;moveMenu();break;
          case '4': curServ1 = 6; curServ2 = 0;Serial.println("Current servos: WRIST");incomingByte = 0;curMenu = 1;moveMenu();break;
          case '5': curServ1 = 7; curServ2 = 0;Serial.println("Current servos: GRIPPER ROTATION");incomingByte = 0;curMenu = 1;moveMenu();break;
          case '6': curServ1 = 8; curServ2 = 0;Serial.println("Current servos: GRIPPER");incomingByte = 0;curMenu = 1; moveMenu();break;
          case '7': printPositions();break;
          case '8': doHome();break;
        }
      }
    } 
  
  if(incomingByte == '0' && curMenu != 0){mainMenu();curServ1=0; curServ2 = 0;incomingByte = 0;curMenu = 0;}
 
  if(curMenu == 1)
  {
    switch(incomingByte)
    {
      case '1': moveServo(10);incomingByte = 999; break;
    //  case '2': moveServo(50);incomingByte = 999; break;
    //  case '3': moveServo(100);incomingByte = 999; break;
      case '2': moveServo(-10);incomingByte = 999; break;
      case '3': moveServo(1);incomingByte = 999; break;
      case '4': moveServo(-1);incomingByte = 999; break;
   //   case '5': moveServo(-50);incomingByte = 999; break;
   //   case '6': moveServo(-100);incomingByte = 999; break;
    }
  }  
}

void mainMenu()
{
  curServ1 = 0;
  curServ2 = 0;
  Serial.println("###########################");
   Serial.println("Choose the servos: (0 - return to main menu)");
   Serial.println("1. Rotation");
   Serial.println("2. Shoulder");
   Serial.println("3. Elbow");
   Serial.println("4. Wrist");
   Serial.println("5. Gripper rotation");
   Serial.println("6. Gripper");
   Serial.println("7. Print servos positions");
   Serial.println("8. Home position");
   Serial.println("###########################");
}

void moveMenu()
{
   Serial.println("###########################");
   Serial.println("Choose the moving: (0 - return to main menu)");
   Serial.println("1. +10");
 //  Serial.println("2. +50");
 //  Serial.println("3. +100");
   Serial.println("2. -10");
   Serial.println("3. +1");
   Serial.println("4. -1");
//   Serial.println("5. -50");
 //  Serial.println("6. -100"); 
   Serial.println("###########################");
}

void moveServo(int m)
{
  switch(curServ1)
  {
    case 1:serv1 += m; SetPosition(1, serv1); delay(10); break;
    case 2:serv2 += m;serv3 -= m; SetPosition(2, serv2);SetPosition(3, serv3); delay(10); break;
    case 4:serv4 += m;serv5 -= m; SetPosition(4, serv4);SetPosition(5, serv5); delay(10); break;
    case 6:serv6 += m; SetPosition(6, serv6); delay(10); break;
    case 7:serv7 += m; SetPosition(7, serv7); delay(10); break;
    case 8:serv8 += m; SetPosition(8, serv8); delay(10); break;
  }
}

void printPositions()
{
   Serial.println("########Servos positions####");
   Serial.print("Servo 1: ");Serial.println(serv1);
   Serial.print("Servo 2: ");Serial.println(serv2);
   Serial.print("Servo 3: ");Serial.println(serv3);
   Serial.print("Servo 4: ");Serial.println(serv4);
   Serial.print("Servo 5: ");Serial.println(serv5);
   Serial.print("Servo 6: ");Serial.println(serv6);
   Serial.print("Servo 7: ");Serial.println(serv7);
   Serial.print("Servo 8: ");Serial.println(serv8);
   Serial.print("{ ");
   Serial.print(serv1);
   Serial.print(", ");
   Serial.print(serv2);
   Serial.print(", ");
   Serial.print(serv3);
   Serial.print(", ");
   Serial.print(serv4);
   Serial.print(", ");
   Serial.print(serv5);
   Serial.print(", ");
   Serial.print(serv6);
   Serial.print(", ");
   Serial.print(serv7);
   Serial.print(", ");
   Serial.print(serv8);
   Serial.println(" }");
   Serial.println("###########################");
}

void doHome()
{
delay(100);  
   bioloid.loadPose(Home);   // load the pose from FLASH, into the nextPose buffer
   bioloid.readPose();
   bioloid.interpolateSetup(2000); // setup for interpolation from current->next in 1 second
   while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(3);
    }
 serv1 = Home[1];
 serv2 = Home[2];
 serv3 = Home[3];
 serv4 = Home[4];
 serv5 = Home[5];
 serv6 = Home[6];
 serv7 = Home[7];
 serv8 = Home[8];
}
