#include <ax12.h>
#include <BioloidController.h>
#include <avr/pgmspace.h>

// Baud rate for connection with robot is 1Mbps = 1000000
BioloidController bioloid = BioloidController(1000000);

int WAIT_FOR_CARD_PROCESSING = 5000;

// Servos:
// 1 - Rotation (200 - right, 800 - left)
// 2,3 - Shoulder (2--, 3++) (212, 812 - full backward), (812, 212 - full forward)
// 4,5 - Elbow (2--, 3++) (212, 812 - full down), (812, 212 - full up)
// 6 - Wrist (200 - down, 800 up)
// 7 - Gripper rotation (200 - revers rotation, 700 - clockwise rotation)
// 8 - Gripper  (100 close)


PROGMEM prog_uint16_t vCardHolderRestPosition[] =  { 8, 507, 380, 644, 360, 664, 461, 205, 100 }; //507

PROGMEM prog_uint16_t step1_1[] =                  { 8, 510, 553, 471, 313, 711, 782, 205, 450 }; //782
PROGMEM prog_uint16_t step2_1[] =                  { 8, 656, 553, 471, 313, 711, 782, 205, 450 };
PROGMEM prog_uint16_t step3_1[] =                  { 8, 656, 553, 471, 313, 711, 782, 205, 100 };
PROGMEM prog_uint16_t step4_1[] =                  { 8, 537, 553, 471, 313, 711, 782, 205, 100 };

PROGMEM prog_uint16_t step1_2[] =                  { 8, 500, 465, 559, 316, 708, 695, 205, 450 }; //592
PROGMEM prog_uint16_t step2_2[] =                  { 8, 656, 465, 559, 316, 708, 695, 205, 450 };
PROGMEM prog_uint16_t step3_2[] =                  { 8, 656, 465, 559, 316, 708, 695, 205, 100 };
PROGMEM prog_uint16_t step4_2[] =                  { 8, 527, 465, 559, 316, 708, 695, 205, 100 };

PROGMEM prog_uint16_t step1_3[] =                  { 8, 500, 417, 607, 358, 666, 594, 205, 450 }; //592
PROGMEM prog_uint16_t step2_3[] =                  { 8, 657, 417, 607, 358, 666, 594, 205, 450 };
PROGMEM prog_uint16_t step3_3[] =                  { 8, 657, 417, 607, 358, 666, 594, 205, 50 };
PROGMEM prog_uint16_t step4_3[] =                  { 8, 527, 417, 607, 358, 666, 594, 205, 50 };

PROGMEM prog_uint16_t step1_4[] =                  { 8, 500, 402, 623, 412, 612, 529, 205, 450 }; //592
PROGMEM prog_uint16_t step2_4[] =                  { 8, 657, 402, 623, 412, 612, 529, 205, 450 };
PROGMEM prog_uint16_t step3_4[] =                  { 8, 657, 402, 623, 412, 612, 529, 205, 100 };
PROGMEM prog_uint16_t step4_4[] =                  { 8, 527, 402, 623, 412, 612, 529, 205, 100 };

PROGMEM prog_uint16_t step1_5[] =                  { 8, 500, 410, 614, 485, 538, 455, 205, 450 }; //459
PROGMEM prog_uint16_t step2_5[] =                  { 8, 657, 410, 614, 485, 538, 455, 205, 450 };
PROGMEM prog_uint16_t step3_5[] =                  { 8, 657, 410, 614, 485, 538, 455, 205, 100 };
PROGMEM prog_uint16_t step4_5[] =                  { 8, 527, 410, 614, 485, 538, 455, 205, 100 };

PROGMEM prog_uint16_t step1_6[] =                  { 8, 535, 438, 586, 581, 443, 400, 205, 450 }; //500
PROGMEM prog_uint16_t step2_6[] =                  { 8, 657, 438, 586, 581, 443, 400, 205, 450 };
PROGMEM prog_uint16_t step3_6[] =                  { 8, 657, 438, 586, 581, 443, 400, 205, 100 };
PROGMEM prog_uint16_t step4_6[] =                  { 8, 535, 438, 586, 581, 443, 400, 205, 100 };//527

PROGMEM prog_uint16_t iStep1[] =                  { 8, 517, 620, 404, 311, 713, 865, 205, 50 };//517
PROGMEM prog_uint16_t iStep2[] =                  { 8, 517, 655, 369, 426, 598, 750, 205, 50 };//{ 8, 517, 655, 369, 426, 598, 750, 205, 50 }
PROGMEM prog_uint16_t iStep3[] =                  { 8, 517, 650, 374, 436, 588, 760, 205, 50 };//745
PROGMEM prog_uint16_t iStep4_initial[] =          { 8, 517, 585, 439, 371, 653, 720, 205, 450 };//746
PROGMEM prog_uint16_t iStep4[] =                  { 8, 517, 650, 374, 436, 588, 760, 205, 450 };//746
PROGMEM prog_uint16_t iStep5[] =                  { 8, 517, 640, 384, 411, 613, 760, 205, 450 }; // { 8, 517, 625, 399, 396, 628, 737, 205, 450 };
PROGMEM prog_uint16_t iStep6[] =                  { 8, 517, 640, 384, 411, 613, 760, 205, 50 };

PROGMEM prog_uint16_t wStep1[] =                  { 8, 517, 770, 254, 811, 213, 520, 205, 50 };

int curMenu = 0;
int repeats = 1;



void setup()
{
  delay(200);
  Serial.begin(9600);
  
  mainMenu();

}  

void move_insert()
{
  doGesture(iStep1, 800);
  delay(1000);
  doGesture(iStep2, 2000);
  doGesture(iStep3, 100);
  doGesture(iStep4, 1000);
  doGesture(iStep5, 1000);
  doGesture(iStep6, 1000);
}

void move_insert_initial()
{
  doGesture(iStep4_initial, 1000);
  doGesture(iStep4, 1000);
  doGesture(iStep5, 1000);
  doGesture(iStep6, 1000);
}

void move_insert_invert()
{
  doGesture(iStep2, 800);
  doGesture(iStep1, 800);
}

void move5_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_5, 800);
  doGesture(step2_5, 800);
  delay(1000);
  doGesture(step3_5, 800);
  doGesture(step4_5, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move5_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_5, 800);
  doGesture(step3_5, 800);
  delay(1000);
  doGesture(step2_5, 800);
  doGesture(step1_5, 800);
  doGesture(vCardHolderRestPosition, 800);
}

void move4_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_4, 800);
  doGesture(step2_4, 800);
  delay(1000);
  doGesture(step3_4, 800);
  doGesture(step4_4, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move4_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_4, 800);
  doGesture(step3_4, 800);
  delay(1000);
  doGesture(step2_4, 800);
  doGesture(step1_4, 800);
  doGesture(vCardHolderRestPosition, 800);
}

void move6_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_6, 800);
  doGesture(step2_6, 800);
  delay(1000);
  doGesture(step3_6, 800);
  doGesture(step4_6, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move6_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_6, 800);
  doGesture(step3_6, 800);
  delay(1000);
  doGesture(step2_6, 800);
  doGesture(step1_6, 800);
  doGesture(vCardHolderRestPosition, 800);
}

void move3_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_3, 800);
  doGesture(step2_3, 800);
  delay(1000);
  doGesture(step3_3, 800);
  doGesture(step4_3, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move3_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_3, 800);
  doGesture(step3_3, 800);
  delay(1000);
  doGesture(step2_3, 800);
  doGesture(step1_3, 800);
  doGesture(vCardHolderRestPosition, 800);
}

void move1_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_1, 800);
  doGesture(step2_1, 800);
  delay(1000);
  doGesture(step3_1, 800);
  doGesture(step4_1, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move1_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_1, 800);
  doGesture(step3_1, 800);
  delay(1000);
  doGesture(step2_1, 800);
  doGesture(step1_1, 800);
  doGesture(vCardHolderRestPosition, 800);
}

void move2_take()
{
  
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step1_2, 800);
  doGesture(step2_2, 800);
  delay(1000);
  doGesture(step3_2, 800);
  doGesture(step4_2, 800);
  doGesture(vCardHolderRestPosition, 800);
  
}

void move2_return()
{
  doGesture(vCardHolderRestPosition, 800);
  doGesture(step4_2, 800);
  doGesture(step3_2, 800);
  delay(1000);
  doGesture(step2_2, 800);
  doGesture(step1_2, 800);
  doGesture(vCardHolderRestPosition, 800);
}




void loop()
{
  int incomingByte = 0;
  
   delay(100);
   if (Serial.available() > 0) {
                
    incomingByte = Serial.read();
    
    if(curMenu == 0 && incomingByte != 10)
    {
      switch(incomingByte)
      {
        case '1': doInsertion(1); curMenu = 0;  incomingByte = 10;  break;
        case '2': doInsertion(2); curMenu = 0;  incomingByte = 10;  break;
        case '3': doInsertion(3); curMenu = 0;  incomingByte = 10;  break;
        case '4': doInsertion(4); curMenu = 0;  incomingByte = 10;  break;
        case '5': doInsertion(5); curMenu = 0;  incomingByte = 10;  break;
        case '6': doInsertion(6); curMenu = 0;  incomingByte = 10;  break;
        case '7': initSlot1(); curMenu = 0; incomingByte = 10; break;
        case '8': initSlot2(); curMenu = 0; incomingByte = 10; break;
        case '9': initSlot3(); curMenu = 0; incomingByte = 10; break;
        case 'a': initSlot4(); curMenu = 0; incomingByte = 10; break;
        case 'b': initSlot5(); curMenu = 0; incomingByte = 10; break;
        case 'c': initSlot6(); curMenu = 0; incomingByte = 10; break;
        case 'd': demo(); curMenu = 0; incomingByte = 10;  break;
        
      }
    }

      mainMenu();
   
   }
}

void doInsertion(int slotNumber)
{
  
  switch(slotNumber)
  {
    case 1: insertFromSlot1(); break;
    case 2: insertFromSlot2(); break;
    case 3: insertFromSlot3(); break;
    case 4: insertFromSlot4(); break;
    case 5: insertFromSlot5(); break;
    case 6: insertFromSlot6(); break;
  }
   
}

void insertFromSlot1()
{
  
  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move1_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move1_return();
    doGesture(vCardHolderRestPosition, 1000);
  }
   
}

void insertFromSlot2()
{
  
  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move2_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move2_return();
    doGesture(vCardHolderRestPosition, 1000);
  }

}

void insertFromSlot3()
{
  
  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move3_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move3_return();
    doGesture(vCardHolderRestPosition, 1000);
  }

}

void insertFromSlot4()
{
  
  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move4_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move4_return();
    doGesture(vCardHolderRestPosition, 1000);
  }

}

void insertFromSlot5()
{
  
  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move5_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move5_return();
    doGesture(vCardHolderRestPosition, 1000);
  }

}

void insertFromSlot6()
{
        

  for(int i = 1; i <= repeats; i++)
  {
    doGesture(vCardHolderRestPosition, 1000);
    move6_take();
    move_insert();
    delay(WAIT_FOR_CARD_PROCESSING);
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move6_return();
    doGesture(vCardHolderRestPosition, 1000);
  }

}

void initSlot1()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move1_return();
    doGesture(vCardHolderRestPosition, 1000);

}

void initSlot2()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move2_return();
    doGesture(vCardHolderRestPosition, 1000);

}

void initSlot3()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move3_return();
    doGesture(vCardHolderRestPosition, 1000);

}

void initSlot4()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move4_return();
    doGesture(vCardHolderRestPosition, 1000);

}

void initSlot5()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move5_return();
    doGesture(vCardHolderRestPosition, 1000);

}

void initSlot6()
{
    move_insert_initial();
    move_insert_invert();
    doGesture(vCardHolderRestPosition, 1000);
    move6_return();
    doGesture(vCardHolderRestPosition, 1000);

}


void demo()
{
  
  for(int i = 1; i <= 1; i++)
 {
  doGesture(vCardHolderRestPosition, 1000);
  move6_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move1_return();
  
  doGesture(vCardHolderRestPosition, 1000);
  move1_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move5_return();
  
  doGesture(vCardHolderRestPosition, 1000);
  move5_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move2_return();
  
  doGesture(vCardHolderRestPosition, 1000);
  move2_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move4_return();
  
  doGesture(vCardHolderRestPosition, 1000);
  move4_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move3_return();
  
  doGesture(vCardHolderRestPosition, 1000);
  move3_take();
  move_insert();
  delay(WAIT_FOR_CARD_PROCESSING);
  move_insert_invert();
  doGesture(vCardHolderRestPosition, 1000);
  move6_return();}

}

void mainMenu()
{
  
   Serial.println("###########################");
   Serial.println("Hey, I'm BENDER! What do you want, dude?");
   Serial.println("1. Insert card from slot 1");
   Serial.println("2. Insert card from slot 2");
   Serial.println("3. Insert card from slot 3");
   Serial.println("4. Insert card from slot 4");
   Serial.println("5. Insert card from slot 5");
   Serial.println("6. Insert card from slot 6");
   Serial.println("--------------------------");
   Serial.println("7. Initially put the card into slot #1");
   Serial.println("8. Initially put the card into slot #2");
   Serial.println("9. Initially put the card into slot #3");
   Serial.println("a. Initially put the card into slot #4");
   Serial.println("b. Initially put the card into slot #5");
   Serial.println("c. Initially put the card into slot #6");
   Serial.println("d. Run demo");
   Serial.println("###########################");
}

void scndMenu()
{
  Serial.println("###########################");
  Serial.println("Work! Oh, no! How many times?");
  Serial.println("1. 1");
  Serial.println("2. 2");
  Serial.println("3. 5");
  Serial.println("###########################");
}

void doGesture(const unsigned int* gesture, int speed_)
{
   delay(100);  
   bioloid.loadPose(gesture);   // load the pose from FLASH, into the nextPose buffer
   bioloid.readPose();
   bioloid.interpolateSetup(speed_); // setup for interpolation from current->next in 1 second
   while(bioloid.interpolating > 0){  // do this while we have not reached our new pose
        bioloid.interpolateStep();     // move servos, if necessary. 
        delay(10);
    }
}


