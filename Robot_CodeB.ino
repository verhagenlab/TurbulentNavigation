#include <ArduinoRobot.h>
#include <Wire.h>

/* EXPERIMENT C2
  if L+R increases then keep going forward
  if not do as EXPT A: go L or R if either is stronger, or forward if similar
*/

/* Input/Output

   Sensor               board_name    code_name    variable

   gas Left             D2 top        TKD2         baseLx senseL
   gas right            D0 top        TKD0         baseLx senseL
   light sens           M2 top        TK2          lightSensor
   crash sens left      D9 base       B_TK2        left_push
   crash sens right     D8 base       B_TK4        right_push

*/


int button = false; //delare middle button
int DirRight = false; //declare direcrions and null
int DirMid = false;
int DirLeft = false;
int None = false;
int forward = false;
int SenseR; int SenseL;
int compass;
int lightSensor = TK2;
int baseR; int baseR1; int baseR2; int baseR3; int baseR4;
int baseL; int baseL1; int baseL2; int baseL3; int baseL4;
int baseirR; int baseirR1; int baseirR2; int baseirR3; int baseirR4;  //for IR sensors right
int baseirL; int baseirL1; int baseirL2; int baseirL3; int baseirL4;   // for IR sensors left
int Speed = 100;
int counter;
int Right;
int Left;
int LR1 = 0; //new sum LR
int LR2 = 0; //old sum LR
int dLR = 0;
int orientL = 0; int orientR = 0;
int across_counter = 0;
int RealL_prior; int RealR_prior; //prior odor conc; not needed if runs as-is
int irRight = TK0;  //IR sensor rght
int irLeft = TK4;   //IR sensor left
int irMed = TK6;
int irThresH = 180; //was 370 for 5cm.
bool irRight1; bool irMid1; bool irLeft1; bool irOK1;  //irNone1=irMid1
//int irRight1=false; int irNone1=false; int irLeft1=false; int irOK1=false;

void setup() {    //start communitcation
  Robot.begin();
  Robot.beginTFT();
  Serial.begin(9600);
  Serial.println("Ready, CODE C2");
  //Robot.stroke(0, 0, 0);
  //Robot.text("Ready", 45, 20);
  //delay(500)
  //Robot.text("Ready", 45, 20);
}


void loop() {  //The MAIN LOOP
  keyDown(Robot.keyboardRead());  //use function keyDown to read button state
  if (button == true) { //if button is presses run movement function
    Serial.println("Start");
    //Robot.stroke(0, 0, 0);
    //Robot.text("Start", 45, 20);
    delay(20000);   //GC it was 8 sec.
    //Face_Upstream();  //turn upstream
    sensorBaseline(); //get sensor base odor concentration
    //irsensors();  //get the IR sensors to work
    Movement();       //navigate
    button = false;
  }
  else {                          //if not pressed check again
  }
}


void keyDown(int keyCode) {    //function for button state
  static int oldKey;          //static value for old key
  switch (keyCode) {          //change button value if pressed
    case BUTTON_MIDDLE:
      button = true;
  }
}


void sensorBaseline() {

  baseR1 = Robot.analogRead(TKD0); //AVG over 4 readings No 1; right sensor
  delay(250);
  baseR2 = Robot.analogRead(TKD0);
  delay(250);
  baseR3 = Robot.analogRead(TKD0);
  delay(250);
  baseR4 = Robot.analogRead(TKD0);
  baseR = (baseR1 + baseR2 + baseR3 + baseR4) / 4;

  baseL1 = Robot.analogRead(TKD2);   // AVG over 4 readings No 1; left sensor
  delay(250);
  baseL2 = Robot.analogRead(TKD2);
  delay(250);
  baseL3 = Robot.analogRead(TKD2);
  delay(250);
  baseL4 = Robot.analogRead(TKD2);
  baseL  = (baseL1 + baseL2 + baseL3 + baseL4) / 4;

  Serial.print("BaseOdors: Left: ");
  Serial.print(baseL);
  Serial.print("; BaseOdors: Right: ");
  Serial.println(baseR);
}


void Movement() { //function that actually puts sensors to use


  int lightVal = Robot.analogRead(lightSensor);
  counter = 0;
  while (lightVal < 950) { //while loop to move based on light value   was 700

    IRRead();//calls IRRead function: CRASH SENSOR

    if (irRight1 == true) {
      Serial.println("right IR");
      Robot.motorsWrite(-Speed, Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(Speed, Speed);
      delay(200);
      Robot.motorsStop();
    }

    else if (irLeft1 == true) {
      Serial.println("left IR");
      Robot.motorsWrite(Speed, -Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(Speed, Speed);
      delay(200);
      Robot.motorsStop();
    }

    else if (irMid1 == true) { //was irNone1
      Serial.println("Mid IR");  //was both IR's
      Robot.motorsWrite(Speed, -Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(-Speed, -Speed); //go backwards
      delay(350); //was 200
      Robot.motorsStop();
    }
    //else { // NO CRASH IMMINENT
    // }



    OdorRead();//calls OdorRead function
    if (DirRight == true) { //if right is true, turn 30 degrees and move forward
      Serial.println("right");
      Robot.motorsWrite(Speed, -Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(Speed, Speed);
      delay(200);
      Robot.motorsStop();
    }

    else if (None == true) { //if mid is true move forward
      Serial.println("mid");
      Robot.motorsWrite(120, 120); //speed+speed
      delay(100);
      Robot.motorsStop();
      //Serial.println("mid2");
    }

    else if (DirLeft == true) { //if left id true turn -30 degrees and move forward
      Serial.println("left");
      Robot.motorsWrite(-Speed, Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(Speed, Speed);
      delay(200);
      Robot.motorsStop();
    }

    else { //if all values equal to one another do nothing
      Serial.println("none");
    }

    Serial.print("counter: ");
    Serial.println(counter);
    
    lightVal = Robot.analogRead(lightSensor);
    //Serial.println(lightVal);
  }
}



void OdorRead() { //function which reads and calculates sensor data
  DirRight = false;
  None = false;
  DirLeft = false;

  int senseR = Robot.analogRead(TKD0); //read inital value from sensors
  int senseL = Robot.analogRead(TKD2);
  int RealR = (senseR - baseR) / 1.5 ; //calculate max to see change in value, fixes issue or power distribution and inital settings; NOT ABS; WAS 2.2
  int RealL = (senseL - baseL);

  RealR = constrain(RealR, 0, 1000);
  RealL = constrain(RealL, 0, 1000);

  LR2 = (RealR + RealL) / 2; //EXPT C2
  dLR = LR2 - LR1; //change in mono odor

  Right = RealR - RealL; //FOR EXP C
  Left = RealL - RealR;

  Serial.print("RealR");
  Serial.print(':');
  Serial.print(RealR);
  Serial.print(',');
  Serial.print("RealL");
  Serial.print(':');
  Serial.println(RealL);

  //  Serial.print(RealR);  //prints values if connected to computer
  //  Serial.print(',');
  //  Serial.println(RealL);

  Robot.stroke(0, 0, 0); //write to screen the values
  Robot.text(RealL, 5, 0);
  Robot.text(RealR, 110, 0);

  if (dLR > 5) { // if increase go fwd
    None = true;
    Robot.stroke(0, 0, 0);
    Robot.text("UP GRADIENT", 45, 20);
  }

  else if (Right > 10) { //sets state of direction
    DirRight = true;
    Robot.stroke(0, 0, 0);  //write to screen the direction
    Robot.text("Right", 50, 20);
  }

  else if (Left > 10) {
    DirLeft = true;
    Robot.stroke(0, 0, 0);
    Robot.text("Left", 50, 20);
  }
  else { //null statement
    None = true;
    Robot.stroke(0, 0, 0);
    Robot.text("Middle", 45, 20);
  }

  LR1 = LR2; //save old odor conc

  delay(300); //WAS 2000

  Robot.stroke(255, 255, 255); //clear screen of following pixels:
  Robot.text(RealL, 5, 0);
  Robot.text(RealR, 110, 0);
  Robot.text("Right", 50, 20);
  Robot.text("Middle", 45, 20);
  Robot.text("Left", 50, 20);
  Robot.text("UP GRADIENT", 45, 20);

}


void IRRead() { //function which reads and calculates sensor data
  irRight1 = false;
  irMid1 = false;
  irLeft1 = false;
  irOK1 = false;

  int irRight = Robot.analogRead (TK0);  //calculates the minimum # for activation... IR right
  int irLeft = Robot.analogRead (TK4);    //calculates the minimum # for activation... IR left
  int irMid = Robot.analogRead (TK6);
  
  Serial.print("irLeft: ");
  Serial.print(irLeft);
  Serial.print(" ;irRight: ");
  Serial.print(irRight);
  Serial.print("; irMid: ");
  Serial.println(irMid);

  if ((irMid >= irThresH) && (irRight < irThresH) && (irLeft < irThresH)) {
    irMid1 = true;
  }
  else if ((irRight >= irThresH) && (irLeft < irThresH) && (irMid1 < irThresH)) {
    irRight1 = true;
  }
  else if ((irRight < irThresH) && (irLeft >= irThresH) && (irMid1 < irThresH)) {
    irLeft1 = true;
  }
  else if ((irRight < irThresH) && (irLeft < irThresH ) && (irMid1 < irThresH)) {
    irOK1 = true;
  }
}


