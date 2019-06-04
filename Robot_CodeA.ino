#include <ArduinoRobot.h>
#include <Wire.h>

//EXPERIMENT 1

/* Input/Output
 * 
 * Sensor               board_name    code_name    variable
 * 
 * gas Left             D2 top        TKD2         baseLx senseL
 * gas right            D0 top        TKD0         baseLx senseL
 * light sens           M2 top        TK2          lightSensor
 * crash sens left      D9 base       B_TK2        left_push
 * crash sens right     D8 base       B_TK4        right_push
 * 
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
  Serial.println("Ready");
  //Robot.stroke(0, 0, 0); 
  //Robot.text("Ready", 45, 20);
  //delay(500)
  //Robot.text("Ready", 45, 20);
  
}

/*
void loop() {
  Robot.text("Ready", 45, 20);
  Serial.println("Ready");
  delay(2000)
}
*/
    
void loop() {  //The MAIN LOOP
  keyDown(Robot.keyboardRead());  //use function keyDown to read button state
  if (button == true){ //if button is presses run movement function
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
  else{                           //if not pressed check again
  }
}


void keyDown(int keyCode){     //function for button state
  static int oldKey;          //static value for old key
  switch (keyCode){           //change button value if pressed
    case BUTTON_MIDDLE:
      button = true;
  } 
}


void sensorBaseline(){

    baseR1 = Robot.analogRead(TKD0); //AVG over 4 readings
    delay(250);
    baseR2 = Robot.analogRead(TKD0);
    delay(250);
    baseR3 = Robot.analogRead(TKD0);
    delay(250);
    baseR4 = Robot.analogRead(TKD0);
    baseR = (baseR1+baseR2+baseR3+baseR4)/4;   
    baseL1 = Robot.analogRead(TKD2);
    delay(250);
    baseL2 = Robot.analogRead(TKD2);
    delay(250);
    baseL3 = Robot.analogRead(TKD2);
    delay(250);
    baseL4 = Robot.analogRead(TKD2);
    baseL  = (baseL1+baseL2+baseL3+baseL4)/4;
    Serial.print("BaseOdors: Left: ");
    Serial.println(baseL);
    Serial.print("BaseOdors: Right: ");
    Serial.println(baseR);
}

/*
void irsensors(){
    baseirR = Robot.analogRead(TK0);
    baseirL = Robot.analogRead(TK4);
}
*/


void Movement(){ //function that actually puts sensors to use

  int lightVal = Robot.analogRead(lightSensor);
  counter = 0;
  while (lightVal < 950){ //while loop to move based on light value   was 700
  
    IRRead();//calls IRRead function: CRASH SENSOR
   
    if(irRight1 == true){
    Serial.println("right IR");
    Robot.motorsWrite(-Speed,Speed);
    delay(100);
    Robot.motorsStop();
    delay(1);
    Robot.motorsWrite(Speed,Speed);
    delay(200);
    Robot.motorsStop();
   }
  
    else if(irLeft1 == true){
    Serial.println("left IR");
    Robot.motorsWrite(Speed,-Speed);
    delay(100);
    Robot.motorsStop();
    delay(1);
    Robot.motorsWrite(Speed,Speed);
    delay(200);
    Robot.motorsStop();
   }
   
    else if(irMid1 == true){ //was irNone1
    Serial.println("Mid IR");  //was both IR's
    Robot.motorsWrite(Speed,-Speed); 
    delay(100);
    Robot.motorsStop();
    delay(1);
    Robot.motorsWrite(-Speed,-Speed); //go backwards
    delay(350); //was 200
    Robot.motorsStop();
   }
   //else { // NO CRASH IMMINENT
   // }
    

         
    OdorRead();//calls OdorRead function
    if (DirRight == true){ //if right is true, turn 30 degrees and move forward
      Serial.println("right");
      Robot.motorsWrite(Speed,-Speed);
      delay(100);
      Robot.motorsStop();
      delay(1);
      Robot.motorsWrite(Speed,Speed);
      delay(200);
      Robot.motorsStop();
    } 
    
    else if (None == true){ //if mid is true move forward
        Serial.println("mid");
        Robot.motorsWrite(120,120);  //speed+speed
        delay(100);
        Robot.motorsStop();   
        Serial.println("mid2");  
    } 
    
    else if (DirLeft == true){ //if left id true turn -30 degrees and move forward
          Serial.println("left");
          Robot.motorsWrite(-Speed,Speed);
          delay(100);
          Robot.motorsStop();
          delay(1);
          Robot.motorsWrite(Speed,Speed);
          delay(200);
          Robot.motorsStop();
     } 
     
    else { //if all values equal to one another do nothing
            Serial.println("none");
    }

    Serial.println("counter" + counter);
    lightVal = Robot.analogRead(lightSensor);
    //Serial.println(lightVal);
  }
}

/*
void Face_Upstream(){
  int Sense = 0;
  while (Sense == 0){
    SenseR = Robot.analogRead(TKD0);
    SenseL = Robot.analogRead(TKD2);
    if (SenseR > 8 || SenseL > 8) { //WHY HAVE THIS HERE?
      do {
        compass = Robot.compassRead();
        Robot.motorsWrite(150,-150);
      //Robot.motorsWrite(40,-40);
        compass = Robot.compassRead();
        Serial.println(compass);
      //} while (compass != 0);
      } while (compass < 350 &&+ compass > 10); //ALLOW ERROR SO TO FIND ROBUSTLY
      Robot.motorsStop();
      Sense = 1;
    }
    else {
      Robot.motorsWrite(100,100);
      delay(300);
      Robot.motorsStop();
   }
  } 
 }

*/

void OdorRead(){  //function which reads and calculates sensor data 
  DirRight = false;
  None = false;
  DirLeft = false;
                   
  int senseR = Robot.analogRead(TKD0); //read inital value from sensors
  int senseL = Robot.analogRead(TKD2);
  int RealR = (senseR - baseR)/1.5 ;  //calculate max to see change in value, fixes issue or power distribution and inital settings; NOT ABS; WAS 2.2
  int RealL = (senseL - baseL);
       
  RealR = constrain(RealR, 0, 1000);
  RealL = constrain(RealL, 0, 1000);
 
  Serial.print(RealR);  //prints values if connected to computer
  Serial.print(',');
  Serial.println(RealL);
  Robot.stroke(0, 0, 0); //write to screen the values
  Robot.text(RealL, 5, 0);
  Robot.text(RealR, 110, 0);
  
  Right = RealR - RealL;
  Left = RealL - RealR;
  if (Right > 20){ //sets state of direction 
    DirRight = true;
    Robot.stroke(0, 0, 0);  //write to screen the direction
    Robot.text("Right", 50, 20);
  } 
  else if (Left > 20){
    DirLeft = true;
    Robot.stroke(0, 0, 0);
    Robot.text("Left", 50, 20);
  } 
  else { //null statement
    None = true;
    Robot.stroke(0, 0, 0);
    Robot.text("Middle", 45, 20);
  }
  delay(300); //WAS 2000  
  Robot.stroke(255, 255, 255); //clear screen
  Robot.text(RealL, 5, 0);
  Robot.text(RealR, 110, 0);
  Robot.text("Right", 50, 20);
  Robot.text("Middle", 45, 20);
  Robot.text("Left", 50, 20);

}




void IRRead(){  //function which reads and calculates sensor data 
  irRight1 = false;
  irMid1 = false;
  irLeft1 = false;
  irOK1 = false;
           
  int irRight = Robot.analogRead (TK0);  //calculates the minimum # for activation... IR right
  int irLeft = Robot.analogRead (TK4);    //calculates the minimum # for activation... IR left
  int irMid = Robot.analogRead (TK6);
  Serial.println(irLeft);
  Serial.println(irRight);
  Serial.println(irMid);
  
    if ((irMid >= irThresH) && (irRight < irThresH) && (irLeft < irThresH)){ 
      irMid1 = true;
    }
    else if ((irRight >= irThresH) && (irLeft < irThresH) && (irMid1 < irThresH)){ 
      irRight1 = true;
    }
    else if ((irRight < irThresH) && (irLeft >= irThresH) && (irMid1 < irThresH)){ 
      irLeft1 = true;
    }
    else if ((irRight < irThresH) && (irLeft < irThresH ) && (irMid1 < irThresH)){ 
      irOK1 = true;
    }
}

 
/* 
//void reorient(){
//  compass = Robot.compassRead();
//  if (compass >= 180){
//    Robot.motorsWrite(100,-100);
//    delay(600);
//    Robot.motorsStop();
//  }
//  else{
//    Robot.motorsWrite(-100,100);
//    delay(600);
//    Robot.motorsStop();
//  }
//}


void reorientL(){ //turn left to move cross-stream
    orientL = 1;
    orientR= 0;
      do {
        Robot.motorsWrite(150,-150);
        delay(200);
        compass = Robot.compassRead();
        Serial.println(compass);
      } while (compass < 100 &&+ compass > 80); //ALLOW ERROR SO TO FIND ROBUSTLY
} 

 void reorientR(){ //turn right to move cross-stream
    orientL = 0;
    orientR= 1;
      do {
        Robot.motorsWrite(-150,150);
        delay(200);
        compass = Robot.compassRead();
        Serial.println(compass);
      } while (compass < 280 &&+ compass > 260); //ALLOW ERROR SO TO FIND ROBUSTLY
}


*/


