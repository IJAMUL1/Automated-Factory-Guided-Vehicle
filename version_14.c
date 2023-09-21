#include "simpletools.h" // Include simpletools
#include "servo.h" // Include servo library
#include "ping.h"  // Include Ultrasonic Sensor
serial *lcd;   // Connect to LCD

void forward(void);  //forward declaration of function to move robot forward at fastest speed
void forward_slow(void);//forward declaration of function to move robot forward at slower speed
void stopmotors(void); //forward declaration of function to stop robot
void turnright(void); //forward declaration of function to turn robot
void turnleft(void);  //forward declaration of function to turn robot left
void spinleft(void);  //forward declaration of function to spin robot left
void spinright(void); //forward declaration of function to spin robot right
void linefollow(void);//forward declaration of function to enable robot line follow at fastest speed
void linefollow_slow(void); //forward declaration of function to enable robot line follow at slower speed
void calibrate(void *par1); //forward declaration of function to calibrate the Pulolo reflectance sensor
void pingfromcog(void *par2); //forward declaration of function to read obstacle in front of robot
void pingfromcog2(void *par3);//forward declaration of function to read obstacle at the left of robot
void led(void *par4); //forward declaration of function to blink the different LEDs


static volatile int blink; //variable that controlls the led we blink
volatile int cmdist; // distance reading for front ultrasonic sensor
volatile int cmdist2;// distance reading for left ultrasonic sensor
volatile int position, a; //position helps tell us where the black line is during line follow, while a helps us control when to calibrate and when to read position values
volatile int intersection; //this helps us determine when we are at an intersection
volatile float scaled_sensor_val[8]; //stores the final reading of sensor values
unsigned int stack4[40 + 25]; // controls the first cog
unsigned int stack1[40 + 95]; // controls the second cog
unsigned int stack2[40 + 25]; //controls the third cog
unsigned int stack3[40 + 25]; //controls the 4th cog

int distance; //variable to determine total distance travelled between dropoff and pick up
int dx; // variable to control a counter 
int movecount; // variable to control the number of loops the go into during motor actuation
int maininter; // counter for the center lane
int mainobstacle; // obstacle variable for the main lane
int binter; // variable for b lane during delivery
int bcounter; // counter for b lane during delivery
int acounter; // counter for a lane during delivery
int ainter; // variable for a lane 
int obstacleb; // variable for b lane during obstacle avoidance 
const int ON = 22; // turn on LCD
const int CLR = 12; // clear LCD screen

volatile int sensor_mean;  // intersection variable


int main() // main function
{ 
  a = 1; // begin calibration
  cogstart((void*)calibrate, NULL, stack1, sizeof(stack1));// start up cog 1 and run calibrate function
  pause(2000); // wait 2 seconds
  
  
  movecount = 30;
  spinright(); // spin ring
  movecount = 60;
  spinleft(); // spin left
  movecount = 30;
  spinright(); // spin right
  stopmotors();  //stop motors
  a=2; // stop calibration and just keep reading position value
 
 //initial declaration of counters and other variables
  maininter =0;
  binter = 0;
  ainter = 0;
  mainobstacle =0;
  int bcounter =0;
  int acounter =0;
  int obstacleb =0;
  dx =0;
  distance =0;
  
  //start ultrasonic cogs (2 and 3)
  cogstart((void*)pingfromcog, NULL, stack2, sizeof(stack2));  // start second cog
  cogstart((void*)pingfromcog2, NULL, stack3, sizeof(stack3)); // start third cog
  
  //start led cog
  blink =0; // variable to turn off all leds
  cogstart((void*)led, NULL, stack4, sizeof(stack4)); // start cog 4 and run led function
  
  lcd = serial_open(12, 12, 0, 9600);  // start lcd
  writeChar(lcd, ON); // ON
  writeChar(lcd, CLR); //Clear screen
pause(5); // wait for 0.005 seconds
   
  while(1){
   
    //line follow function on the center lane. if an obstacle is seen in front of robot, go into the B obstacle avoidance line following sequence 
    if(sensor_mean <300&&maininter >=0&&binter ==0 && ainter ==0){      
        
      do{
        
      if(cmdist==1 && maininter <4){ // if we arent at the end of main lane and an obstacle is seen
        linefollow();   // follow the line
        blink = 3; // blink both Leds when an obstacle is seen 
        mainobstacle = 1;  // indicate that an obstacle is seen
        binter =1; // enter obstacle avoidance sequence 
      }
      else{
       
       linefollow();
       }
          
        }while(sensor_mean <300 && binter ==0);    
    }
    
    // avoid obstacle by going into the b lane then go back into the center lane. when robot gets back to center lane check i an obstcale is in front
    // if an obstacle is in front, go back to avoid obstcale and return back to center lane
    else if(sensor_mean <300&&maininter >=0&&binter ==1&& ainter ==0&&(obstacleb == 1 ||obstacleb == 2||obstacleb ==3||obstacleb ==4)){      
        
      do{
        
      if (cmdist ==1 && obstacleb == 2){
        movecount = 20;
        blink = 3; // blink both Leds when an obstacle is seen 
        stopmotors(); //stop motors
      }          
      else if(obstacleb ==2){
        linefollow_slow(); //line follow slowly     
       }
      else if(cmdist ==0 && obstacleb ==4){
      binter =0;
      obstacleb = 0;
      maininter +=1;
      mainobstacle =0;        
        }
      else if(cmdist ==1 &&obstacleb ==4){
       
      movecount = 20;
      stopmotors();  
      pause(50);       
      spinleft();
      pause(170);
      obstacleb = 1;
      maininter +=1;
        
        }
      else{ linefollow();
      }     
          
        }while(sensor_mean <300 && binter ==1);    
    }
   
   //sequence to control line follow process while on the pickup lane. if an obstacle is seen at a pick up location, pause for 2seconds to pickup
    else if(sensor_mean <300 && maininter >=0 && binter == 0 && ainter ==1 &&(acounter>=0)){      
        
      do{
       
      if(acounter ==0){
        linefollow();
        }
      else if (acounter>=1 && cmdist ==1){
        movecount = 20;
        blink = 3; // blink both Leds when an obstacle is seen 
        stopmotors();
      } 
      else if(acounter >=1){
        linefollow_slow();      
       }
         
        }while(sensor_mean <300 && binter ==0 && ainter ==1);
      } 
     
     
  //sequence to control line follow process while robot is in delivery sequence 
  //check for obstacle at the drop off points. if an obstacle is seen, stop robot and display distance travelled  
  else if(sensor_mean <300 && binter == 2){      
        
        do{
      if(bcounter >=3 && cmdist ==1){
        movecount = 20;
        blink =3; // blink both Leds when an obstacle is seen 
        stopmotors();  
        }
      else if(bcounter>=3){
        linefollow_slow();
        }
      else{
        linefollow();
        }          
     }while(sensor_mean <300 && binter ==2); 
     
     }
     
   
    
  // Sequence to control behaviour while at intersections
  else if(sensor_mean>300)
    { 
    if(mainobstacle ==1 &&(obstacleb==0)){// turn left so the robot approaches the b lane once an obstacle is seen at the front of the robot
      maininter +=1;
      obstacleb +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      turnleft();
      pause(500);
    }
    else if(mainobstacle ==1 && (obstacleb ==1||obstacleb==2)){// turn rght and enter the b lane then at the next intersection turn right and approach the center lane
      obstacleb +=1;
      movecount = 20;
      blink =1; // blink red Led when an obstacle is seen 
      turnright();
      pause(500);
    }
   
   else if(maininter <4 && obstacleb ==3){// if robot is still in obstacle avoidance and it's not at the last center lane intersection, turn robot right to enter the center lane  
      movecount =20;
      blink =1; // blink red Led when an obstacle is seen 
      forward();
      pause(10);
      spinleft();
      pause(200);
      obstacleb +=1;
    }
   else if(maininter ==4 && obstacleb ==3){  // if robot is still in obstacle avoidance and it's at the last intersection of the center lane,move forward and approach the a lane 
      movecount =20;
      blink = 1; // blink red Led when an obstacle is seen 
      forward();
      binter =0;
      mainobstacle =0;
      ainter +=1;
      maininter +=1; 
      obstacleb = 0;
    }
    else if(maininter ==4 && obstacleb ==0){ // if robot is not in obstacle avoidance, turn left at the last intersection of center lane
      ainter +=1;
      binter =0;
      obstacleb = 0;
      mainobstacle =0;
      movecount = 20;
      maininter +=1;
      blink = 1; // blink red Led when an obstacle is seen 
      turnright();
      pause(500);
    }
    
    else if(ainter ==1 && acounter ==0 && cmdist ==1){// if obstacle present in first pickup location, pause for 2sec and turn right
      dx = (4-acounter); 
      acounter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      blink = 2; // blink blue Led when an obstacle is seen 
      stopmotors();
      pause(2000);
      turnright();
      pause(500);
    }
    else if(ainter ==1 && acounter ==0){ // if no obstacle present in first pickup location, turn right 
      acounter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      turnright();
      pause(500);
    }
    else if(ainter ==1 && (acounter==1 || acounter ==2 || acounter ==3)){//check each intersection on the a lane, if there is an obstacle pause for 2 sec and move forward
      blink =1; // blink red Led when an obstacle is seen 
      movecount = 20;
      forward_slow();
      pause(100);
      
      if(cmdist2 ==1){
        dx =(4-acounter); 
        acounter +=1;
        movecount = 20;
        blink = 2; // blink blue Led when an obstacle is seen 
        stopmotors();
        pause(2000);
        forward_slow();
      }
      else
        {      
        acounter +=1;
        movecount = 20;
        forward_slow();
        }            
      } 
    
   else if(ainter ==1 && acounter==4){ // if at the last intersection of a lane, check if there is an obstacle. if there is an obstacle pause for 2secs and turn right. if no obstacle turn right
      blink = 1; // blink red Led when an obstacle is seen 
      movecount = 20;
      forward_slow();
      pause(100);
      if(cmdist2 ==1)
      { 
      dx =(4-acounter); 
      binter =2;
      acounter +=1;
      bcounter +=1;
      movecount = 20;
      blink = 2; // blink blue Led when an obstacle is seen 
      stopmotors();
      pause(2000);
      spinright();
      pause(200);
    }
      else{
      binter =2;
      acounter +=1;
      bcounter +=1;
      movecount = 20;
      spinright();
      pause(200);
        
      }              
      }
   
   else if(binter ==2 && (bcounter==1)){// move forward until you are approaching the b lane
      bcounter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      forward();
      }
   else if(binter ==2 && (bcounter==2) && cmdist ==1){ // when at the b lane check if there is a drop off at first point. if there is an  obstacle just pause
      distance = (dx+bcounter)*40;
      bcounter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      while(1){
      dprint(lcd, "distance = %d ", distance); //print distance travelled to lcd
      stopmotors();
      writeChar(lcd, CLR);
      blink = 2; // blink blue Led when an obstacle is seen 
    }     
      }
    else if(binter ==2 && (bcounter==2)){// if there is no obstacle at first drop off point, turn right 
      bcounter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      turnright();
      pause(500);
      }
      
   
   else if(binter ==2 && (bcounter>=3 && bcounter<6)){ // at ever other drop off location. if there is no drop off point, move forward. Else pause. 
      blink =1; // blink red Led when an obstacle is seen 
      movecount = 20;
      forward_slow();
      pause(100);
      
        if(cmdist2 ==1){
          distance = (dx+bcounter)*40; 
          bcounter +=1;
          movecount = 20;
          dprint(lcd, "distance = %d ", distance); //print distance travelled to lcd
          while(1){
            stopmotors();
            blink = 2; // blink blue Led when an obstacle is seen 
            }  
          }
        else{
            bcounter +=1;
            movecount = 20;
            forward_slow();
          }         
  }
  
     else if(binter ==2 && (bcounter==6)){ //when at the last intersection of b lane, stop.
      blink =1; // blink red Led when an obstacle is seen 
      movecount = 20;
      forward_slow();
      pause(50);
      
        if(cmdist2 ==1){
          distance = (dx+bcounter)*40; 
          bcounter +=1;
          movecount = 20;
          dprint(lcd, "distance = %d ", distance); //print distance travelled to lcd
          while(1){
            stopmotors();
            blink = 2; // blink blue Led when an obstacle is seen 
            }  
          }
        else{
          bcounter +=1;
          movecount = 20;
          stopmotors();
          pause(5000);  
          }         
  }
   
     
   else{ // ever other situation, just blink and add one to the main intersection
      maininter +=1;
      movecount = 20;
      blink = 1; // blink red Led when an obstacle is seen 
      forward(); 
      
    }
     
   }   
          
  }
 
}

//function to calibrate ultrasonic sensor and read position of blackline
void calibrate(void *par1){
  // local variables to hold sensor reading 
  int cal_ini_sensor_val[8];
  int cal_sensor_val[8];
  int max_cal[8];
  int min_cal[8];
  int cal_value;
  int sensor_ini_val[8];
  int sensor_val[8];
  int sensor_scale_sum;
  
  // charge all IR sensors of the pulolo, get the RC times and initialize the first reading of each sensor to those values
  set_directions(7, 0, 0b11111111);          // P7...P0 -> output
  set_outputs(7, 0, 0b11111111);
  pause(10);
  for(int i=0;i<8;i++){
    cal_ini_sensor_val[i] = rc_time(i,1);
    max_cal[i] = cal_ini_sensor_val[i];
    min_cal[i]= cal_ini_sensor_val[i];
  }

 while(a==1)
  { // calibrate sequence 
    // while turning the robot left and right,read rc time of each sensor multiple times and get their max and min values
    set_directions(7, 0, 0b11111111);          // P7...P0 -> output
    set_outputs(7, 0, 0b11111111);
    pause(10);
  
  for(int i=0;i<8;i++){
    cal_sensor_val[i] = rc_time(i,1);
  } 
 
  for(int i=0;i<8;i++){
  if(min_cal[i] >cal_sensor_val[i]){
      min_cal[i]= cal_sensor_val[i];
    }
    else if (max_cal[i]<cal_sensor_val[i]){
      max_cal[i] = cal_sensor_val[i];
      }
    }
  }
  
   while(a==2){ 
   // after calibration keep taking rc time, scale each senor values to a value between 0 and 1000, use the weighted function to determine the position
    set_directions(7, 0, 0b11111111);          // P7...P0 -> output
    set_outputs(7, 0, 0b11111111);
    pause(10);    
   for(int i=0;i<8;i++){
    sensor_ini_val[i] = rc_time(i,1);
    sensor_val[i] = sensor_ini_val[i];
      if(sensor_val[i]< min_cal[i]){
        sensor_val[i] = min_cal[i];          
      }
      else if (sensor_val[i] >max_cal[i]){
        sensor_val[i] = max_cal[i];
        }
   }
     sensor_scale_sum = 0;      
     for(int i=0;i<8;i++){
       //conduct an interpolation to enable scaling down. Similar to the map function
       float A = (float)(max_cal[i] -sensor_val[i]);
       float B =(float) (max_cal[i] - min_cal[i]);
       float C = (A/B);
       scaled_sensor_val[i] = 1000 -(1000*(C));
       sensor_scale_sum  += scaled_sensor_val[i];
       
       }
    sensor_mean = (int)(sensor_scale_sum/8);
    float D = 0*scaled_sensor_val[0]+ 1000*scaled_sensor_val[1]+2000*scaled_sensor_val[2]+3000*scaled_sensor_val[3]+4000*scaled_sensor_val[4]+5000*scaled_sensor_val[5]+6000*scaled_sensor_val[6]+7000*scaled_sensor_val[7];
    float E = scaled_sensor_val[0]+ scaled_sensor_val[1]+scaled_sensor_val[2]+scaled_sensor_val[3]+scaled_sensor_val[4]+scaled_sensor_val[5]+scaled_sensor_val[6]+scaled_sensor_val[7];
    position = (int)(D/E); // weighted average

    }   
       
} 

// line follow and adjust position of robot based on blackline reading
void linefollow(void){
     if (position < 2800) {
    do {
      movecount = 1;
      turnright();
       } while (position < 2800);
  } else if (position > 4200)
    do {
      movecount = 1;
      turnleft();
     
    } while (position > 4200);
  movecount = 2;
  forward();
 }  
 
// move robot forward by turning both motor in opposite directions at about 80% of the full speed
void forward(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,80);
    servo_speed(17,-80);
    pause(5);
  }
}

// stop robot by stopping both motors 
void stopmotors(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,0);
    servo_speed(17,0);
    pause(20);
  }
}
// turn robot right by stopping right motor and turning left motor forward
void turnright(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,95);
    servo_speed(17,0);
    pause(20);
  }
}

// turn robot right by stopping left motor and turning right motor forward
void turnleft(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,0);
    servo_speed(17,-95);
    pause(20);
  }
}

// spin robot right by turning both motors forward 
void spinright(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,95);
    servo_speed(17,95);
    pause(20);
  }
}
// spin robot left by turning both motors backwards
void spinleft(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,-95);
    servo_speed(17,-95);
    pause(20);
  }
}

// read distance with front untra sonic sensor then convert it to an obstacle true or false. 
void pingfromcog(void *par2) {
  int a;
  while(1)
{
  a =ping_cm(8);
  if(a>30
  ){
    cmdist = 0;}
  else{
    cmdist = 1;
    }
   
}
}

// read distance with left ultrasonic sensor then convert it to an obstacle true or false. 
void pingfromcog2(void *par3) {
  int a;
  while(1)
{
  a =ping_cm(9);
  if(a>30){
    cmdist2 = 0;}
  else{
    cmdist2 = 1;
    } 
}
}

// same at forward function but at about 40% of full speed
void forward_slow(void) {
  for (int i = 0; i < movecount; i++) {
    servo_speed(16,40);
    servo_speed(17,-40);
    pause(5);
  }
}

// same as linefollow function but at the slower speed 
void linefollow_slow(void){
     if (position < 2800) {
    do {
      movecount = 1;
      turnright();
       } while (position < 2800);
  } else if (position > 4200)
    do {
      movecount = 1;
      turnleft();
     
    } while (position > 4200);
  movecount = 2;
  forward_slow();
 }
 
 
// Led function running on cog 4 
void led(void *par4){
  
  while(1){
    
    //whenever blink is 0 turn off both leds
    while(blink ==0){
      low(14);
      low(15);
      pause(10);
      }
    
    
    while(blink ==1){ //when blink is one turn on red led, wait for 0.2 seconds and turn it off
    high(14);
    pause(200);
    blink =0;
    
  }  
    while(blink ==2){ //when blink is two turn on blue led, wait for 0.2 seconds and turn it off
    high(15);
    pause(200);
    blink =0;
  } 
   while(blink ==3){ // when blink is three turn on both leds, wait for 0.2 seconds and turn it off
    high(14);
    high(15);
    pause(200);
    blink =0;
  }
     }    
     
}
 
