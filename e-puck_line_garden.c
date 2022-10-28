/*
 * Copyright 2022 Subramanian Raghavan u2179625 University of Huddersfield
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include <string.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/supervisor.h>

// Global defines
#define TRUE 1
#define FALSE 0
#define NO_SIDE -1
#define LEFT 0
#define RIGHT 1
#define WHITE 0
#define BLACK 1
#define TIME_STEP 32  // [ms]

// 8 IR proximity sensors
#define NB_DIST_SENS 8
#define PS_RIGHT_00 0 //ps0
#define PS_RIGHT_45 1 //ps1
#define PS_RIGHT_90 2 //ps2
#define PS_RIGHT_REAR 3 //ps3
#define PS_LEFT_REAR 4 //ps4
#define PS_LEFT_90 5 //ps5
#define PS_LEFT_45 6 //ps6
#define PS_LEFT_00 7 //ps7
const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = {300, 300, 300, 300, 300, 300, 300, 300};
const int PS_OFFSET_REALITY[NB_DIST_SENS] = {480, 170, 320, 500, 600, 680, 210, 640};
WbDeviceTag ps[NB_DIST_SENS]; /* proximity sensors */
int ps_value[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0};

// 3 IR ground color sensors
#define NB_GROUND_SENS 3
#define GS_WHITE 900
#define GS_LEFT 0
#define GS_CENTER 1
#define GS_RIGHT 2
WbDeviceTag gs[NB_GROUND_SENS]; /* ground sensors */
unsigned short gs_value[NB_GROUND_SENS] = {0, 0, 0};

// Apple count
int applecount =0;

//placeholder
int gDeltaS=0;

// Motors
WbDeviceTag left_motor, right_motor;

// LEDs
#define NB_LEDS 8
WbDeviceTag led[NB_LEDS];

//File: Please change this location when running the code in your m/c
char PATHNAME[100] = "C:\\Users\\sbsub\\Downloads\\AI\\Lectures\\Robotics\\Assignment2\\code\\logs\\";
char FILEPATH[100] = "";

//------------------------------------------------------------------------------
//
//    BEHAVIORAL MODULES
//
//------------------------------------------------------------------------------
// LFM - Line Following Module
int lfm_speed[2];
#define LFM_FORWARD_SPEED 200
#define LFM_K_GS_SPEED 0.4

void LineFollowingModule(void) {  
  int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];  
  /*
   DeltaS > 0 imples the following:-
    i.gs_value[GS_RIGHT] > gs_value[GS_LEFT]
    ii.epuck movement is biased towards right. 
    iii.To stay on course, epuck needs to be pulled towards left
    iv.Left motors speed < right motor speed i.e lfm_speed[LEFT] should be set to a value < lfm_speed[RIGHT]
   DeltaS < 0 implies the following:-
    i.gs_value[GS_RIGHT] < gs_value[GS_LEFT]
    ii.epuck movement biased towards left. 
    iii.To stay on course, epuck needs to be pulled towards right
    iv.Left motors speed > right motor speed i.e lfm_speed[LEFT] should be set to a value > lfm_speed[RIGHT]
  */
  lfm_speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
  lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;

  //write to file
  FILE *fptr;
  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);  
  strcat(FILEPATH,"deltas_gs.csv");  
  fptr=fopen(FILEPATH,"a+");            
  fprintf(fptr,"%d, %d, %d, %d, %d\n", DeltaS, gs_value[GS_RIGHT], gs_value[GS_LEFT],lfm_speed[LEFT],lfm_speed[RIGHT]);  
  fclose(fptr);

}

//DetectApple module - Detect apple from image captured by the camera
//If colour detected is green, return true
bool DetectApple(WbDeviceTag camera)
{
    bool ret=false;
    //x,y:pixel dimension
    //r,g,b:pixel colour
    int x = 0, y = 0, r = 0, g = 0, b = 0;

    //first get a pointer to the image
    const unsigned char * image = wb_camera_get_image(camera);
        
    //get image dimensions
    int image_width = wb_camera_get_width(camera);
    int image_height = wb_camera_get_height(camera);

    printf("Inside DetectApple\n");
    
    //get colour for each pixel moving across width and height of the image
    for (x = 0; x < image_width; x++)
    {
    for (y = 0; y < image_height; y++)
        {
        // get the intensity value of the current pixel on R channel
        r = wb_camera_image_get_red(image, image_width, x, y);
        // get the intensity value of the current pixel on G channel
        g = wb_camera_image_get_green(image, image_width, x, y);
        // get the intensity value of the current pixel on B channel
        b = wb_camera_image_get_blue(image, image_width, x, y);
        
        //check whether the current pixel is GREEN or not
        //50 is the tolerance value selected after trial and error        
        if((g > 50 && (r<g && b<g))) //making sure that the intensity of the green pixel is more than red(r) and blue(b)
         {
            printf("red=%d, green=%d, blue=%d \n", r, g, b);
            ret=true;
         }
         else
            ret=false; //either the pixel is green but its intensity is less than the set tolerance or it is red or blue
        }		
    }
    return ret;
}

/*
  OAM - Obstacle Avoidance Module
  This module does the following:-
    i.Detects an obstacle
    ii.Determines whether the obstacle is an apple or not
    iii.Determines whether the obstacle is to the left or right side of the epuck
    iv.Determines the speed of left and right wheels of epuck
*/
int oam_active, oam_reset;
int oam_speed[2];
int oam_side = NO_SIDE;

#define OAM_OBST_THRESHOLD 100
#define OAM_FORWARD_SPEED 150
#define OAM_K_PS_90 0.2
#define OAM_K_PS_45 0.9
#define OAM_K_PS_00 1.2
#define OAM_K_MAX_DELTAS 600

bool ObstacleAvoidanceModule(WbDeviceTag camera) 
{
  bool applefound=false;

  // Module RESET
  if (oam_reset) {
    oam_active = FALSE;
    oam_side = NO_SIDE;
  }
  oam_reset = 0;
  
  /*//uncomment for debug
  printf("Within OAM, ps_value[PS_LEFT_00]:%d,ps_value[PS_LEFT_45]:%d,ps_value[PS_LEFT_90]:%d\n",ps_value[PS_LEFT_00],ps_value[PS_LEFT_45],ps_value[PS_LEFT_90]);
  printf("Within OAM, ps_value[PS_RIGHT_00]:%d,ps_value[PS_RIGHT_45]:%d,ps_value[PS_RIGHT_90]:%d\n",ps_value[PS_RIGHT_00],ps_value[PS_RIGHT_45],ps_value[PS_RIGHT_90]);
  */
  //check whether either of the two proximity sensors, placed at front, have detected the obstacle by comparing their values to predefined threshold
  if ((ps_value[PS_RIGHT_00] > OAM_OBST_THRESHOLD) || ((ps_value[PS_LEFT_00] > OAM_OBST_THRESHOLD)))
  {
    oam_active = TRUE;    
    //catch and process an image of the obstacle to determine if it is apple
    applefound=DetectApple(camera);
    printf("Apple Found?: %s\n", applefound ? "true" : "false");
   } 

  if (oam_active && oam_side == NO_SIDE)  //check for side of obstacle only when not already detected
  {
    if(ps_value[PS_RIGHT_00] > ps_value[PS_LEFT_00])
      oam_side = RIGHT; //obstacle detected on the right 
    else
    {
      /*//uncomment for debug
      printf("With oam_side==LEFT, ps_value[PS_LEFT_00]:%d,ps_value[PS_LEFT_45]:%d,ps_value[PS_LEFT_90]:%d\n",ps_value[PS_LEFT_00],ps_value[PS_LEFT_45],ps_value[PS_LEFT_90]);
      printf("With oam_side==LEFT, ps_value[PS_RIGHT_00]:%d,ps_value[PS_RIGHT_45]:%d,ps_value[PS_RIGHT_90]:%d\n",ps_value[PS_RIGHT_00],ps_value[PS_RIGHT_45],ps_value[PS_RIGHT_90]);
      */
      oam_side = LEFT; //obstacle detected on the left 
    }
  } 
  
  /*
    Initialise speed of each wheel
    If there is no obstacle, ps_value[PS_LEFT/RIGHT_00],ps_value[PS_LEFT/RIGHT_45], ps_value[PS_LEFT/RIGHT_90] will be==0.
    So, when in OAM module, code execution will directly come to this point and assign oam_speed[LEFT/RIGHT] = OAM_FORWARD_SPEED
  */
  oam_speed[LEFT] = OAM_FORWARD_SPEED;
  oam_speed[RIGHT] = OAM_FORWARD_SPEED;

  //Determine the Error Signal (DeltaS) which will be used to moderate the speed of each wheel
  if (oam_active) {
    int DeltaS = 0; //if there is no obstacle found yet, oam_active==1, oam_side == -1 and error signal(DeltaS) will be ==0.
                    //This implies, oam_speed[LEFT/RIGHT] = OAM_FORWARD_SPEED and epuck will keep moving straight 
    // The rotation of the robot is determined by the location and the side of the obstacle
    if (oam_side == LEFT) {
      
      DeltaS -= (int)(OAM_K_PS_90 * ps_value[PS_LEFT_90]); //while moving away from the obstacle (here on the left i.e., oam_side==LEFT), oam_active==1 and ps_value becomes==0. This makes oam_speed[RIGHT]:150, ofm_speed[RIGHT]: 150 because DeltaS==0  
      DeltaS -= (int)(OAM_K_PS_45 * ps_value[PS_LEFT_45]);      
      DeltaS -= (int)(OAM_K_PS_00 * ps_value[PS_LEFT_00]);
    } else {  // oam_side == RIGHT      
      /*
        In addition to obstacles that are detected on right, this section will also be executed when apple is found(all apples)
        Because, all apples are detected as obstacles whose locations are biased towards the right side of epuck
        therefore, when apple is removed, values of oam_side: 1 and oam_active: 1
        NOTE: when apple is found, epuck will exit this module with applefound=TRUE. 
        The controller will proceed to execute LLM where apple will be removed, and then come back after completing one full for(;;) loop in main() 
      */
      DeltaS += (int)(OAM_K_PS_00 * ps_value[PS_RIGHT_00]);      
      DeltaS += (int)(OAM_K_PS_45 * ps_value[PS_RIGHT_45]);
      DeltaS += (int)(OAM_K_PS_90 * ps_value[PS_RIGHT_90]);      
    }

    //Normalise DeltaS for wide differences between proximity sensor values (>600 or <600)
    if (DeltaS > OAM_K_MAX_DELTAS)
      DeltaS = OAM_K_MAX_DELTAS;
    if (DeltaS < -OAM_K_MAX_DELTAS)
      DeltaS = -OAM_K_MAX_DELTAS;

    /*
      Now set speed of each individual epuck wheel as follows:-
       if oam_side==0,it implies:- 
        i.Obstacle side is on the left
        ii.epuck will leave line turning towards right
        iii.oam_speed[RIGHT] < oam_speed[LEFT] 
        iv.DeltaS will be <=0
       if oam_side==1,it implies:-
        i.Obstacle side is on the right
        ii.epuck will leave line turning towards left
        iii.oam_speed[LEFT] < oam_speed[RIGHT]
        iv.DeltaS will be >=0
      if there is no obstacle found yet, oam_active==1, oam_side == -1 and error signal(DeltaS) will be ==0.
        This implies, oam_speed[LEFT/RIGHT] = OAM_FORWARD_SPEED and epuck will keep moving straight   
    */
    oam_speed[LEFT] -= DeltaS;
    oam_speed[RIGHT] += DeltaS;   

    //placeholder
    gDeltaS=DeltaS;

    /*
    NOTES:
    1.For epuck to keep moving straight when apple is removed, following criteria will be have to be met:-
       i.oam_active should be TRUE[i.e 1] 
       ii.oam_side should be RIGHT[i.e 1] 
       iii.DeltaS should be approach zero which will ensure both wheels continue moving with same speed i.e OAM_FORWARD_SPEED
       iv.oam_speed[LEFT] and oam_speed[LEFT]=OAM_FORWARD_SPEED 

    2. Since, OAM_FORWARD_SPEED 150 is which is < LFM_FORWARD_SPEED 200 as set earlier in LineFollowingModule().
       So, epuck will move a bit slow here   
    */    
    
  //write to file:for Debug
  FILE *fptr;
  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);
  strcat(FILEPATH,"oam_speed.csv");  
  fptr=fopen(FILEPATH,"a+");           
  fprintf(fptr,"%d, %d, %d, %d\n", oam_side,oam_speed[LEFT],oam_speed[RIGHT],DeltaS);  
  fclose(fptr);    
    
  }
  return applefound;
}

/*
  LLM - Line Leaving Module
  This module does the following:-
    i.Move each of the four apples to predefined poositions within the arena if they were found earlier in Obstacle Avoidance Module
    ii.Determines the speed of left and right wheels of epuck such that it will leave the line in a direction opposite to the obstacle side    
*/
#define LLM_THRESHOLD 800
#define OFM_DELTA_SPEED 150

int llm_active = FALSE, llm_ofm_speed, llm_past_side = NO_SIDE;
int lem_reset;
int ofm_speed[2];
int oam_ofm_speed[2];

void LineLeavingModule(bool applefound,int side) {
printf("Inside LineLeavingModule\n");  
  //Check whether Line Leaving Module flag was already set and whether the obstacle's side was already found
  if (!llm_active && side != NO_SIDE && llm_past_side == NO_SIDE)
    llm_active = TRUE;
  // Updating the memory of the "side" state at the previous call
  llm_past_side = side;  

//Apples are moved to new location (x,y,z coordinates) by running Epuck in Supervisor mode
WbNodeRef apple_node;
WbFieldRef translation_field;
double new_value[3]; //x,y,z coordinates of new position for apple
if(applefound)
{
  printf("applecount: %d\n",applecount);
  if(applecount==0)
   {
     apple_node = wb_supervisor_node_get_from_def("APPLE1");
     translation_field = wb_supervisor_node_get_field(apple_node, "translation");
     new_value[0] = 0.22;  
     new_value[1] = 0.33;
     new_value[2] = 0;
     applecount+=1;
  }
  else if (applecount==1)
  {
     apple_node = wb_supervisor_node_get_from_def("APPLE2");
     translation_field = wb_supervisor_node_get_field(apple_node, "translation");
     new_value[0] = 0.22;//-0.26;  
     new_value[1] = 0.27;//-0.24;
     new_value[2] = 0;
     applecount+=1;
  }
  else if (applecount==2)
  {
     apple_node = wb_supervisor_node_get_from_def("APPLE3");
     translation_field = wb_supervisor_node_get_field(apple_node, "translation");
     new_value[0] = -0.32;  
     new_value[1] = -0.24;
     new_value[2] = 0;
     applecount+=1;
  }
  else if (applecount==3)
  {
     apple_node = wb_supervisor_node_get_from_def("APPLE4");
     translation_field = wb_supervisor_node_get_field(apple_node, "translation");
     new_value[0] = -0.23;  
     new_value[1] = -0.27;
     new_value[2] = 0;
     applecount+=1;
  }
  wb_supervisor_field_set_sf_vec3f(translation_field, new_value);
}  
  
  if (llm_active) {  
    if (side == LEFT) {// obstacle's side is on LEFT
      if ((gs_value[GS_CENTER] + gs_value[GS_LEFT]) / 2 > LLM_THRESHOLD) {//when the obstacle is on the left, the left wheel will cross over the black line when epuck is moving with oam_speed[LEFT/RIGHT] values away from obstacle 
        llm_active = FALSE;
        llm_ofm_speed = FALSE;
        lem_reset = TRUE;        
      } else {  
        llm_ofm_speed = TRUE; //obstacle detected, its side is on left but line leaving threshold not crossed yet, stay on line and keep moving towards the obstacle             
      }
    } else {// obstacle's side is on RIGHT
      if ((gs_value[GS_CENTER] + gs_value[GS_RIGHT]) / 2 > LLM_THRESHOLD) {  //when the obstacle is on the right, the right wheel will cross over the black line when epuck is moving with oam_speed[LEFT/RIGHT] values away from obstacle 
        llm_active = FALSE;        
        llm_ofm_speed = FALSE;
        lem_reset = TRUE;       

      } else {          
        llm_ofm_speed = TRUE; //obstacle detected, its side is on right but line leaving threshold not crossed yet, stay on line and keep moving towards the obstacle      
      }
    }
  }
  
  if (side == NO_SIDE) { //side not detected => no obstacle present
    ofm_speed[LEFT] = 0;
    ofm_speed[RIGHT] = 0;
  }
  else
  {
    if (llm_ofm_speed) { //side detected=>obstacle present but epuck not out of line yet
      ofm_speed[LEFT] = 0; //do not set the lane leaving speed for the left wheel yet
      ofm_speed[RIGHT] = 0; //do not set the lane leaving speed for the left wheel yet
    }
    else{ //side detected=>obstacle present, line leaving threshold crossed, epuck ready to move out of line
        if (side == LEFT) {
        ofm_speed[LEFT] = -OFM_DELTA_SPEED; //obstacle side is on left, so prepare to move right
        ofm_speed[RIGHT] = OFM_DELTA_SPEED;
        /*//Uncomment for Debug
        printf("With side == LEFT, gDeltaS:%d\n",gDeltaS);
        printf("With side == LEFT, ps_value[PS_LEFT_90]:%d, ps_value[PS_LEFT_45]: %d, ps_value[PS_LEFT_00]:%d\n",ps_value[PS_LEFT_90],ps_value[PS_LEFT_45],ps_value[PS_LEFT_00]);
        printf("With side == LEFT, ps_value[PS_RIGHT_90]:%d, ps_value[PS_RIGHT_45]: %d, ps_value[PS_RIGHT_00]:%d\n",ps_value[PS_RIGHT_90],ps_value[PS_RIGHT_45],ps_value[PS_RIGHT_00]);
        printf("With side == LEFT, oam_speed[LEFT]:%d, ofm_speed[LEFT]: %d\n",oam_speed[LEFT],ofm_speed[LEFT]);
        printf("With side == LEFT, oam_speed[RIGHT]:%d, ofm_speed[RIGHT]: %d\n",oam_speed[RIGHT],ofm_speed[RIGHT]);
        */
      } else {
        ofm_speed[LEFT] = OFM_DELTA_SPEED;
        ofm_speed[RIGHT] = -OFM_DELTA_SPEED; //obstacle side is on right, so prepare to move left
        /*//Uncomment for Debug
        printf("With side == RIGHT, ofm_speed[LEFT]: %d\n",ofm_speed[LEFT]);
        printf("With side == RIGHT, ofm_speed[RIGHT]: %d\n",ofm_speed[RIGHT]);
        */
      }
    }
 }
  //Determine the supression speeds
  oam_ofm_speed[LEFT] = oam_speed[LEFT] + ofm_speed[LEFT]; 
  oam_ofm_speed[RIGHT] = oam_speed[RIGHT] + ofm_speed[RIGHT];

  //write to file-For Debug
  FILE *fptr;
  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);
  strcat(FILEPATH,"oam_ofm_speed.csv");  
  fptr=fopen(FILEPATH,"a+");           
  fprintf(fptr,"%d,%d, %d, %d, %d,%d, %d,%d,%d,%d\n", side,oam_ofm_speed[LEFT],oam_ofm_speed[RIGHT],oam_speed[LEFT],ofm_speed[LEFT], oam_speed[RIGHT],ofm_speed[RIGHT], ps_value[PS_LEFT_90], ps_value[PS_LEFT_45], ps_value[PS_LEFT_00]);  
  fclose(fptr);

}

/*
  LEM - Line Entering Module
  This module does the following:-
    i.Checks the obstacle' side to decide which way to re-enter the line
    ii.Sets the epuck speed depending upon its line finding state    
    iii.Reenters the line path
*/
int lem_active;
int lem_speed[2];
int lem_state, lem_black_counter;
int cur_op_gs_value, prev_op_gs_value;

#define LEM_FORWARD_SPEED 100
#define LEM_K_GS_SPEED 0.5
#define LEM_THRESHOLD 500

#define LEM_STATE_STANDBY 0
#define LEM_STATE_LOOKING_FOR_LINE 1
#define LEM_STATE_LINE_DETECTED 2
#define LEM_STATE_ON_LINE 3

void LineEnteringModule(int side) {
  int Side, OpSide, GS_Side, GS_OpSide;

  // check whether obstacle was found, and epuck had started moving away from line (=>lem_reset==TRUE set in OAM)
  if (lem_reset)
    lem_state = LEM_STATE_LOOKING_FOR_LINE;
  lem_reset = FALSE;

  // Initialization
  lem_speed[LEFT] = LEM_FORWARD_SPEED;
  lem_speed[RIGHT] = LEM_FORWARD_SPEED;
  if (side == LEFT) {  // if obstacle on left side -> enter line rightward
    Side = RIGHT;      // line entering direction
    OpSide = LEFT;
    GS_Side = GS_RIGHT;
    GS_OpSide = GS_LEFT;
  } else {        // if obstacle on right side -> enter line leftward
    Side = LEFT;  // line entering direction
    OpSide = RIGHT;
    GS_Side = GS_LEFT;
    GS_OpSide = GS_RIGHT;
  }
  
  //Set the epuck speed depending upon its line finding state
  switch (lem_state) {
    case LEM_STATE_STANDBY:
      lem_active = FALSE;
      break;
    case LEM_STATE_LOOKING_FOR_LINE:
      if (gs_value[GS_Side] < LEM_THRESHOLD) { 
        lem_active = TRUE; //keep epuck in line entering mode
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED; 
        lem_speed[Side] = LEM_FORWARD_SPEED;  // - LEM_K_GS_SPEED * gs_value[GS_Side];
        lem_state = LEM_STATE_LINE_DETECTED; //black strip path found on the wheel closer to the obstacle
        // save ground sensor value
        if (gs_value[GS_OpSide] < LEM_THRESHOLD) { //black strip path found on the wheel away to the obstacle
          cur_op_gs_value = BLACK;          
          lem_black_counter = 1;
        } else {
          cur_op_gs_value = WHITE; //only one wheel has detected the black strip          
          lem_black_counter = 0;
        }
        prev_op_gs_value = cur_op_gs_value;
      }
      break;
    case LEM_STATE_LINE_DETECTED:
      // save the oposite ground sensor value
      if (gs_value[GS_OpSide] < LEM_THRESHOLD) {
        cur_op_gs_value = BLACK;
        lem_black_counter++;
      } else
        cur_op_gs_value = WHITE;
      // detect the falling edge BLACK->WHITE
      if (prev_op_gs_value == BLACK && cur_op_gs_value == WHITE) {
        lem_state = LEM_STATE_ON_LINE;
        lem_speed[OpSide] = 0;
        lem_speed[Side] = 0;
      } else {
        prev_op_gs_value = cur_op_gs_value;
        // set speeds for entering line
        lem_speed[OpSide] = LEM_FORWARD_SPEED + LEM_K_GS_SPEED * (GS_WHITE - gs_value[GS_Side]);
        lem_speed[Side] = LEM_FORWARD_SPEED - LEM_K_GS_SPEED * (GS_WHITE - gs_value[GS_Side]);
      } 
      break;
    case LEM_STATE_ON_LINE:
      oam_reset = TRUE;
      lem_active = FALSE;
      lem_state = LEM_STATE_STANDBY;
      break;
  }
}

//------------------------------------------------------------------------------
//
//    CONTROLLER
//
//------------------------------------------------------------------------------
// Main
int main() {
  int ps_offset[NB_DIST_SENS] = {0, 0, 0, 0, 0, 0, 0, 0}, i, speed[2];  

  /* intialize Webots */
  wb_robot_init();  
  
  /* initialization */
  char name[20];
  for (i = 0; i < NB_LEDS; i++) {
    sprintf(name, "led%d", i);
    led[i] = wb_robot_get_device(name); /* get a handler to the sensor */
  }
  for (i = 0; i < NB_DIST_SENS; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name); /* proximity sensors */
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }
  for (i = 0; i < NB_GROUND_SENS; i++) {
    sprintf(name, "gs%d", i);
    gs[i] = wb_robot_get_device(name); /* ground sensors */
    wb_distance_sensor_enable(gs[i], TIME_STEP);
  }
  // motors
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // camera
  // get camera and activate it
  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP * 16);	

  //initialise file path
  strcpy(FILEPATH, "");    

  //When the system first starts up all modules start in the distinguished state named NIL. 
  //In line with Rodney Brooks' A Robust Layered Control System For A Mobile Robot page 4/10 "A. Finite State Machines"
  oam_reset = TRUE;
  llm_active = FALSE;
  llm_past_side = NO_SIDE;
  lem_active = FALSE;
  lem_state = LEM_STATE_STANDBY;

   for (i = 0; i < NB_DIST_SENS; i++)
       ps_offset[i] = PS_OFFSET_SIMULATION[i];        
    
    wb_robot_step(TIME_STEP);  // Just run one step to make sure we get correct sensor values

  //set up Debug files
  FILE *fptr;  
  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);
  strcat(FILEPATH,"deltas_gs.csv");  //deltas_gs.csv
  fptr=fopen(FILEPATH,"a+");    
  fprintf(fptr,"DeltaS, gs_value[GS_RIGHT], gs_value[GS_LEFT],lfm_speed[LEFT],lfm_speed[RIGHT]\n");  
  fclose(fptr);

  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);
  strcat(FILEPATH,"oam_speed.csv");  //oam_speed.csv
  fptr=fopen(FILEPATH,"a+");    
  fprintf(fptr,"oam_side,oam_speed[LEFT],oam_speed[RIGHT],DeltaS\n");  
  fclose(fptr);

  strcpy(FILEPATH, "");    
  strcat(FILEPATH,"oam_ofm_speed.csv");  //oam_ofm_speed.csv
  fptr=fopen(FILEPATH,"a+");
  fprintf(fptr,"side,oam_ofm_speed[LEFT],oam_ofm_speed[RIGHT],oam_speed[LEFT],ofm_speed[LEFT], oam_speed[RIGHT],ofm_speed[RIGHT],ps_value[PS_LEFT_90], ps_value[PS_LEFT_45], ps_value[PS_LEFT_00]\n");      
  fclose(fptr);

  strcpy(FILEPATH, "");    
  strcpy(FILEPATH, PATHNAME);
  strcat(FILEPATH,"supressing_speed.csv");  //supressing_speed.csv
  fptr=fopen(FILEPATH,"a+");  
  fprintf(fptr,"oam_side,oam_active,speed[LEFT],oam_ofm_speed[LEFT],speed[RIGHT],oam_ofm_speed[RIGHT]\n");         
  fclose(fptr);
  
  for (;;) {  // Main loop
    // Run one simulation step
    wb_robot_step(TIME_STEP);
    printf("inside main-for loop\n");

    //Note:The supervisor mode for epuck is set in .wbt file
    
    // read sensors value    
    for (i = 0; i < NB_DIST_SENS; i++)    
        ps_value[i] = (((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]) < 0) ?
                      0 :
                      ((int)wb_distance_sensor_get_value(ps[i]) - ps_offset[i]);
    
    for (i = 0; i < NB_GROUND_SENS; i++)    
      gs_value[i] = wb_distance_sensor_get_value(gs[i]);
      
    // Speed initialization
    speed[LEFT] = 0; 
    speed[RIGHT] = 0; 
    
    
        // *** START OF SUBSUMPTION ARCHITECTURE ***               
        //Line Following Module(LFM): Competency Level 0        
        LineFollowingModule();

        //set wheel speed
        speed[LEFT] = lfm_speed[LEFT]; 
        speed[RIGHT] = lfm_speed[RIGHT];

        //Obstacle Avoidance Module(OAM): Competency Level 1        
        bool applefound=false;
        applefound=ObstacleAvoidanceModule(camera); //send camera parameters

        //Line Leaving Module(LLM): Competency Level 2
        LineLeavingModule(applefound,oam_side); //LLM is able to examine data(e.g applefound) coming from a layer before it[i.e OAM]      

        //Write to File-for Debug
        strcpy(FILEPATH, "");    
        strcpy(FILEPATH, PATHNAME);
        strcat(FILEPATH,"supressing_speed.csv");  
        fptr=fopen(FILEPATH,"a+");        
        fprintf(fptr,"%d,%d, %d, %d, %d, %d\n",oam_side,oam_active,speed[LEFT],oam_ofm_speed[LEFT],speed[RIGHT],oam_ofm_speed[RIGHT]);      
        fclose(fptr);  
        
        /*
          Suppression S_2: this should affect input of layer below LLM [i.e LFM]    
          oam_ofm_speed[LEFT] value should be set with values coming from layer above LFM.
          i.e., oam_ofm_speed[LEFT] = oam_speed[LEFT] + ofm_speed[LEFT]; where oam_speed[LEFT] is set in OAM and ofm_speed[LEFT] is set in LLM        
          i/p signal(speed[LEFT] or speed[RIGHT] ) getting replaced with signal (oam_ofm_speed[LEFT] or oam_ofm_speed[RIGHT]) set in a layer above
          oam_ofm_speed[LEFT] or oam_ofm_speed[RIGHT] will suppress the signal to the wheel on which the rotation needs to happen
        */
        if (oam_active) { 
        speed[LEFT] = oam_ofm_speed[LEFT]; 
        speed[RIGHT] = oam_ofm_speed[RIGHT]; 
        }
         
        //Line Entering Module(LEM): Competency Level 3        
        LineEnteringModule(oam_side);

        /*
          Suppression S_3: this should affect input of layer below LEM [i.e LFM]
          lem_speed[LEFT],lem_speed[RIGHT] value should be set with values coming from layer above LFM.
          Thus lem_speed[LEFT] is set in LEM which is a layers above LFM    
        */
        if (lem_active) {
        speed[LEFT] = lem_speed[LEFT];
        speed[RIGHT] = lem_speed[RIGHT];
        }
        // *** END OF SUBSUMPTION ARCHITECTURE ***
    
    // Debug display
    printf("oam_active %d oam_side %d   llm_active %d llm_ofm_speed %d   lem_active %d lem_state %d oam_reset %d\n", oam_active, oam_side, llm_active,
           llm_ofm_speed, /*ofm_active,*/ lem_active, lem_state, oam_reset);

    // Set wheel speeds
    // while the robot executes the requested actions via motors, the loop starts again
    wb_motor_set_velocity(left_motor, 0.00628 * speed[LEFT]);
    wb_motor_set_velocity(right_motor, 0.00628 * speed[RIGHT]);  

  }//end of (for ;;)
  return 0;
}
