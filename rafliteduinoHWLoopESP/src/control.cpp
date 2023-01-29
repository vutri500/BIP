#include <Arduino.h>
#include "proj_types.h"
#include "robot.h"
#include "IRline.h"

extern IRLine_t IRLine;
int count;

void control(robot_t& robot)
{
    robot.tis = millis() - robot.tes;
    // Rules for the state evolution
     if(robot.state == 0 && robot.LastTouchSwitch && !robot.TouchSwitch) {
      robot.rel_s = 0;
      robot.setState(1);

    } else if (robot.state == 1 && robot.TouchSwitch) {
      robot.setState(2);

    } else if(robot.state == 2 && robot.tis > 100) {
      robot.rel_s = 0;
      robot.setState(3);

    } else if(robot.state == 3 && robot.rel_s < -0.12) {
      robot.rel_theta = 0;
      robot.setState(4);

    //} else if(robot.state == 4 && robot.rel_theta > radians(90) && IRLine.total > 1500) {
    } else if(robot.state == 4 && robot.rel_theta > radians(170)) {
      robot.setState(5);
      IRLine.crosses = 0;

    } else if(robot.state == 5 && IRLine.crosses >= 4) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);

    } else if(robot.state == 6 && robot.rel_theta < radians(-70) && IRLine.total > 1500) {
      count += 1;
      robot.setState(7);

    } else if(robot.state == 7 && robot.tis > 2000) {
      IRLine.crosses = 0;
      robot.setState(8);

    } else if(robot.state == 8 && robot.tis > 1500) {
      robot.rel_theta = 0;
      robot.rel_s = 0;
      robot.setState(9); 
    // state for second box
    } else if(robot.state == 9 && robot.rel_theta < radians(-90)) {
      IRLine.crosses = 0;
      if (count == 1) {robot.setState(10);}
      else if (count == 2) {robot.setState(20);}
      else if (count == 3) {robot.setState(24);}

    } else if(robot.state == 10 && IRLine.crosses >=5) {
      if (count == 1) {
        robot.setState(11);
        IRLine.crosses = 0;
      } else if (count == 2) {
        robot.setState(21);
        IRLine.crosses = 0;
      } else if (count == 3) {
        robot.setState(25);
        IRLine.crosses = 0;
      }

    } else if(robot.state == 11 && IRLine.crosses >= 1) {
      robot.setState(12);
    } else if(robot.state == 12 && robot.TouchSwitch) {
      robot.setState(13);
    } else if(robot.state == 13 && robot.tis > 500) {
      robot.rel_s = 0;
      robot.setState(14);
    } else if(robot.state == 14 && robot.rel_s < -0.12) {
      robot.rel_theta = 0;
      robot.setState(15);
    } else if(robot.state == 15 && robot.rel_theta > radians(120)) {
      IRLine.crosses = 0;
      robot.setState(16);
    } else if(robot.state == 16 && robot.tis > (count*2000 + 1000)) {
      IRLine.crosses = 0;
      robot.setState(17);
    } else if(robot.state == 17 && IRLine.crosses >= 5) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      
      if (count == 1) {
        robot.setState(18);
        IRLine.crosses = 0;
      } else if (count == 2) {
        robot.setState(23);
        IRLine.crosses = 0;
      } else if (count == 3) {
        robot.setState(27);
        IRLine.crosses = 0;
      }
      
    } else if(robot.state == 18 && IRLine.crosses >= 1) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);
    /*} else if(robot.state == 19 && robot.tis > 4000) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      IRLine.crosses = 0;
      robot.setState(6);*/
    // state for third box
    } else if(robot.state == 20 && IRLine.crosses >= 2){
      robot.setState(10);
      IRLine.crosses = 1;
    } else if(robot.state == 21 && IRLine.crosses >= 2){
      robot.setState(22);
    } else if(robot.state == 22 && robot.TouchSwitch){
      robot.setState(13);
    } else if(robot.state == 23 && IRLine.crosses >= 2) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);

    // state for last box
    }else if(robot.state == 24 && IRLine.crosses >= 3){
      robot.setState(10);
      IRLine.crosses = 1;
    } else if(robot.state == 25 && IRLine.crosses >= 3){
      robot.setState(26);
    } else if(robot.state == 26 && robot.TouchSwitch){
      robot.setState(13);
    } else if(robot.state == 27 && IRLine.crosses >= 3) {
      robot.rel_s = 0;
      robot.rel_theta = 0;
      robot.setState(6);
    }

/*
    } else if (robot.state == 202 && robot.tis > robot.T1) {
      robot.setState(200);
    }
*/
    // Actions in each state
    if (robot.state == 0) {         // Robot Stoped            
      robot.solenoid_state = 0;
      robot.setRobotVW(0, 0);
    
    } else if (robot.state == 1) {  // Go: Get first box
      robot.solenoid_state = 0;
      robot.followLineLeft(IRLine, 0.2, -0.05);
      // if the request is older than 1000 ms repeat the request

    } else if (robot.state == 2) { // Turn Solenoid On and Get the Box
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.1, -0.05);
    
    } else if (robot.state == 3) {  // Go back with the box
      robot.solenoid_state = 1;
      robot.setRobotVW(-0.1, 0);
      
    } else if (robot.state == 4) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      robot.setRobotVW(0, 2.5);
      
    } else if (robot.state == 5) {  // long travel to the box final destination
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.2, -0.04);
      
    } else if (robot.state == 6) {  // Advance a little then turn to place the box
      robot.solenoid_state = 1;
      if (robot.rel_s < 0.1) robot.followLineRight(IRLine, 0.1, -0.04);
      else if (robot.rel_s < 0.33) robot.followLineLeft(IRLine, 0.1, -0.04);
      else robot.setRobotVW(0.0, -1);
      
    } else if (robot.state == 7) {  
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.05); 


    } else if (robot.state == 8) { // Drop the box and go back
      robot.solenoid_state = 0;
      robot.setRobotVW(-0.1, 0);
    // second box
    } else if (robot.state == 9) { 
      robot.solenoid_state = 0;
      robot.setRobotVW(0, - 1);
    } else if (robot.state == 10) { // go get 2nd box
      robot.solenoid_state == 0;
      robot.followLineLeft(IRLine, 0.1, -0.04);
    } else if (robot.state == 11) {
      robot.solenoid_state == 0;
      robot.followLineRight(IRLine, 0.1, -0.05);
    } else if (robot.state == 12) {
      robot.solenoid_state == 0;
      robot.followLineLeft(IRLine, 0.05,-0.05);
    } else if (robot.state == 13) { // Turn Solenoid On and Get the Box
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.2, -0.04);
    } else if (robot.state == 14) {  // Go back with the box
      robot.solenoid_state = 1;
      robot.setRobotVW(-0.1, 0);
    } else if (robot.state == 15) {  // Turn 180 degrees
      robot.solenoid_state = 1;
      robot.setRobotVW(0, 1.5);
    } else if (robot.state == 16) {   
      robot.solenoid_state = 1;
      robot.followLineLeft(IRLine, 0.1, -0.05);
    } else if (robot.state == 17) {
      robot.solenoid_state = 1;
      robot.followLineRight(IRLine, 0.1, -0.05);
    } else if (robot.state == 18) {
      robot.solenoid_state = 1; 
      robot.followLineLeft(IRLine,0.1,-0.04);
    } else if (robot.state == 19) {
      robot.solenoid_state = 1; 
      robot.followLineRight(IRLine,0.05,-0.04);

    // third box
    } else if (robot.state == 20) {
      robot.solenoid_state = 0; 
      robot.followLineRight(IRLine,0.05,-0.04);
    } else if (robot.state == 21) {
      robot.solenoid_state == 0;
      robot.followLineRight(IRLine, 0.05, -0.05);
    } else if (robot.state == 22) {
      robot.solenoid_state == 0;
      robot.followLineLeft(IRLine,0.1,-0.04);
    } else if (robot.state == 23) {
      robot.solenoid_state = 1; 
      robot.followLineLeft(IRLine,0.05,-0.05);

    // last box
    } else if (robot.state == 24) {
      robot.solenoid_state = 0; 
      robot.followLineRight(IRLine,0.05,-0.04);
    } else if (robot.state == 25) {
      robot.solenoid_state == 0;
      robot.followLineRight(IRLine, 0.05, -0.05);
    } else if (robot.state == 26) {
      robot.solenoid_state == 0;
      robot.followLineLeft(IRLine,0.1,-0.06);
    } else if (robot.state == 27) {
      robot.solenoid_state = 1; 
      robot.followLineLeft(IRLine,0.05,-0.05);
    }
     else if (robot.state == 100) {
      robot.v_req = 0;
      robot.w_req = 0;

    } else if (robot.state == 101) {
      //robot.v_req = 0;
      //robot.w_req = 0;

    } else if (robot.state == 199) {
      /*robot.v_req = 0.1;
      robot.w_req = 4 * IRLine.IR_values[4] / 1024.0 
                  + 2 * IRLine.IR_values[3] / 1024.0
                  - 2 * IRLine.IR_values[1] / 1024.0
                  - 4 * IRLine.IR_values[0] / 1024.0;*/

    } else if (robot.state == 200) {
      robot.PWM_1 = 0;
      robot.PWM_2 = 0;

    } else if (robot.state == 201) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;
    
    } else if (robot.state == 202) {
      robot.PWM_1 = robot.PWM_1_req;
      robot.PWM_2 = robot.PWM_2_req;      
    }
  
}