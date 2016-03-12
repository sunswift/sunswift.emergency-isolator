/* --------------------------------------------------------------------------                                 
    Template project main
    File name: template.c
    Author: Etienne Le Sueur
    Description: The template main file. It provides a simple example of using
                 some standard scandal functions such as the UART library, the
                 CAN library, timers, LEDs, GPIOs.
                 It is designed to compile and work for the 3 micros we use on
                 the car currently, the MSP430F149 and MCP2515 combination and
                 the LPC11C14 and LPC1768 which have built in CAN controllers.

                 UART_printf is provided by the Scandal stdio library and 
                 should work well on all 3 micros.

                 If you are using this as the base for a new project, you
                 should first change the name of this file to the name of
                 your project, and then in the top level makefile, you should
                 change the CHIP and MAIN_NAME variables to correspond to
                 your hardware.

                 Don't be scared of Scandal. Dig into the code and try to
                 understand what's going on. If you think of an improvement
                 to any of the functions, then go ahead and implement it.
                 However, before committing the changes to the Scandal repo,
                 you should discuss with someone else to ensure that what 
                 you've done is a good thing ;-)

                 Keep in mind that this code is live to the public on
                 Google Code. No profanity in comments please!

    Copyright (C) Etienne Le Sueur, 2011

    Created: 07/09/2011
   -------------------------------------------------------------------------- */

/* 
 * This file is part of the Sunswift Template project
 * 
 * This tempalte is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 
 * It is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 
 * You should have received a copy of the GNU General Public License
 * along with the project.  If not, see <http://www.gnu.org/licenses/>.
 */

#undef 	DOUBLE_BUFFER_EXAMPLE
#undef 	IN_CHANNEL_EXAMPLE
#undef WAVESCULPTOR_EXAMPLE

#define BASE_YEL_LED_PORT 3
#define BASE_YEL_LED_BIT 1
#define BASE_RED_LED_PORT 3
#define BASE_RED_LED_BIT 0


//#include <scandal/engine.h>
#include <scandal/message.h>
#include <scandal/leds.h>
#include <scandal/utils.h>
#include <scandal/uart.h>
#include <scandal/stdio.h>
#include <scandal/wdt.h>
#include <scandal/wavesculptor.h>
#include <scandal/error.h>

#include <string.h>

#include <project/driver_config.h>
#include <project/target_config.h>
#include <arch/can.h>
#include <arch/uart.h>
#include <arch/timer.h>
#include <arch/gpio.h>
#include <arch/types.h>
#include <arch/i2c.h>
#include <arch/wdt.h>
void std_msg_handler(can_msg *msg);
float targetTorque(int32_t targetRPM, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct);
int32_t torqueControl(float desiredTorque, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct);
Wavesculptor_Output_Struct Left_WS;
Wavesculptor_Output_Struct Right_WS;




/* Do some general setup for clocks, LEDs and interrupts
 * and UART stuff on the MSP430 */
void setup(void) {

    WDT_Init(WATCHDOG_TIMER_PERIOD);
    
    int i=0;;
	GPIO_Init();
	GPIO_SetDir(BASE_RED_LED_PORT, BASE_RED_LED_BIT, OUTPUT);
	GPIO_SetDir(BASE_YEL_LED_PORT, BASE_YEL_LED_BIT, OUTPUT);
    
    
    
     /* Initialise UART0 */
	UART_Init(115200);
    
    init_can();
    sc_init_timer();
    
    while(can_register_id(0, 0, 0, 0)==NO_ERR)
        ;

	/* Initialise Scandal, registers for the config messages, timesync messages and in channels */
	scandal_init();
    
    

	/* Set LEDs to known states */
	red_led(0);
	yellow_led(1);
    
    Left_WS.BaseAddress  = 0x400;
    Right_WS.BaseAddress = 0x420;
    
    Left_WS.ControlAddress  = 0x500;
    Right_WS.ControlAddress = 0x520;
    
    scandal_send_ws_drive_command(0x523, 0, 0);
    
}



/* This is your main function! You should have an infinite loop in here that
 * does all the important stuff your node was designed for */
int main(void) {
    can_msg msg;
    uint8_t err;
    uint8_t txnum;
    uint8_t txbuf[20];
    uint8_t i;
    uint8_t chksum;
    uint8_t variableHold;
    float controllerOutput = 0;
    
    //int i;
    
    //err = can_get_msg(&msg);

	setup();
    register_standard_message_handler(std_msg_handler);
    
   
	/* Display welcome header over UART */
	//UART_printf("Welcome to the template project! This is coming out over UART0\n\r");
	//UART_printf("A debug LED should blink at a rate of 1HZ\n\r");

	sc_time_t one_sec_timer = sc_get_timer();


	/* This is the main loop, go for ever! */
    while (1) {
        handle_scandal();

		/* Send a UART and CAN message and flash an LED every second */
		if(sc_get_timer() >= one_sec_timer) {
			/* Send the message */
			UART_printf("Left Motor: %x \t Vel: %f \t LastUpd: %u \n\r", Left_WS.BaseAddress, Left_WS.MotorVelocity, (unsigned int) Left_WS.lastUpdateTime);
      UART_printf("Right Motor: %x \t Vel: %f \t LastUpd: %u\n\r", Right_WS.BaseAddress, Right_WS.MotorVelocity, (unsigned int) Right_WS.lastUpdateTime);
      
      controllerOutput = targetTorque(150, &Left_WS, &Right_WS);
      torqueControl(controllerOutput, &Left_WS, &Right_WS);
      if (controllerOutput < 0){
      UART_printf("NEG ");
      }
      UART_printf("CTRL = %1.3f\n\r", controllerOutput);
      
      //send_ws_drive_commands((float)120, (float)0.3, (float)0.3, &Left_WS);
      //send_ws_drive_commands((float)30, (float)0.3, (float)0.3, &Right_WS);
			
			/* Twiddle the LEDs */
			toggle_red_led();

			/* Update the timer to run 1000ms later */
			one_sec_timer = one_sec_timer + 100;
		}
        
        //WDT_Feed();
	}
}

void std_msg_handler(can_msg *msg){
    //UART_printf("STD_RCVD %d id= %d\r\n", sc_get_timer(), msg->id-0x420);
    uint16_t baseAddress = msg->id & 0x7E0;
    
    if(baseAddress == Left_WS.BaseAddress){
            scandal_store_ws_message(msg, &Left_WS);
    }else if(baseAddress == Right_WS.BaseAddress){
            scandal_store_ws_message(msg, &Right_WS);
    }else{
        UART_printf("Daaam, got some other message I wasn't expecting :S ID = 0x%x\r\n", msg->id);
    }
    //scandal_store_ws_message(msg, &Left_WS);
}

float targetTorque(int32_t targetRPM, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct){
  #define FIXED_POINT_MULT  1000000
  #define PROPORTIONAL_MAX  0.3*FIXED_POINT_MULT
  #define INTEGRAL_MAX      0.2*FIXED_POINT_MULT
  int32_t leftVel; //angular velocity of the LHS wheel (rpm)
  int32_t rightVel; //angular velocity of the RHS wheel (rpm)
  int32_t avgVel;
  int error;
  int proportinalVar;
  static int integralVar=0;
  float output;
  
  leftVel = (ws_1_struct->MotorVelocity) * FIXED_POINT_MULT;
  rightVel = (ws_2_struct->MotorVelocity) * FIXED_POINT_MULT;
  avgVel = (leftVel + rightVel)/2;
  error = (targetRPM * FIXED_POINT_MULT) - avgVel; //Difference multiplied by FIXED_POINT_MULTI
  UART_printf("LV = %u, RV = %u, AV = %u, ER = %d\n\r", leftVel, rightVel, avgVel, error>>10);
  proportinalVar = (error >> 10);
  integralVar = integralVar + (error >> 15);

  if (proportinalVar > PROPORTIONAL_MAX){
    proportinalVar = PROPORTIONAL_MAX;
  } else if (proportinalVar < (-PROPORTIONAL_MAX)){
    proportinalVar = -PROPORTIONAL_MAX;
  }
  
  if (integralVar > INTEGRAL_MAX){
    integralVar = INTEGRAL_MAX;
  } else if (integralVar < (-INTEGRAL_MAX)){
    integralVar = -INTEGRAL_MAX;
  }

  output = (float) proportinalVar + (float) integralVar;
  //UART_printf("OUT = %f\n\r", output);
  //UART_printf("RV = %d ,PV = %d, IV = %d, OUT = %f\n\r", (int) rightVel, (int) proportinalVar, (int) integralVar, (float) output);

  output = output / FIXED_POINT_MULT;
  return output;
}

int32_t torqueControl(float desiredTorque, Wavesculptor_Output_Struct *ws_1_struct, Wavesculptor_Output_Struct *ws_2_struct){
  #define WS20_SPEED_LIMIT  100
  #define WS22_SPEED_LIMIT  400
  int32_t deviceType;
  float sign;
  float leftTorque;
  float rightTorque;
  
  /* Note: Slip ratio can be calculated by (LeftSpeed / (LeftSpeed + RightSpeed)) or vice versa 0:All right, 0.5:Balanced ,1:All Left */
  if(desiredTorque < 0){
    sign = -1;
    leftTorque = -1.0 * desiredTorque;
    rightTorque = -1.0 * desiredTorque;
  }else{
    sign = 1;
    leftTorque = desiredTorque;
    rightTorque = desiredTorque;
  }
    
  /* Set motor speed limit depending on type of motor controller, WS22 is in RPM, WS20 is in m/s - send roughly RPM/4 */
  deviceType = check_device_type(ws_1_struct);
  
  if(deviceType == WS_22){
    send_ws_drive_commands(sign*WS22_SPEED_LIMIT, leftTorque, (float) 1.0, ws_1_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_SPEED_LIMIT, leftTorque, (float) 1.0, ws_1_struct);
  }


  deviceType = check_device_type(ws_2_struct);

  if(deviceType == WS_22){
    send_ws_drive_commands(sign*WS22_SPEED_LIMIT, rightTorque, (float) 1.0, ws_2_struct);
  }else if(deviceType == WS_20){
    send_ws_drive_commands(sign*WS20_SPEED_LIMIT, rightTorque, (float) 1.0, ws_2_struct);
  }

  /* Send command to each WS: send_ws_drive_commands(sign*MAX_SPEED, torque_command, (float)0.3, ws_1_struct); */
  return 0;
}