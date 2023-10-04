/* Cruise control skeleton for the IL 2206 embedded lab
 *
 * Maintainers:  Rodolfo Jordao (jordao@kth.se), George Ungereanu (ugeorge@kth.se)
 *
 * Description:
 *
 *   In this file you will find the "model" for the vehicle that is being simulated on top
 *   of the RTOS and also the stub for the control task that should ideally control its
 *   velocity whenever a cruise mode is activated.
 *
 *   The missing functions and implementations in this file are left as such for
 *   the students of the IL2206 course. The goal is that they get familiriazed with
 *   the real time concepts necessary for all implemented herein and also with Sw/Hw
 *   interactions that includes HAL calls and IO interactions.
 *
 *   If the prints prove themselves too heavy for the final code, they can
 *   be exchanged for alt_printf where hexadecimals are supported and also
 *   quite readable. This modification is easily motivated and accepted by the course
 *   staff.
 */
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "system.h"
#include "includes.h"
#include "altera_avalon_pio_regs.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"

#define DEBUG 0
#define VEHICLE_PRINT 0 //whether print vehicle info or not.
#define MBOX_DEBUG 0
#define IO_DEBUG 0
#define STATUS_DEBUG 0
#define WATCHDOH_DEBUG 0

#define HW_TIMER_PERIOD 100 /* 100ms */

#define TURN_OFF 0x00
/* Button Patterns */

#define GAS_PEDAL_FLAG      0x08
#define BRAKE_PEDAL_FLAG    0x04
#define CRUISE_CONTROL_FLAG 0x02
/* Switch Patterns */

#define TOP_GEAR_FLAG       0x00000002
#define ENGINE_FLAG         0x00000001
#define TOP_GEAR_And_ENGINE_FLAG 0x00000003

/* LED Patterns */

#define LED_RED_0 0x00000001 // Engine
#define LED_RED_1 0x00000002 // Top Gear

#define LED_GREEN_0 0x0001 // Cruise Control activated
#define LED_GREEN_2 0x0004 // Cruise Control Button   rivised: LED_GREEN_2 is 0x0004 not 0x0002
#define LED_GREEN_4 0x0010 // Brake Pedal
#define LED_GREEN_6 0x0040 // Gas Pedal

#define LED_RED_12 0x1000     // [2000m, 2400m]
#define LED_RED_13 0x2000     // [1600m, 2000m)
#define LED_RED_14 0x4000     // [1200m, 1600m)
#define LED_RED_15 0x8000     // [800m, 1200m)
#define LED_RED_16 0x10000    // [400m, 800m)
#define LED_RED_17 0x20000    // [0m, 400m)

/*pid controler*/
typedef struct 
{
  INT8U kp;
  INT8U ki;
  INT8U kd;
  INT16S cur_error;
  INT16S cur_vel;
  INT16S ref_vel;
  INT16S output;
  INT16S error[2]; 
  INT16S output_max;
}PID_t;


/*
 * Definition of Tasks
 */

#define TASK_STACKSIZE 2048

OS_STK StartTask_Stack[TASK_STACKSIZE]; 
OS_STK ControlTask_Stack[TASK_STACKSIZE]; 
OS_STK VehicleTask_Stack[TASK_STACKSIZE];
OS_STK SwitchIOTask_Stack[TASK_STACKSIZE];
OS_STK ButtonIOTask_Stack[TASK_STACKSIZE];

OS_STK ExtraLoad_Stack[TASK_STACKSIZE];
OS_STK OverloadDetection_Stack[TASK_STACKSIZE];
OS_STK Watchdog_Stack[TASK_STACKSIZE];

// Task Priorities

#define STARTTASK_PRIO    5
#define VEHICLETASK_PRIO  7
#define CONTROLTASK_PRIO  10
#define SwitchIOTask_PRIO 8
#define ButtonIOTask_PRIO 9

#define OverloadTask_PRIO 12
#define ExtraloadTask_PRIO 11
#define Watchdog           6

// Task Periods

#define CONTROL_PERIOD  300
#define VEHICLE_PERIOD  300
#define SWITCHIO_PERIOD 300
#define BUTTONIO_PERIOD 300
#define OVERLOAD_PERIOD 300

/*
 * Definition of Kernel Objects 
 */

// Mailboxes
OS_EVENT *Mbox_Throttle;  //control task -> vehicle task
OS_EVENT *Mbox_Velocity;  //vehicle task -> control task; control task -> button task
OS_EVENT *Mbox_Brake;     //button task -> vehicle task
OS_EVENT *Mbox_Engine_to_vehicle;    //switch task -> vehicle task; 
OS_EVENT *Mbox_Engine_to_control;    //switch task -> control task
OS_EVENT *Mbox_Cruise;    //button task -> control task
OS_EVENT *Mbox_Top_gear;  //switch task -> control task
OS_EVENT *Mbox_Gas_pedel; //button task -> control task

// Semaphores
OS_EVENT *CONTROLTmrSem;
OS_EVENT *VEHICLETmrSem;
OS_EVENT *SwitchIOSem;
OS_EVENT *ButtonIOSem;
OS_EVENT *OverloadDetectionSem;
OS_EVENT *WatchdogSem;
OS_EVENT *ExtraLoadSem;

// SW-Timer
OS_TMR *CONTROLTmr;
OS_TMR *VEHICLETmr;
OS_TMR *SWITCHIOTmr;
OS_TMR *BUTTONIOTmr;
OS_TMR *OverloadTmr;

// pid controller init
void PID_init(PID_t* pid){
  pid->kp = 2;
  pid->ki = 0;
  pid->kd = 0;
  pid->error[0] = 0;
  pid->error[1] = 0;
  pid->output_max = 80;
}

// PID control
void PID_control(PID_t* pid, INT16S cur_velocity, INT16S ref_velocity){
INT16S derivative;
pid->cur_vel = cur_velocity;
pid->ref_vel = ref_velocity;
pid->cur_error = pid->ref_vel - pid->cur_vel;
derivative = pid->cur_error - pid->error[0];
// pid control
// pid->output = pid->kp*(pid->cur_error - pid->error[1]) + pid->ki*pid->cur_error + 
// pid->kd * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
// pid->error[0] = pid->error[1];
// pid->error[1] = pid->cur_error;

// pd control
pid->output = pid->kp * pid->cur_error + derivative * pid->kd;
pid->error[0] = pid->cur_error;
// set maxumum and minimum
if(pid->output > pid->output_max) pid->output = pid->output_max;
if(pid->output < -pid->output_max) pid->output = -pid->output_max;
}
/* Timer Callback Functions*/
void CONTROLTmrCallback(void *ptmr, void *callback_arg){
  OSSemPost(CONTROLTmrSem);
  if(DEBUG){
  printf("OSSemPost(CONTROLSem);\n");
  }
}

void VEHICLETmrCallback(void *ptmr, void *callback_arg){
  OSSemPost(VEHICLETmrSem);
  if(DEBUG){
  printf("OSSemPost(VEHICLESem);\n");
  }
}

void SWITCHIOTmrCallback(void *ptmr, void *callback_arg){
  OSSemPost(SwitchIOSem);
  if(DEBUG){
  printf("OSSemPost(SWITCHIOSem);\n");
  }
}

void BUTTONIOTmrCallback(void *ptmr, void *callback_arg){
  OSSemPost(ButtonIOSem);
  if(DEBUG){
  printf("OSSemPost(BUTTONIOSem);\n");
  }
}

// void OverloadTmrCallbask(void *ptmr, void *callback_arg){

// }
/*
 * Types
 */
enum active {on = 2, off = 1};

/*
 * Global variables
 */
int delay; // Delay of HW-timer 
INT16U led_green = 0; // Green LEDs
INT32U led_red = 0;   // Red LEDs


/*
 * Helper functions
 */
// capture buttons
int buttons_pressed(void)
{
  return ~IORD_ALTERA_AVALON_PIO_DATA(D2_PIO_KEYS4_BASE);    
}

// capture switches
int switches_pressed(void)
{
  return IORD_ALTERA_AVALON_PIO_DATA(DE2_PIO_TOGGLES18_BASE);    
}

/*
 * ISR for HW Timer
 */
alt_u32 alarm_handler(void* context)
{
  OSTmrSignal(); /* Signals a 'tick' to the SW timers */

  return delay;
}

static int b2sLUT[] = {0x40, //0
  0x79, //1
  0x24, //2
  0x30, //3
  0x19, //4
  0x12, //5
  0x02, //6
  0x78, //7
  0x00, //8
  0x18, //9
  0x3F, //-
};

/*
 * convert int to seven segment display format
 */
int int2seven(int inval){
  return b2sLUT[inval];
}

/*
 * output current velocity on the seven segement display
 */
void show_velocity_on_sevenseg(INT8S velocity){
  int tmp = velocity;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(velocity < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_LOW28_BASE,out);
}

/*
 * shows the target velocity on the seven segment display (HEX5, HEX4)
 * when the cruise control is activated (0 otherwise)
 */
void show_target_velocity(INT8S target_vel)
{
  int tmp = target_vel;
  int out;
  INT8U out_high = 0;
  INT8U out_low = 0;
  INT8U out_sign = 0;

  if(target_vel < 0){
    out_sign = int2seven(10);
    tmp *= -1;
  }else{
    out_sign = int2seven(0);
  }

  out_high = int2seven(tmp / 10);
  out_low = int2seven(tmp - (tmp/10) * 10);

  out = int2seven(0) << 21 |
    out_sign << 14 |
    out_high << 7  |
    out_low;
  
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_HEX_HIGH28_BASE,out);

}

/*
 * indicates the position of the vehicle on the track with the four leftmost red LEDs
 * LEDR17: [0m, 400m)
 * LEDR16: [400m, 800m)
 * LEDR15: [800m, 1200m)
 * LEDR14: [1200m, 1600m)
 * LEDR13: [1600m, 2000m)
 * LEDR12: [2000m, 2400m]
 */
void show_position(INT16U position)
{
  if(position >= 0 && position < 400)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_17); // LEDR17 ON
  else if (position >= 400 && position < 800)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_16); // LEDR16 ON
  else if (position >= 800 && position < 1200)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_15); // LEDR15 ON
  else if (position >= 1200 && position < 1600)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_14); // LEDR14 ON
  else if (position >= 1600 && position < 2000)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_13); // LEDR13 ON
  else if (position >= 2000 && position < 2400)
  IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_12); // LEDR12 ON
}

/*
 * The task 'VehicleTask' is the model of the vehicle being simulated. It updates variables like
 * acceleration and velocity based on the input given to the model.
 * 
 * The car model is equivalent to moving mass with linear resistances acting upon it.
 * Therefore, if left one, it will stably stop as the velocity converges to zero on a flat surface.
 * You can prove that easily via basic LTI systems methods.
 */
void VehicleTask(void* pdata)
{ 
  // constants that should not be modified
  const unsigned int wind_factor = 1;
  const unsigned int brake_factor = 4;
  const unsigned int gravity_factor = 2;
  // variables relevant to the model and its simulation on top of the RTOS
  INT8U err;  
  void* msg;
  INT8U* throttle; 
  INT16S acceleration;  
  INT16U position = 0; 
  INT16S velocity = 0; 
  enum active brake_pedal = off;
  enum active engine = off;

  printf("Vehicle task created!\n");

  while(1)
  {
    err = OSMboxPost(Mbox_Velocity, (void *) &velocity);

    OSSemPend(VEHICLETmrSem, 0, &err);

    /* Non-blocking read of mailbox: 
       - message in mailbox: update throttle
       - no message:         use old throttle
       */

    // from: control task
    msg = OSMboxPend(Mbox_Throttle, 1, &err); 
    if (err == OS_NO_ERR) {
      throttle = (INT8U*) msg;
      if(MBOX_DEBUG){
        printf("Throttle_msg recieve! \n");
    }
    }

    /* Same for the brake signal that bypass the control law
    from buttonIOtask */
    msg = OSMboxPend(Mbox_Brake, 1, &err); 
    if (err == OS_NO_ERR) {
      if(MBOX_DEBUG){
        printf("brake_msg recieve! \n");
    }
      brake_pedal = *(enum active*) msg;
    }

    /* Same for the engine signal that bypass the control law 
    from switchIOtask*/
    msg = OSMboxPend(Mbox_Engine_to_vehicle, 1, &err); 
    if (err == OS_NO_ERR) {
      if(MBOX_DEBUG){
        printf("engine_msg recieve! \n");
    }
      engine = *(enum active*)msg;
    }

    // vehichle cannot effort more than 80 units of throttle
    if (*throttle > 80) *throttle = 80;

    // brakes + wind
    if (brake_pedal == off)
    {
      // wind resistance
      acceleration = - wind_factor*velocity;
      // actuate with engines
      if (engine == on)
        acceleration += (*throttle);

      // gravity effects
      if (400 <= position && position < 800)
        acceleration -= gravity_factor; // traveling uphill
      else if (800 <= position && position < 1200)
        acceleration -= 2*gravity_factor; // traveling steep uphill
      else if (1600 <= position && position < 2000)
        acceleration += 2*gravity_factor; //traveling downhill
      else if (2000 <= position)
        acceleration += gravity_factor; // traveling steep downhill
    }
    // if the engine and the brakes are activated at the same time,
    // we assume that the brake dynamics dominates, so both cases fall
    // here.
    else 
      acceleration = - brake_factor*velocity;

    if(VEHICLE_PRINT){
    printf("Position: %d m\n", position);
    printf("Velocity: %d m/s\n", velocity);
    printf("Accell: %d m/s2\n", acceleration);
    printf("Throttle: %d V\n", *throttle);
    }
    if(STATUS_DEBUG)
    printf("velocity: %d \t engine: %d \t brake: %d \n",velocity, engine, brake_pedal);
    position = position + velocity * VEHICLE_PERIOD / 1000;
    velocity = velocity  + acceleration * VEHICLE_PERIOD / 1000.0;
    // reset the position to the beginning of the track
    if(position > 2400)
      position = 0;
    show_position(position);
    show_velocity_on_sevenseg((INT8S) velocity);
  }
} 

/*
 * The task 'ControlTask' is the main task of the application. It reacts
 * on sensors and generates responses.
 */

void ControlTask(void* pdata)
{
  INT8U err;
  INT8U throttle = 0; /* Value between 0 and 80, which is interpreted as between 0.0V and 8.0V */
  void* msg;
  INT16S* velocity_recv;
  INT16S current_velocity;
  INT16S target_velocity;
  PID_t PID;
  bool SET_Target = false;

  enum active ENGINE_status = off;
  enum active TOP_GEAR_status = off;
  enum active CRUISE_CONTROL_status = off;
  enum active GAS_PEDAL_status = off;

  printf("Control Task created!\n");
  PID_init(&PID);

  while(1)
  {
  //   // get current_velocity from Mbox_Velocity
    velocity_recv = (INT16S*)OSMboxPend(Mbox_Velocity, 0, &err);
    current_velocity =  *velocity_recv;

  //   // Here you can use whatever technique or algorithm that you prefer to control
  //   // the velocity via the throttle. There are no right and wrong answer to this controller, so
  //   // be free to use anything that is able to maintain the cruise working properly. You are also
  //   // allowed to store more than one sample of the velocity. For instance, you could define
  //   //
  //   // INT16S previous_vel;
  //   // INT16S pre_previous_vel;
  //   // ...
  //   //
  //   // If your control algorithm/technique needs them in order to function. 
    
     /* Non-blocking read of mailbox: */
    // from: control task

    /* engine signal from switchIOtask */
    msg = OSMboxPend(Mbox_Engine_to_control, 1, &err); 
    if (err == OS_NO_ERR) {
      if(MBOX_DEBUG){
        printf("engine_msg recieve! \n");
    }
      ENGINE_status = *(enum active*)msg;
    }

    //* cruise signal from buttonIOtask */
    msg = OSMboxPend(Mbox_Cruise, 1, &err); 
    if (err == OS_NO_ERR) {
      if(MBOX_DEBUG){
        printf("cruise_msg recieve! \n");
    }
      CRUISE_CONTROL_status = *(enum active*)msg;
    }
    
    //* gas_pedel signal from buttonIOtask */
    msg = OSMboxPend(Mbox_Gas_pedel, 1, &err); 
    if (err == OS_NO_ERR) {
      if(MBOX_DEBUG){
        printf("gas_pedel_msg recieve! \n");
    }
      GAS_PEDAL_status = *(enum active*)msg;
    }

    //* top_gear signal from switchIOtask */
    msg = OSMboxPend(Mbox_Top_gear, 1, &err); 
    if (err == OS_NO_ERR){
      if(MBOX_DEBUG){
        printf("Top_gear_msg recieve! \n");
    }
      TOP_GEAR_status = *(enum active*)msg;
    }

    // debug for mbox
    if(STATUS_DEBUG){
    printf("cruise_control:%d \t gas_pedal:%d \t top_gear:%d \n" 
    ,CRUISE_CONTROL_status, GAS_PEDAL_status, TOP_GEAR_status);
    }

    if(ENGINE_status == on){
      // speed up
      if(GAS_PEDAL_status == on){
        throttle = 80;
      }

      // cruise control
      if(TOP_GEAR_status == on)
      {
        if(CRUISE_CONTROL_status == on && current_velocity >= 25)
        {
          printf("cruise mode!!\n");
          IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_0);
            if(DEBUG) printf("cuise control active!, LEDG0: on\n");
            if(!SET_Target) { //set the target velocity
              target_velocity = current_velocity;
              SET_Target = true;
            }
            show_target_velocity((INT8S)target_velocity);
            PID_control(&PID, current_velocity, target_velocity);
            throttle = (INT8U)(throttle + PID.output);
            printf("output: %d \n", PID.output);

            if(target_velocity - current_velocity >4 
            || current_velocity - target_velocity >4) CRUISE_CONTROL_status == off;
        
        } 

        else
        { // cruise control deavtivate 
            CRUISE_CONTROL_status = off;
            show_target_velocity(0);
        }
      }
      else
      { // The cruise control is deactivated, when the gear position is moved to low.
          CRUISE_CONTROL_status = off;
      }

      
      if(CRUISE_CONTROL_status == off) {
        if(SET_Target) throttle = 10; // engine power
        SET_Target = false; //reset the flag
      }

      if(CRUISE_CONTROL_status == off && GAS_PEDAL_status == off){
        throttle = 10; // engine power
      }
    }

    // printf("current velocity: %d \t throttle value: %d \n",current_velocity, throttle);
    err = OSMboxPost(Mbox_Throttle, (void *) &throttle);
    OSSemPend(CONTROLTmr, 0, &err); 
  }
}

/* SwitchIO task: create the signals ENGINE and TOP_GEAR
    red LEDs: active
    ENGINE:   SW0   LEDR0
    TOP_GEAR: SW1   LEDR1 */
void SwitchIOTask(void* pdata){
  int SwitchValue;
  INT8U err;
  enum active ENGINE_status = off;
  enum active TOP_GEAR_status = off;

  printf("SwitchIO Task created!\n");
  while (1)
  {
    // read switch value:
    SwitchValue = switches_pressed();
    switch (SwitchValue)
    {
    case ENGINE_FLAG:
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0);
      if(IO_DEBUG){
        printf("ENGINE:on, LEDR0:on\n");
      }
      ENGINE_status = on;
      
      break;

    case TOP_GEAR_FLAG:
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_1);
      if(IO_DEBUG){
        printf("TOP_GEAR:on, LEDR1:on\n");
      }
      TOP_GEAR_status = on;
      break;

    case TOP_GEAR_And_ENGINE_FLAG:
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE, LED_RED_0+LED_RED_1);
     if(IO_DEBUG){
        printf("ENGINE:on, LEDR0:on \t");
        printf("TOP_GEAR:on, LEDR1:on\n");
      }
      ENGINE_status = on;
      TOP_GEAR_status = on;
      break;

    default:
    ENGINE_status = off;
    TOP_GEAR_status = off;
    if(IO_DEBUG){
        printf("no switch is pressed! \n");
      }
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_REDLED18_BASE,TURN_OFF);
      break;
    }

    // put engine signal into Mbox_engine_vehicle
    err = OSMboxPost(Mbox_Engine_to_vehicle, (void*)&ENGINE_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("engine_to_vehicle_msg post! \n");
    }
    }
    // put engine signal into Mbox_engine_control
    err = OSMboxPost(Mbox_Engine_to_control, (void*)&ENGINE_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("engine__to_control_msg post! \n");
    }
    }
    // put TOP_gear signal into Mbox_top_gear
    err = OSMboxPost(Mbox_Top_gear, (void*)&TOP_GEAR_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("TOP_gear_msg post! \n");
    }
    }

    OSSemPend(SwitchIOSem, 0, &err);
  }

}

/* ButtonIO task: create the signals CRUISE_CONTROL, GAS_PEDAL and BRAKE_PEDAL
   green LEDs: active
   CRUISE_CONTROL:   KEY1   LEDG2
   BRAKE_PEDAL:      KEY2   LEDG4 
   GAS_PEDAL:        KEY3   LEDG6*/
void ButtonIOTask(void* pdata){
  int ButtonValue;
  INT8U err;
  enum active CRUISE_CONTROL_status = off;
  enum active BRAKE_PEDAL_status = off;
  enum active GAS_PEDAL_status = off;

   printf("ButtonIO Task created!\n");

  while (1)
  {
    
    // read button value:
    ButtonValue = buttons_pressed();
    ButtonValue = (ButtonValue) & 0x0f;  // because the upper 16-bit of ButtonValue is 0xffff
    
    switch (ButtonValue)
    {
    case CRUISE_CONTROL_FLAG:
    // if(TOP_GEAR_status == on && current_velocity >= 25 
    // && GAS_PEDAL_status == off && GAS_PEDAL_status == off){
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_2);
      if(IO_DEBUG){
        printf("CRUISE_CONTROL:on, LEDG2:on\n");
      }
      CRUISE_CONTROL_status = on;
    // }
      break;

    case BRAKE_PEDAL_FLAG:
      IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_4); 
      if(IO_DEBUG){
        printf("BRAKE_PEDAL:on, LEDG4:on\n");
      }
      BRAKE_PEDAL_status = on;
      CRUISE_CONTROL_status = off;
      break;

    case GAS_PEDAL_FLAG:
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_6); 
    if(IO_DEBUG){
        printf("GAS_PEDAL:on, LEDG6:on\n");
      }
      GAS_PEDAL_status = on;
      CRUISE_CONTROL_status = off;
      break;
    
    default:
    GAS_PEDAL_status = off;
    BRAKE_PEDAL_status = off;
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE,TURN_OFF);
    if(IO_DEBUG){
        printf("no button is pressed! \n");
      }
    if(CRUISE_CONTROL_status == on) 
    IOWR_ALTERA_AVALON_PIO_DATA(DE2_PIO_GREENLED9_BASE, LED_GREEN_2);
      break;
    }

     // put cruise control signal into Mbox_Cruise
    err = OSMboxPost(Mbox_Cruise, (void*)&CRUISE_CONTROL_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("cruise_msg post! \n");
    }
    }
    // put TOP_gear signal into Mbox_Brake
    err = OSMboxPost(Mbox_Brake, (void*)&BRAKE_PEDAL_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("brake_msg post! \n");
    }
    }
    // put gas_padel signal into Mbox_Brake
    err = OSMboxPost(Mbox_Gas_pedel, (void*)&GAS_PEDAL_status);
    if(MBOX_DEBUG){
      if(err == OS_NO_ERR){ // successfully
        printf("gas_pedel_msg post! \n");
    }
    }

    OSSemPend(ButtonIOSem, 0, &err);
  }
}
/* 
 * The task 'StartTask' creates all other tasks kernel objects and
 * deletes itself afterwards.
 */ 

void StartTask(void* pdata)
{
  INT8U err;
  void* context;

  static alt_alarm alarm;     /* Is needed for timer ISR function */

  /* Base resolution for SW timer : HW_TIMER_PERIOD ms */
  delay = alt_ticks_per_second() * HW_TIMER_PERIOD / 1000; 
  printf("delay in ticks %d\n", delay);

  /* 
   * Create Hardware Timer with a period of 'delay' 
   */
  if (alt_alarm_start (&alarm,
        delay,
        alarm_handler,
        context) < 0)
  {
    printf("No system clock available!n");
  }

  /* 
   * Create and start Software Timer 
   */

  CONTROLTmr = OSTmrCreate(0, //initial delay
                        CONTROL_PERIOD/HW_TIMER_PERIOD, // period
                        OS_TMR_OPT_PERIODIC, // automatically reload itself
                        CONTROLTmrCallback, // callback function
                        (void *)0,
                        "CONTROLTmr",
                        &err);
  if(DEBUG){
    if(err == OS_ERR_NONE){ // create successfully
        printf("CONTROLTmr created \n");
    }
  }
  
  VEHICLETmr = OSTmrCreate(0, //initial delay
                        VEHICLE_PERIOD/HW_TIMER_PERIOD, // period
                        OS_TMR_OPT_PERIODIC, // automatically reload itself
                        VEHICLETmrCallback, // callback function
                        (void *)0,
                        "VEHICLETmr",
                        &err);
  if(DEBUG){
    if(err == OS_ERR_NONE){ // create successfully
        printf("VEHICLETmr created \n");
    }
  }

   SWITCHIOTmr = OSTmrCreate(0, //initial delay
                        SWITCHIO_PERIOD/HW_TIMER_PERIOD, // period
                        OS_TMR_OPT_PERIODIC, // automatically reload itself
                        SWITCHIOTmrCallback, // callback function
                        (void *)0,
                        "SWITCHIOTmr",
                        &err);
  if(DEBUG){
    if(err == OS_ERR_NONE){ // create successfully
        printf("CONTROLTmr created \n");
    }
  }
  
  BUTTONIOTmr = OSTmrCreate(0, //initial delay
                        BUTTONIO_PERIOD/HW_TIMER_PERIOD, // period
                        OS_TMR_OPT_PERIODIC, // automatically reload itself
                        BUTTONIOTmrCallback, // callback function
                        (void *)0,
                        "BUTTONIOTmr",
                        &err);
  if(DEBUG){
    if(err == OS_ERR_NONE){ // create successfully
        printf("BUTTONIOTmr created \n");
    }
  }


  // Start timers
  OSTmrStart(CONTROLTmr, &err);
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //start successful
      printf("CONTROLTmr started\n");
    }
   }

   OSTmrStart(VEHICLETmr, &err);
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //start successful
      printf("VEHICLETmr started\n");
    }
   }

  OSTmrStart(SWITCHIOTmr, &err);
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //start successful
      printf("SWITCHIOTmr started\n");
    }
   }

   OSTmrStart(BUTTONIOTmr, &err);
  if (DEBUG) {
    if (err == OS_ERR_NONE) { //start successful
      printf("BUTTONIOTmr started\n");
    }
   }

  /*
   * Creation of Kernel Objects
   */

  CONTROLTmrSem = OSSemCreate(0);
  VEHICLETmrSem = OSSemCreate(0);
  SwitchIOSem = OSSemCreate(0);
  ButtonIOSem = OSSemCreate(0);

  // Mailboxes
  Mbox_Throttle = OSMboxCreate((void*) 0); /* Empty Mailbox - Throttle */
  Mbox_Velocity = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Brake = OSMboxCreate((void*) 0); /* Empty Mailbox - Velocity */
  Mbox_Engine_to_vehicle = OSMboxCreate((void*) 0); /* Empty Mailbox - Engine */
  Mbox_Engine_to_control = OSMboxCreate((void*) 0);
  Mbox_Cruise = OSMboxCreate((void*) 0); /* Empty Mailbox - Cruise */
  Mbox_Gas_pedel = OSMboxCreate((void*) 0); /* Empty Mailbox - gas_pedel */
  Mbox_Top_gear = OSMboxCreate((void*) 0); /* Empty Mailbox - top_gear */

  /*
   * Create statistics task
   */

  OSStatInit();

  /* 
   * Creating Tasks in the system 
   */


  err = OSTaskCreateExt(
      ControlTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ControlTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      CONTROLTASK_PRIO,
      CONTROLTASK_PRIO,
      (void *)&ControlTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      VehicleTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &VehicleTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      VEHICLETASK_PRIO,
      VEHICLETASK_PRIO,
      (void *)&VehicleTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);
  
  err = OSTaskCreateExt(
      SwitchIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &SwitchIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      SwitchIOTask_PRIO,
      SwitchIOTask_PRIO,
      (void *)&SwitchIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  err = OSTaskCreateExt(
      ButtonIOTask, // Pointer to task code
      NULL,        // Pointer to argument that is
      // passed to task
      &ButtonIOTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack
      ButtonIOTask_PRIO,
      ButtonIOTask_PRIO,
      (void *)&ButtonIOTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,
      OS_TASK_OPT_STK_CHK);

  printf("All Tasks and Kernel Objects generated!\n");

  /* Task deletes itself */

  OSTaskDel(OS_PRIO_SELF);
}

/*
 *
 * The function 'main' creates only a single task 'StartTask' and starts
 * the OS. All other tasks are started from the task 'StartTask'.
 *
 */

int main(void) {

  printf("Lab: Cruise Control\n");

  OSTaskCreateExt(
      StartTask, // Pointer to task code
      NULL,      // Pointer to argument that is
      // passed to task
      (void *)&StartTask_Stack[TASK_STACKSIZE-1], // Pointer to top
      // of task stack 
      STARTTASK_PRIO,
      STARTTASK_PRIO,
      (void *)&StartTask_Stack[0],
      TASK_STACKSIZE,
      (void *) 0,  
      OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);

  OSStart();

  return 0;
}
