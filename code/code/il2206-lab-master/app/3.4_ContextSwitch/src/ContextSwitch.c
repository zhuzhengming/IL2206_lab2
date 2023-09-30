// File: ContextSwitch.c 

#include <stdio.h>
#include "includes.h"
#include <string.h>
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_performance_counter.h"
#include "sys/alt_irq.h"
#include "sys/alt_alarm.h"
#include "system.h"

#define DEBUG 0

/* Definition of Task Stacks */
/* Stack grows from HIGH to LOW memory */
#define   TASK_STACKSIZE       2048
OS_STK    task1_stk[TASK_STACKSIZE];
OS_STK    task2_stk[TASK_STACKSIZE];
OS_STK    stat_stk[TASK_STACKSIZE];
OS_EVENT *Sem1;
OS_EVENT *Sem2;

int cycle = 0;
int counter = 0;
double ContextSwitchTime_us = 0;
double ContextSwitchTimeSum = 0;
double ContextSwitchTimeAverage = 0;



/* Definition of Task Priorities */
#define TASK1_PRIORITY      6  // highest priority
#define TASK2_PRIORITY      7
#define TASK_STAT_PRIORITY 12  // lowest priority 

void printContextSwitchTime(void){
      counter = perf_get_total_time((void *) PERFORMANCE_COUNTER_BASE);
      ContextSwitchTime_us = (double) counter / 50.0;
      printf("Context Switch time is : %f us \n",ContextSwitchTime_us);
      //calculate average time, remove  the values which differ so much to the average 20%
      if (ContextSwitchTime_us < ContextSwitchTimeAverage * 1.3 || ContextSwitchTimeAverage == 0 ){
        cycle += 1;
        ContextSwitchTimeSum += ContextSwitchTime_us;
        ContextSwitchTimeAverage = ContextSwitchTimeSum / cycle; 
      }

      // print average context switch time every 10 cycles
      if(cycle % 10 == 0){
        printf("average Context Switch time is : %f us \n",ContextSwitchTimeAverage );
      }
}
void printStackSize(char* name, INT8U prio) 
{
  INT8U err;
  OS_STK_DATA stk_data;
    
  err = OSTaskStkChk(prio, &stk_data);
  if (err == OS_NO_ERR) {
    if (DEBUG == 1)
      printf("%s (priority %d) - Used: %d; Free: %d\n", 
	     name, prio, stk_data.OSUsed, stk_data.OSFree);
  }
  else
    {
      if (DEBUG == 1)
	printf("Stack Check Error!\n");    
    }
}

/* Prints a message and sleeps for given time interval */
void task1(void* pdata)
{
  INT8U err;
  int i;
  
  while (1)
    { 
      char text1[] = "Task 0 - State 0 \n";
      for(i =0; i < strlen(text1); i++ ) {
        putchar(text1[i]);
      }

      OSSemPost(Sem2);

      // reset and start conut when blocking task1
      PERF_RESET(PERFORMANCE_COUNTER_BASE);
      PERF_START_MEASURING(PERFORMANCE_COUNTER_BASE);

      OSSemPend(Sem1, 0 , &err);

      strcpy(text1,"Task 0 - State 1 \n");
      
      for (i =0; i < strlen(text1); i++) {
        putchar(text1[i]);
      }

      OSTimeDlyHMSM(0, 0, 0, 10); 
      /* Context Switch to next task
				   * Task will go to the ready state
				   * after the specified delay
				   */
      
    }
}

/* Prints a message and sleeps for given time interval */
void task2(void* pdata)
{
  INT8U err;
  int i;
  while (1)
    { 
      OSSemPend(Sem2, 0 , &err);

      PERF_STOP_MEASURING(PERFORMANCE_COUNTER_BASE);
      printContextSwitchTime();

      char text2[] = "Task 1 - State 0 \n";
      for(i =0; i < strlen(text2); i++ ) {
        putchar(text2[i]);
      }

      strcpy(text2,"Task 1 - State 1 \n");

      for (i = 0; i < strlen(text2); i++){
        putchar(text2[i]);
      }
      OSSemPost(Sem1);

      OSTimeDlyHMSM(0, 0, 0, 10);
      
    }
}

/* Printing Statistics */
void statisticTask(void* pdata)
{
  while(1)
    {
      printStackSize("Task1", TASK1_PRIORITY);
      printStackSize("Task2", TASK2_PRIORITY);
      printStackSize("StatisticTask", TASK_STAT_PRIORITY);
    }
}

/* The main function creates two task and starts multi-tasking */
int main(void)
{
  printf("Lab 3 - Two Tasks\n");
  Sem1 = OSSemCreate(0);
  Sem2 = OSSemCreate(0);

  OSTaskCreateExt
    ( task1,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task1_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK1_PRIORITY,               // Desired Task priority
      TASK1_PRIORITY,               // Task ID
      &task1_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                                 
      );
	   
  OSTaskCreateExt
    ( task2,                        // Pointer to task code
      NULL,                         // Pointer to argument passed to task
      &task2_stk[TASK_STACKSIZE-1], // Pointer to top of task stack
      TASK2_PRIORITY,               // Desired Task priority
      TASK2_PRIORITY,               // Task ID
      &task2_stk[0],                // Pointer to bottom of task stack
      TASK_STACKSIZE,               // Stacksize
      NULL,                         // Pointer to user supplied memory (not needed)
      OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
      OS_TASK_OPT_STK_CLR           // Stack Cleared                       
      );  

  if (DEBUG == 1)
    {
      OSTaskCreateExt
	( statisticTask,                // Pointer to task code
	  NULL,                         // Pointer to argument passed to task
	  &stat_stk[TASK_STACKSIZE-1],  // Pointer to top of task stack
	  TASK_STAT_PRIORITY,           // Desired Task priority
	  TASK_STAT_PRIORITY,           // Task ID
	  &stat_stk[0],                 // Pointer to bottom of task stack
	  TASK_STACKSIZE,               // Stacksize
	  NULL,                         // Pointer to user supplied memory (not needed)
	  OS_TASK_OPT_STK_CHK |         // Stack Checking enabled 
	  OS_TASK_OPT_STK_CLR           // Stack Cleared                              
	  );
    }  

  OSStart();
  return 0;
}
