/**********************************************************************************************************

TI EVALBOT code
Go-To-Goal Controller (P)
BME Homework
v1.0

István Gnyálin										

***********************************************************************************************************/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <includes.h>
#include "IQmathLib.h"

/*
*********************************************************************************************************
*                                             LOCAL DEFINES
*********************************************************************************************************
*/ 
#define DEST_X                  10.0f
#define DEST_Y                  0.0f
#define TURNLEFT	        0.52f           //30°
#define TURNRIGHT	        -1.57f          //-90°

#define P                       3.0f            //PID P <= 3.5
#define V                       50u             //Velocity
#define R                       1.15f           //Radius of wheel
//#define L                       8.5f            //Distance of wheels before calibration
#define L                       7.559f          //Distance of wheels
#define OFFSET                  0u              //Right motor not properly working

#define QEI_MAX_POSITION        5000u
#define QEI_START_POSITION      2000u

#define WHEEL_COE               0.225f          //2*R*3.14 / (TICKS/REVOLUTION) * 4
#define L_REC                   1.0f/L          
#define R_REC                   1.0f/R          

#define APP_TASK_DELAY          200u
#define QEI_READ_DELAY          500u
#define LED_BLINK_DELAY         1u

#define APP_TASK_START_PRIO     6u
#define QEI_READ_PRIO           5u
#define GTG_CONTROL_PRIO        5u
#define LED_BLINK_PRIO          6u

#define MSG_NUM                 10u
/*
*********************************************************************************************************
*                                            LOCAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB      AppTaskStartTCB;         //TCB button press and init
static  OS_TCB      QEIReadTask_TCB;         //TCB QEI Read
static  OS_TCB      LedBlinkTask_TCB;        //TCB led function
static  OS_TCB      GTGControlTask_TCB;      //TCB controller

static  CPU_STK     AppTaskStartStk[APP_TASK_ROBOT_STK_SIZE];
static  CPU_STK     QEIReadTask_Stk[APP_TASK_ROBOT_STK_SIZE];        
static  CPU_STK     LedBlinkTask_Stk[APP_TASK_ROBOT_STK_SIZE]; 
static  CPU_STK     GTGControlTask_Stk[APP_TASK_ROBOT_STK_SIZE]; 

/*
*********************************************************************************************************
*                                     LOCAL VARIABLES - GLOBAL
*********************************************************************************************************
*/
typedef  enum {                                                 /* States for the control task.                         */
  CONTROL_INIT = 0,
  CONTROL_START,
  CONTROL_GOING,
  CONTROL_STOP,
  CONTROL_TURN_LEFT,
  CONTROL_TURN_RIGHT,
} tControlTaskState;

CPU_INT08S  X[5] = "0000\0";              //X to print
CPU_INT08S  Y[5] = "0000\0";              //Y to print
CPU_INT08S  PHI[5] = "0000\0";            //Phi to print
CPU_INT08S  vel[5] = "0000\0";            //velocity to print
CPU_FP32   Velo = 0;                      //velocity uint
CPU_FP32   deltaS,deltaPhi;
CPU_FP32   x,y,Phi,dl,dr;                 //Dead reckoning változók

OS_MUTEX MyMutex;

/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart          (void  *p_arg);
static  void  QEIReadTask           (void  *p_arg);
static  void  LedBlinkTask          (void  *p_arg);
static  void  GTGControlTask        (void  *p_arg);

static  void  AppRobotTasksCreate   (void);
static  void  QEIConfig             (void);

/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none.
*
* Returns     : none.
*********************************************************************************************************
*/

int  main (void)
{
  OS_ERR  err;
  
  BSP_IntDisAll();                                            /* Disable all interrupts.     */
  
  OSInit(&err);                                               /* Init uC/OS-III.           */
  
  OSMutexCreate(&MyMutex,"TurningMutex",&err);                // Mutex for turning
  
  OSTaskCreate((OS_TCB     *)&AppTaskStartTCB,                /* initialize everything and monitor buttons         */
               (CPU_CHAR   *)"App Task Start",
               (OS_TASK_PTR ) AppTaskStart,
               (void       *) 0,
               (OS_PRIO     ) APP_TASK_START_PRIO,
               (CPU_STK    *)&AppTaskStartStk[0],
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE / 10u,
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE,
               (OS_MSG_QTY  ) 0u,
               (OS_TICK     ) 0u,
               (void       *) 0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR     *)&err);    
  OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none.
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg)
{
  CPU_INT32U  clk_freq;
  CPU_INT32U  cnts, counter, first;
  OS_ERR      err;
  CPU_TS      ts;
  tControlTaskState  eState;
  eState = CONTROL_INIT; 
  
  BSP_Init();                                                 /* Initialize BSP functions                             */
  CPU_Init();                                                 /* Initialize the uC/CPU services                       */
  clk_freq = BSP_CPUClkFreq();                                /* Determine SysTick reference freq.                    */
  cnts     = clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz;        /* Determine nbr SysTick increments                     */
  OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */
  CPU_TS_TmrFreqSet(clk_freq);
  
  //CPU Usage, ProbeInit
  
  AppRobotTasksCreate();              //Create remaining tasks                                                                         
  
  BSP_WheelSensorEnable();            //Enable IR Sensors on Board
  
  BSP_DisplayClear();
  BSP_DisplayStringDraw("Welcome!", 0u,0u);
  BSP_DisplayStringDraw("Press Button 1", 0u,1u);
  
  
  while (DEF_ON) {                                            /* Task body, always written as an infinite loop.       */
    OSTimeDlyHMSM(0u, 0u, 0u, APP_TASK_DELAY,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    switch (eState) {
    case CONTROL_INIT:
      if (BSP_PushButtonGetStatus(1u) == 0)
      {
        eState = CONTROL_START;
        counter = 0;
      } 
      break;
      
    case CONTROL_START:  
      
      first = 0;
      //Starting motors and init odometry
      x = y = Phi = deltaS = deltaPhi = 0.0;
      BSP_DisplayClear();
      
      BSP_DisplayStringDraw("X", 0u,0u);
      BSP_DisplayStringDraw("Y", 50u,0u);
      BSP_DisplayStringDraw("Ph", 0u, 1u);
      BSP_DisplayStringDraw("V", 50u, 1u); 
      
      BSP_MotorSpeed(LEFT_SIDE, 0<<8u);
      BSP_MotorDir(LEFT_SIDE, FORWARD);
      BSP_MotorRun(LEFT_SIDE);
      
      BSP_MotorSpeed(RIGHT_SIDE, 0<<8u);
      BSP_MotorDir(RIGHT_SIDE, FORWARD);
      BSP_MotorRun(RIGHT_SIDE);
      
      eState = CONTROL_GOING;
      
      break;
      
    case CONTROL_GOING: 
      //Displaying x,y,Phi,v
      sprintf(X,"%0.1f",x);
      sprintf(Y,"%0.1f",y);
      sprintf(PHI,"%0.1f",Phi);
      sprintf(vel,"%0.1f",Velo);
      
      BSP_DisplayStringDraw(X, 15u,0u);
      BSP_DisplayStringDraw(Y, 65u,0u);       
      BSP_DisplayStringDraw(PHI, 15u,1u);       
      BSP_DisplayStringDraw(vel, 65u,1u);
      
      if (((DEST_X - x) < 1.0) && ((DEST_Y - y) < 1.0 ))
      { 
        counter ++;
        if (counter == 12)
          eState = CONTROL_STOP;
        else
          eState = CONTROL_TURN_LEFT;
          
      }
      break;
      
    case CONTROL_STOP:  
      //Destination reached, stop motors
      BSP_DisplayClear();
      BSP_MotorStop(LEFT_SIDE);
      BSP_MotorStop(RIGHT_SIDE);
      BSP_DisplayStringDraw("Final Destination", 0u,0u);
      BSP_DisplayStringDraw("Press Button 1", 0u,1u);
      eState = CONTROL_INIT;
      break;
      
      
    case CONTROL_TURN_RIGHT:  
      if (first == 0)
      {
        OSMutexPend((OS_MUTEX *)&MyMutex, (OS_TICK) 0, (OS_OPT) OS_OPT_PEND_BLOCKING, (CPU_TS *) &ts, (OS_ERR *) &err); //Pend until mutex is released, no control task		
      
        BSP_MotorSpeed(LEFT_SIDE, 30<<8u);
        BSP_MotorDir(LEFT_SIDE, FORWARD);
        BSP_MotorRun(LEFT_SIDE);
      
        BSP_MotorSpeed(RIGHT_SIDE, 30<<8u);
        BSP_MotorDir(RIGHT_SIDE, REVERSE);
        BSP_MotorRun(RIGHT_SIDE);
        
        first = 1;
      }
      
      sprintf(PHI,"%0.1f",Phi);
      BSP_DisplayStringDraw(PHI, 15u,1u);
      
      if (TURNRIGHT - Phi > - 0.1f)
      {
        eState = CONTROL_START;
        OSMutexPost((OS_MUTEX *)&MyMutex, (OS_OPT) OS_OPT_POST_NONE, (OS_ERR *) &err); //Releasing mutex
      }	
      
      break;
      
      
      case CONTROL_TURN_LEFT:  
      if (first == 0)
      {
        OSMutexPend((OS_MUTEX *)&MyMutex, (OS_TICK) 0, (OS_OPT) OS_OPT_PEND_BLOCKING, (CPU_TS *) &ts, (OS_ERR *) &err); //Pend until mutex is released		
      
        BSP_MotorSpeed(LEFT_SIDE, 30<<8u);
        BSP_MotorDir(LEFT_SIDE, REVERSE);
        BSP_MotorRun(LEFT_SIDE);
      
        BSP_MotorSpeed(RIGHT_SIDE, 30<<8u);
        BSP_MotorDir(RIGHT_SIDE, FORWARD);
        BSP_MotorRun(RIGHT_SIDE);
        
        first = 1;
      }
      
      sprintf(PHI,"%0.1f",Phi);
      BSP_DisplayStringDraw(PHI, 15u,1u);
      
      if (TURNLEFT - Phi < 0.15f)
      {
        eState = CONTROL_START;
        OSMutexPost((OS_MUTEX *)&MyMutex, (OS_OPT) OS_OPT_POST_NONE, (OS_ERR *) &err); //Releasing mutex
      }	
      
      break;
    }
  }
}


//Reading QEI module
static void QEIReadTask(void *p_arg)
{
  OS_ERR err;
  CPU_FP32 Msg[3];
  
  CPU_INT64U mathxy, deltaxy;           
  CPU_INT16U leftQEI = 0;                   //QEI Position left
  CPU_INT16U rightQEI = 0;                  //QEI Position right
  CPU_INT32U leftVelo = 0;                  //QEI Velocity left
  CPU_INT32U rightVelo = 0;                 //QEI Velocity right
  CPU_INT16S Dl = 0;                        //Left position difference
  CPU_INT16S Dr = 0;                        //Right position difference
  
  x = y = Phi = deltaS = deltaPhi = 0.0;
  dl = dr = 0.0;
  mathxy = deltaxy = 0; 
  QEIConfig();
  while(1){
    OSTimeDlyHMSM(0u, 0u, 0u, QEI_READ_DELAY,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    mathxy = OS_TS_GET();
    rightQEI = QEIPositionGet(QEI0_BASE);
    leftQEI = QEIPositionGet(QEI1_BASE);
    QEIPositionSet(QEI0_BASE, QEI_START_POSITION );
    QEIPositionSet(QEI1_BASE, QEI_START_POSITION );
    Dl = leftQEI - 2000;
    Dr = rightQEI - 2000;
    rightVelo = QEIVelocityGet(QEI0_BASE);
    leftVelo = QEIVelocityGet(QEI1_BASE);
    
    dl = Dl * WHEEL_COE;
    dr = Dr * WHEEL_COE;
    
    deltaS = (dl + dr) * 0.5;
    deltaPhi = (dr - dl) * L_REC; 
    
    Phi = Phi + deltaPhi * 0.5;
    x = x + deltaS * _IQ10toF(_IQ10cos(_IQ10(Phi)));
    y = y + deltaS * _IQ10toF(_IQ10sin(_IQ10(Phi)));
    Phi = Phi + deltaPhi * 0.5;
    
    Velo = (rightVelo + leftVelo) * 0.5 * WHEEL_COE;
    
    deltaxy = OS_TS_GET() - mathxy; // deltaxy = 20 us
    
    Msg[0] = x;
    Msg[1] = y;
    Msg[2] = Phi;
    
    //Sending message to GTGController (internal message Q)
    OSTaskQPost((OS_TCB    *)&GTGControlTask_TCB,
                (void      *)&Msg[0],
                (OS_MSG_SIZE) 3u,
                (OS_OPT     ) OS_OPT_POST_FIFO,
                (OS_ERR    *)&err);       
    
  }
}

//GTG Controller
static void GTGControlTask(void *p_arg)
{
  CPU_FP32 dx, dy, Phi_d, e, w, vr, vl;
  CPU_INT16U ur, ul;
  OS_ERR err;
  OS_MSG_SIZE msg_size;
  CPU_FP32 *Msg;
  CPU_TS ts;
  CPU_INT64U period, deltaQ; 
  CPU_INT64U mathGTG, deltaGTG;  
  
  dx = dy = Phi_d = e = w = vr = vl =0.0;
  ur = ul = 0;
  while(1){
    Msg = (CPU_FP32  *)OSTaskQPend(0u,                 /* Pend until msg is received from switch monitor task. */
                                   OS_OPT_PEND_BLOCKING,
                                   &msg_size,
                                   &ts,
                                   &err);
    OSMutexPend((OS_MUTEX *)&MyMutex, (OS_TICK) 0, (OS_OPT) OS_OPT_PEND_BLOCKING, (CPU_TS *) &ts, (OS_ERR *) &err); //Pend until mutex is released
    //Delay counting ts
    period = BSP_CPUClkFreq();  //50 MHz         
    deltaQ = OS_TS_GET() - ts;  //deltaQ = 36 us
    
    mathGTG = OS_TS_GET();
    
    dx = DEST_X - Msg[0]; 
    dy = DEST_Y - Msg[1]; 
    Phi_d = _IQ10toF(_IQ10atan2(_IQ10(dy),_IQ10(dx)));
    e = Phi_d - Msg[2];
    e = _IQ10toF(_IQ10atan2(_IQ10sin(_IQ10(e)),_IQ10cos(_IQ10(e))));
    w = P * e;
    vr = V * R_REC + w * L * 0.5 *R_REC;
    vl = V * R_REC - w * L * 0.5 *R_REC;  
    ur = (CPU_INT16U) vr + OFFSET;
    ul = (CPU_INT16U) vl;
    
    BSP_MotorSpeed(LEFT_SIDE, ul<<8u);
    
    BSP_MotorSpeed(RIGHT_SIDE, ur<<8u);
    
    deltaGTG = OS_TS_GET() - mathGTG; // deltaGTG = 30 us
    
    OSMutexPost((OS_MUTEX *)&MyMutex, (OS_OPT) OS_OPT_POST_NONE, (OS_ERR *) &err); //Releasing mutex
  }
}

//Blink LED
static void LedBlinkTask(void *p_arg)
{
  OS_ERR err;
  while(1){
    OSTimeDlyHMSM(0u, 0u, LED_BLINK_DELAY , 0u,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
    BSP_LED_Toggle(1u);
    BSP_LED_Toggle(2u);
  }
}

static  void  AppRobotTasksCreate (void)
{
  OS_ERR  err;
  
  OSTaskCreate((OS_TCB     *)&QEIReadTask_TCB,                /* Read QEI Position and Velocity register     */
               (CPU_CHAR   *)"Read QEI Position and Velocity Register",
               (OS_TASK_PTR ) QEIReadTask,
               (void       *) 0,
               (OS_PRIO     ) QEI_READ_PRIO,
               (CPU_STK    *)&QEIReadTask_Stk[0],
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE / 10u,
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE,
               (OS_MSG_QTY  ) 0u,
               (OS_TICK     ) 0u,
               (void       *) 0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR     *)&err);    
  OSTaskCreate((OS_TCB     *)&LedBlinkTask_TCB,                /* LED Blink       */
               (CPU_CHAR   *)"LED Blink",
               (OS_TASK_PTR ) LedBlinkTask,
               (void       *) 0,
               (OS_PRIO     ) LED_BLINK_PRIO,
               (CPU_STK    *)&LedBlinkTask_Stk[0],
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE / 10u,
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE,
               (OS_MSG_QTY  ) 0u,
               (OS_TICK     ) 0u,
               (void       *) 0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR     *)&err);    
  OSTaskCreate((OS_TCB     *)&GTGControlTask_TCB,                /* GoToGoal Controller       */
               (CPU_CHAR   *)"GoToGoal Controller",
               (OS_TASK_PTR ) GTGControlTask,
               (void       *) 0,
               (OS_PRIO     ) GTG_CONTROL_PRIO,
               (CPU_STK    *)&GTGControlTask_Stk[0],
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE / 10u,
               (CPU_STK_SIZE) APP_TASK_ROBOT_STK_SIZE,
               (OS_MSG_QTY  ) MSG_NUM,
               (OS_TICK     ) 0u,
               (void       *) 0,
               (OS_OPT      )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
               (OS_ERR     *)&err);    
}

static  void  QEIConfig (void)
{
  unsigned long   system_clk;
  
  // Set the clocking to run directly from the crystal.
  
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
  
  GPIOPinConfigure(GPIO_PE2_PHA0);
  GPIOPinConfigure(GPIO_PE3_PHA1);
  GPIOPinConfigure(GPIO_PC7_PHB0);
  GPIOPinConfigure(GPIO_PG7_PHB1);
  
  
  GPIOPinTypeQEI( GPIO_PORTE_BASE, GPIO_PIN_2 );   // PhA0
  GPIOPinTypeQEI( GPIO_PORTE_BASE, GPIO_PIN_3 );   // PhA1
  
  GPIOPinTypeQEI( GPIO_PORTC_BASE, GPIO_PIN_7 );   // PhB0
  GPIOPinTypeQEI( GPIO_PORTG_BASE, GPIO_PIN_7 );   // PhB1
  
  // Configure the right encoder
  QEIConfigure( QEI0_BASE,
               ( QEI_CONFIG_CAPTURE_A_B   |   // capture both edges
                QEI_CONFIG_NO_RESET      |   // ignore index input
                  QEI_CONFIG_QUADRATURE    |   // not clock/dir mode
                    QEI_CONFIG_NO_SWAP ),        // PhA and PhB not swapped
               QEI_MAX_POSITION );
  
  // Configure the left encoder
  QEIConfigure( QEI1_BASE,
               ( QEI_CONFIG_CAPTURE_A_B   |   // capture both edges
                QEI_CONFIG_NO_RESET      |   // ignore index input
                  QEI_CONFIG_QUADRATURE    |   // not clock/dir mode
                    QEI_CONFIG_NO_SWAP ),        // PhA and PhB not swapped
               QEI_MAX_POSITION );
  
  //Clock rate of module
  system_clk = SysCtlClockGet();
  
  // Configure the right encoder's velocity module to return ticks/second
  QEIVelocityConfigure( QEI0_BASE,
                       QEI_VELDIV_1,
                       system_clk );
  
  // Configure the left encoder's velocity module
  QEIVelocityConfigure( QEI1_BASE,
                       QEI_VELDIV_1,
                       system_clk );
  
  
  // Enable the right encoder's velocity module
  QEIVelocityEnable( QEI0_BASE );
  
  // Enable the left encoder's velocity module
  QEIVelocityEnable( QEI1_BASE );
  
  //Starting position
  QEIPositionSet(QEI0_BASE, QEI_START_POSITION );
  QEIPositionSet(QEI1_BASE, QEI_START_POSITION );
  
  // Enable the right encoder
  QEIEnable( QEI0_BASE );
  
  // Enable the left encoder
  QEIEnable( QEI1_BASE );
  
  
}