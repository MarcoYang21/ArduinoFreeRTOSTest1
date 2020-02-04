#include "Arduino.h"
//The setup function is called once at startup of the sketch

#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
//声明信互斥号量用它来管理串口
// It will be used to ensure only only one Task is accessing this resource at any time.
//某时刻只有一个任务能获取到串口通讯这个资源
SemaphoreHandle_t xSerialSemaphore;

// define two Tasks for DigitalRead & AnalogRead
//定义两个任务DigitalRead 和 AnalogRead
void TaskDigitalRead( void *pvParameters );
void TaskAnalogRead( void *pvParameters );
void TaskInerrupt0Effect( void *pvParameters );
void vLEDFlashTask( void *pvParameters);


volatile int state = LOW;
void blink()
{
  state = !state;
}
//设置中断
void setupInerrupt0()
{
  attachInterrupt(0, blink, CHANGE);
//attachInterrupt(pin, ISR, mode)
//mode:定义中断触发类型，有四种形式：
//LOW：低电平触发；
//CHANGE：电平变化触发；
//RISING ：上升沿触发（由LOW变为HIGH）；
//FALLING：下降沿触发（由HIGH变为LOW）；
//Due板子还支持高电平触发。
}

// the setup function runs once when you press reset or power the board
// 当上电或reset时setup函数只运行一次
void setup() {
  setupInerrupt0();
  // initialize serial communication at 9600 bits per second:
  //初始化串口连接波特率
  Serial.begin(115200);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
    //等待串口连接。需要LEONARDO, MICRO, YUN, 或其他 32u4板子上的usb接口
  }

  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  //信号量在需要让一个任务进程停一会儿时很有用，它让任务处于暂停等待的状态，
  // because it is sharing a resource, such as the Serial port.
  //就像串口这样的共享资源
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  //信号量应该只在调度运行的时候使用，在这里我们可以对它进行设置。
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.//检查确认串口信号没被创建。
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port//创建信号来管理串口
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.//调用xSemaphoreGive （“给”信号）使串口处于有效状态（没被占用）
  }

  // Now set up two Tasks to run independently.
  //现在设置两个任务互不影响的运行。
 xTaskCreate(
    TaskDigitalRead
    ,  (const portCHAR *)"DigitalRead"  // A name just for humans//只是人可识别的名称
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater堆栈大小可以被检查调整，通过读。。。
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.优先级3(configMAX_PRIORITIES - 1)最大0最小
    ,  NULL );

  xTaskCreate(
    TaskAnalogRead
    ,  (const portCHAR *) "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
  xTaskCreate(
    vLEDFlashTask
    ,  (const portCHAR *) "LEDFlash"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );
  xTaskCreate(
    TaskInerrupt0Effect
    ,  (const portCHAR *) "Inerrupt0Effect"
    ,  128  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL );
  // Now the Task scheduler, which takes over control of scheduling individual Tasks, is automatically started.
  //现在，任务调度程序开始自动控制调度单个任务，它将自动启动。


}


void loop()
{
  // Empty. Things are done in Tasks.
  //所有事情都在每个任务里做，这里留空。
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
// high priority for blinking LED
//闪灯任务高优先级
const uint8_t LED_PIN = 3;
void vLEDFlashTask(void *pvParameters __attribute__((unused)) )
{
  pinMode(LED_PIN, OUTPUT);
  // Flash led every 200 ms.
  //led每200ms闪烁一次
  for (;;)
{
    digitalWrite(LED_PIN, HIGH);    // Turn LED on.点亮led
    vTaskDelay((50L * configTICK_RATE_HZ) / 1000L);// Sleep for 50 milliseconds.//延时50ms
    pinMode(LED_PIN, INPUT);//设置针脚为input
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println(digitalRead(LED_PIN));
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.//释放串口给其他任务使用
    }
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW); // Turn LED off.关闭led
    vTaskDelay((150L * configTICK_RATE_HZ) / 1000L); // Sleep for 150 milliseconds.延时150ms
    pinMode(LED_PIN, INPUT);
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      Serial.println(digitalRead(LED_PIN));
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    pinMode(LED_PIN, OUTPUT);
    vTaskDelay(14);//1500ms
  }
}
void TaskDigitalRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  /*
    DigitalReadSerial
    Reads a digital input on pin 4, prints the result to the serial monitor
    This example code is in the public domain.
    读取pin2的数字输入，然后用串口打印出来
  */

  // digital pin 4 has a pushbutton attached to it. Give it a name:
  uint8_t pushButton = 4;

  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // read the input pin:
    int buttonState = digitalRead(pushButton);


    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
     Serial.print("Button:");
      Serial.println(buttonState);

      xSemaphoreGive( xSerialSemaphore );
    }

    vTaskDelay(5);
  }
}

void TaskAnalogRead( void *pvParameters __attribute__((unused)) )  // This is a Task.
{

  for (;;)
  {
    // read the input on analog pin 012:

int x,y,z,sw;
  x=analogRead(A0);
  y=analogRead(A1);
  sw= analogRead(A2);
  z= (sw<50) ?0:1;


    // See if we can obtain or "Take" the Serial Semaphore.
    //看是否能拿到使用串口的信号
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    //如果信号无效，调度程序等5个tick再看信号是否被释放
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 5 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the value you read:
      //我们能获取到了信号量，就可以访问共享资源。
//因为需要一些时间来输出信息，所以我们此时想单独占有串口资源，
//所以我们不希望串口资源在这个交互过程中被别的任务抢走。
//打印出所读的值：
      //Serial.println(sensorValue);
  Serial.print("X:");
  Serial.println(x);
  Serial.print(":Y:");
  Serial.println(y);
  Serial.print(":sw:");
  Serial.println(sw);
  Serial.print(":z:");
  Serial.println(z);
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.//释放使用完的串口资源
    }

    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability//为了稳定读取延时100tick ，1个tick 15ms
  }
}


void TaskInerrupt0Effect( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
 int pin = 13;
 pinMode(pin, OUTPUT);
 for (;;)
  {
  digitalWrite(pin, state);
  vTaskDelay(10);
  }
}
