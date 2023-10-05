### Note

注：如果run.sh编译不过可以试试删除之前生成bsp文件，直接修改run.sh的APP_NAME就好了。

#### 2.1 **Literature**

- 阅读MicroC/OS-II的参考文档	
- hello_ucosii 提供的使用范例

#### 2.2 Question

1. When dealing with message queues does MicroC/OS-II provide a

   (a) non-blocking write?    **yes, OSQPost()**

   (b) blocking write with timeout? **No**

   (c) blocking write? **No**

   (d) non-blocking read?   **Yes OSQAccept()**

   (e) blocking read with timeout?   **Yes, OSQPend()**

   (f) blocking read?   **Yes, OSQPend(),  A timeout value of 0 indicates that the task wants to wait** 

   **forever for the message.**

2.  Which memory management functions can you use to allocate or free

   memory in MicroC/OS-II? How do they work and what is the advantage

   of these functions compared to usual C-functions as *malloc*?

   Functions: 

   - OSmemCreate()  
   - OSmemGet()  
   - OSMemPut()  
   - OSMemQuery() 

   Advantages:

   - faster and with predictable time cost.
   - can obtain fixed size adjacent memory blocks.
   - Avoid memory leaks because of fixed size memory block.
   
3. The function OSQPend returns a pointer void*.

   (a) What is a void*-pointer and how can it be used?

   - point to the address with received data, we can transfer the data type we want by a pointer with specific data type.

   (b) How can you regenerate the message that has been send using the void*-pointer.

   ```C
   int* pInt;
   pInt = (int*) OSQPend(myQueue, timeout, &err);
   if (err == OS_NO_ERR) {
       int receivedValue = *pInt; 
   }
   ```

   #### 3 RTOS Tasks

   ##### 3.1 **Accessing a Shared Resource: I/O**

   打开TwoTasks.c, 观察现象

   - What is the expected output of this program in the first glance?

     ```c
     // print 
     Lab 3 - Two Tasks
     Hello from Task1
     Hello from Task2
     // output stack usage
     Task1 (priority 6) - Used: ; Free: 
     Task2 (priority 7) - Used: ; Free: 
     StatisticTask (priority 12) - Used: ; Free: 
     ```

   - The program might not show the desired behavior. Explain why this might happen.

     - task 2 will be interupted by task 1 because of higher priority, and task 2 will excute from a new start.

   - What will happen if you take away OSTimeDlyHMSM() statements? Why?

     - task 1 will excute forever because of highest priority in the loop.

   - Semaphores can help you to get the desired program behavior. What are semaphores? How can one declare and create a semaphore in MicroC/OS-II?

     It's used for controling access to a shared resource.

   - How can a semaphore protect a critical section? Give a code example!

     - ```C
       OS_EVENT *resourceSem = OSSemCreate(1); 
       
       // task 1
       OSSemPend(resourceSem1, 0, &err); // wait signal
       OSSemPost(resourceSem2); // post signal
       
       // task 2
       OSSemPend(resourceSem2, 0, &err); // wait signal
       OSSemPost(resourceSem1); // wait signal
       ```
       
       

   - Who is allowed to lock and release semaphores? Can a task release a

     semaphore that was locked by another task?

     - every task can change semaphore.

   - Explain the mechanisms behind the command OSSemPost() and

     OSSemPend()
     
     - `OSSemPend()`: wait for a signal, if there is a signal, semephore value will minus 1, or block the task, can set the timeout value.
     - `OSSemPost()`: post a signal for the task which is ready to excute.

   ##### 3.2**Communication by Handshakes**

   修改TwoTasks.c，使得两个任务用handshaking的方式交互。

   - 使用是semaphore实现：

   - ```
     Task 0 - State 0
     Task 1 - State 0
     Task 1 - State 1
     Task 0 - State 1
     Task 0 - State 0
     Task 1 - State 0
     ```

   - 画示意图

   ##### 3.3 **Shared Memory Communication**

   - 修改Handshake.c实现：task 0 发送1开始的数字到task 1，task 1将数字乘以-1返回给task 0，使用同一个地址的数据。

   - ```
     Sending : 1
     Receiving : -1
     Sending : 2
     Receiving : -2
     ```

   - 画示意图

   ##### 3.4 **Context Switch**

   - 自己写一个定时器。

   - 在[Alt10, PER]参考。
   - 修改Handshake.c 来测试时间

#### 4. **Cruise Control Application**

- 包含以下部分：

  - Engine：给信号启动，如果速度为0会关闭。
  - Cruise Control：当TOP_GEAR激活时，可以保持恒速，如果速度大于20，则GAS_PEDAL和BRAKE_PEDAL就不能激活。
  - Gas Pedal: 可以给车加速，Cruise control变为未激活状态。
  - Brake： 可以给车减速，Cruise control变为未激活状态。
  - Gear(HIGH/LOW): 有high和low两种状态，为high的时候可以激活恒速，为low的时候取消恒速状态。

  对应的输出显示为下：

  ![image-20230925224509783](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925224509783.png)

- 系统有4个循环的task，需要跑2400m：

  ![image-20230925224712617](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925224712617.png)

  ![image-20230925224728015](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925224728015.png)

##### 4.1**Understanding the Initial Program**

- - 看代码框架cruise skeleton.c

  - 恒定油门为40
  - 不要修改taskVehicleTask，只有timer需要改。

##### 4.2 **Use Soft Timers to Implement Periodic Tasks**

- 结合soft timers和semaphore到task。
- 参考： OSTmrCreate in *µ*C/OS-II Reference Manual, Chapter 16.
- 记得选择硬件外部定时器。

##### 4.3 I/O Tasks

- 创建task **ButtonIO** 和 **SwitchIO**:
  - SwitchIO创建信号量：ENGINE, TOP_GEAR
  - ButtonIO创建信号量：CRUISE_CONTROL，GAS_PEDAL， BRAKE_PADAL.
  - 使用相应的LED灯显示状态：红灯激活，绿灯未激活。

##### 4.4 Control Law

- 刹车要在VehicleTask里并且要发送信息，而ControlTask发送油门量，因为刹车要有任意时候disable 油门的作用。

- 恒速下，速度至少为25时，速度偏离要为4，用LEDG0来表示恒速状态。

- 用show position和show target velocity. show positionLED表示位置：

  ![image-20230925230906075](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925230906075.png)

- show target velocity显示目标速度，显示在HEX4和HEX5，恒速未激活时显示为0。

##### 4.5 watchdog

- 添加看门狗和helper：如果没有overload就显示OK并喂狗，否则警告。
- 增加extra load,可调整，用SW4  到SW9，SW4是最低位，步长为2%, 超过50为100%，
- ![image-20230925231328785](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925231328785.png)

