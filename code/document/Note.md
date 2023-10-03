### Note

注：如果run.sh编译不过可以试试删除之前生成bsp文件，直接修改run.sh的APP_NAME就好了。

#### 2.1 **Literature**

- 阅读MicroC/OS-II的参考文档	
- hello_ucosii 提供的使用范例

#### 2.2 Question

1. When dealing with message queues does MicroC/OS-II provide a

   (a) non-blocking write?（立即完成）    **yes, OSQPost()**

   (b) blocking write with timeout?（等待一段时间） **No**

   (c) blocking write?（一直等待） **No**

   (d) non-blocking read?   **Yes OSQAccept()**

   (e) blocking read with timeout?   Yes, OSQPend()

   (f) blocking read?   **Yes, OSQPend(),  A timeout value of 0 indicates that the task wants to wait** 

   **forever for the message.**

2.  Which memory management functions can you use to allocate or free

   memory in MicroC/OS-II? How do they work and what is the advantage

   of these functions compared to usual C-functions as *malloc*?

   Functions: 

   - OSmemCreate()  创建固定大小blocks的储存空间
   - OSmemGet()  获得固定大小blocks的储存空间
   - OSMemPut()  释放固定大小blocks的储存空间
   - OSMemQuery()  获得储存空间的信息

   Advantages:

   - 时间固定而malloc和free操作时间不确定。
   - 可以获得固定大小相邻的内存块
   - 返回超时
   - 避免内存泄漏，分割

3. The function OSQPend returns a pointer void*.

   (a) What is a void*-pointer and how can it be used?

   - 指向一个接收数据的数据类型的地址，将其传入函数可以改变该地址的值。

   (b) How can you regenerate the message that has been send using the void*-pointer.

   ```C
   int* pInt;
   pInt = (int*) OSQPend(myQueue, timeout, &err);
   if (err == OS_NO_ERR) {
       int receivedValue = *pInt;
   }
   ```

   #### 3 RTOS Tasks

   ##### 3.1**Accessing a Shared Resource: I/O**

   打开TwoTasks.c, 观察现象

   - What is the expected output of this program in the first glance?

     - 一开始会输出：

     - ```c
       // print 
       Lab 3 - Two Tasks
       Hello from Task1
       Hello from Task2
       // output stack usage
       Task1 (priority 6) - Used: ; Free: 
       Task2 (priority 7) - Used: ; Free: 
       StatisticTask (priority 12) - Used: ; Free: 
       ```

       

   - The program might not show the desired behavior. Explain why this

     might happen.

     - 因为几个task都是无限循环，并且都带有延迟函数，task1具有最高的优先级，所以可能会出先task2打印更加频繁的情况。

   - What will happen if you take away OSTimeDlyHMSM() statements? Why?

     - 去掉延迟，几个任务都会不断的执行，由于task1 有更高的优先级，可能会一直保持在循环中。

   - Semaphores can help you to get the desired program behavior. What

     are semaphores? How can one declare and create a semaphore in

     MicroC/OS-II?

     semaphore适用于控制对共享资源的访问和同步任务的工具。

   - How can a semaphore protect a critical section? Give a code example!

     - ```C
       OS_EVENT *resourceSem = OSSemCreate(1); // 二进制信号量
       
       // 任务A
       OSSemPend(resourceSem, 0, &err); // 等待信号量
       // 临界区
       OSSemPost(resourceSem); // 释放信号量
       
       // 任务B
       OSSemPend(resourceSem, 0, &err); // 等待信号量
       // 临界区
       OSSemPost(resourceSem); // 释放信号量
       ```

       

   - Who is allowed to lock and release semaphores? Can a task release a

     semaphore that was locked by another task?

     - 任何任务都可以改变semaphore的值。

   - Explain the mechanisms behind the command OSSemPost() and

     OSSemPend()
     
     - `OSSemPend()`: 任务使用此函数请求或“pend”一个信号量。如果信号量计数大于零，它会减少计数并立即返回。如果计数为零，任务将被阻塞，直到另一个任务或中断发布信号量，或直到发生可选的超时。
     - `OSSemPost()`: 用于释放或“post”一个信号量。它增加了信号量计数。如果有任务在等待信号量，其中一个（基于优先级）将准备好运行。

   画出示意图。

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

   - 自己写一个定时器

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

- 添加看门狗：如果没有overload就显示OK，否则警告。
- 仔细阅读源文件：
- ![image-20230925231328785](C:\Users\Administrator\AppData\Roaming\Typora\typora-user-images\image-20230925231328785.png)

