# eYourbike
一套电动化你的自行车的工具，包括了电机套件，电池套件，智能终端套件，以及机械套件。具有低成本，适应大多数自行车，高效率，客制化的特点。  
A set of tools to electrify your bike! Includes motor kit, battery kit, smart terminal kit, and mechanical components. Features: low cost, fits most bikes, high efficiency, and customizable.  

# 简介/Introducing
  最近我从上海搬到了柏林开始新的工作，这里的电动自行车选择实在是太少了，又贵又不咋地。所以我打算启动这个项目。该项目总体包括三个部分，一是机械部分，主要是起到传动，固定整个套件的作用。二是电气部分，包括电池管理，电源总管理，电机驱动，动能回收四个部分。三是软件部分，软件应该具有以下功能，一是基础的传感器数据收集反馈，二是电机的运动控制包括基础的驱动控制，动能回收启停，三是系统层次的电力管理，电池温度管理，四是智能交互。  
  Recently, I moved from Shanghai to Berlin for a new job and noticed there are very few e-bike options here—they’re expensive and not great. So I decided to start this project! The project has three main parts: Mechanical section: Handles drivetrain and mounting for all components. Electrical section: Includes battery management, power control, motor drive, and regenerative braking. Software section: Features sensor data collection, motor control (driving + regenerative braking start/stop), system-level power and battery temperature management, and smart interactions.  
  
  该项目将会具有以下特性： Key project features: 
  **高度客制化，高适应性** **Highly customizable & adaptable**  
  **模块化设计，可配置并组合各个零件** **Modular design—mix and match components freely**  
  **高达1C的充电速度** **1C fast charging speed**  
  **完善的安全措施，电池安全管理和电机紧急开路** **Robust safety measures—battery protection and emergency motor shutdown**  
  **动能回收系统** **Energy Recycle system**  
  **智能化，与你的智能设备联动** **Smart connectivity—works with your smart devices**  

# 机械设计 Mechanical Design
  本项目的机械部分我会尽量地使用3D打印制作，以最大化降低成本，为了适配性和传动效率，将选择链传动和同步带传动以适配不同的工作情况。机械部分主要为电池模组外壳及挂载套件，电机及驱动挂载套件，牙盘适配器，传感器挂载套件。我将设计不同配置的机械组件，例如不同的电机功率，不同的牙盘尺寸的适配器，不同电压和容量的电池模块等。   
  其次机械部分应该具有**全天候能力**，以在不同天气和环境下保护内部设备。  
  For the mechanical parts, I’ll use 3D printing as much as possible to keep costs low. To ensure compatibility and drivetrain efficiency, I’ll use both chain and belt drives for different scenarios. Main components include: battery module casing & mounts, motor & drive mounts, chainring adapters, and sensor mounts. I’ll design interchangeable parts for different motor powers, chainring sizes, and battery voltages/capacities.  
  The mechanical build should also be **all-weather ready** to protect internal components in any weather or environment.  
  **TBD**  
  
# 电气部分 Electric Design
  电气部分将是本项目的重点，首先是电池管理模块，考虑到安全性问题，我只会设计使用磷酸铁锂电池（LiFeP4)的电池模块，设计电压为36V和72V两种。  
  The electrical section is the project’s core. First, the battery management module: for safety, I’ll only design battery modules using LiFePO4 batteries, with voltages of 36V and 72V.  
  以上是电气部分架构图： 
  Here’s the electrical architecture diagram:  
  <img width="730" alt="{1CC823F9-7E03-422D-958A-7BCBED5B54A1}" src="https://github.com/user-attachments/assets/89a6fc43-7e76-4989-8376-16e244eeaed3" />   

  对于**充放电模块**的设计，考虑到我没有独立设计开关电源的能力，因此我计划直接使用现成的开关电源提供DC供电，主控使用esp32C3，该MCU能够直接产生PWM信号调节充电占空比，且具有ADC引脚能够测量电压，使用厚膜电阻进行采样，低容量时使用恒流充电，接近充满时候使用恒压充电模式，且具有通信接口与平衡模块进行通信，通过串口与上位MCU通信。同时应具有温度监测功能，考虑到ESP32C3的I2C通道仅有一个，因此将使用I2C多路转换IC，以监控多个传感器。  
  For**Charge/Discharge module**. Since I’m not designing a switch-mode power supply from scratch, I’ll use an off-the-shelf DC power supply. The main controller will be an ESP32-C3, which can generate PWM signals to adjust charging duty cycle, has ADC pins for voltage measurement, and uses thick-film resistors for current sampling. It’ll charge at constant current when the battery is low and switch to constant voltage as it nears full. It also has communication interfaces for the balancing module and upper MCU via UART. Temperature monitoring will use an I2C multiplexer since the ESP32-C3 only has one I2C channel, allowing multiple sensors to be connected.  
  
  对于**电机驱动器**的设计，我计划使用开源的FOC驱动板，并进行改进，该驱动器应该是有感的，以霍尔信号反馈为主，其次具有I2C传感器（如MT6701）接口，以适配更多电机。其次应该具有接受上位机信号的功能，包括速度指令，刹车指令，紧急开路指令，以及反馈信息给上位机的功能。主控计划采用ESP32S3，该MCU性能更为强大，且GPIO丰富。**TBD**  
  For**Motor Driver**. I plan to modify an open-source FOC driver board. It’ll be sensor-based, mainly using hall effect feedback, with an I2C sensor (like MT6701) interface for more motor compatibility. It needs to accept commands from the upper computer—speed, brake, emergency shutdown—and send feedback. The main controller here will be an ESP32-S3 for its stronger performance and more GPIOs. **TBD**  
  
  对于**主控及电源管理**的设计，主控计划采用ESP32S3，电源管理部分集成在主控板上。该控制板包括了交互部分，初步计划使用简单的可变电阻控制速度，一个简单的OLED屏幕和操作按钮，由于我在ui设计和上层软件设计上没有经验，所以我决定只添加几个简单功能，应急开路，速度控制扳手，定速巡航开关，当前速度显示，时间显示，电量显示，动能回收开关，模块参数识别。该板上集成DC-DC，为其他的IC供电，因此同时也应该定义电缆线接口（**TBD**）,该接口应该具有高可靠性，全天候能力，高耐用性。该板为通用套件。  
  For**Main Control & Power Management**. The main controller will also use an ESP32-S3, with power management integrated on the board. The control panel will include basic interaction: a variable resistor for speed control, a small OLED screen, and buttons. Since I’m new to UI and upper software design, I’ll keep it simple with features like emergency shutdown, speed control, cruise control, speed/time/battery display, Energy Recycle toggle, and module parameter detection. The board will have integrated DC-DC converters to power other ICs and a high-reliability cable interface (TBD) designed for all-weather and heavy use. This is a universal kit.  
  
  对于**动能回收板**的设计，使用AC-DC-DC的方案，三相电经整流滤波后进入双向DC-DC电路，经过BUCK或BOOST后的电流回流进主控及电源管理板，再回流入电池充放电模块实现动能回收。同样由ESP32C3作为主控，通过配置MCU参数来确定不同的电压，以适应不同的电池组和电机。同时该板应具有电流控制功能，以控制进行动能回收时刹车的力度，这一功能直接通过软件实现（**TBD**）。  
  For**Energy Recycle Board**. Using an AC-DC-DC scheme: three-phase power is rectified and filtered, then fed into a bidirectional DC-DC converter. After buck/boost conversion, current flows back to the main control board and into the battery charge/discharge module. An ESP32-C3 acts as the controller, adjusting parameters to fit different battery packs and motors. It’ll also have current control to adjust braking force during Energy Recycle, implemented purely through software (TBD).  

# 软件部分
  每块电路板上均有一个MCU，分别具有不同的功能，总软件架构图如下。  
  <img width="710" alt="{C0AA734E-62EA-47C2-82DF-CA57D2A1D70A}" src="https://github.com/user-attachments/assets/b4181596-98d3-4129-8d42-17358f2c7d4b" />  
  通信使用UART通信  

  
  
  
  
  
