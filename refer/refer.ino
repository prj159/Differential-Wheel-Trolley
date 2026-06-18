#include<MsTimer2.h>//定时器库的头文件
#include<Servo.h>//舵机库的头文件

//----------------------------------定义管脚----------------------------------
#define ENCODER_A1 2  //电机 1 (左)
#define ENCODER_B1 4
#define ENCODER_A2 3  //电机 2 (右)
#define ENCODER_B2 5

#define PWM1 11
#define DIR1 6       // 左电机方向
#define PWM2 12
#define DIR2 7       // 右电机方向

// 定义8个光电管 (从左到右 L0 到 L7)
#define L0 A7     
#define L1 A6
#define L2 A5
#define L3 A4     
#define L4 A3
#define L5 A2
#define L6 A1
#define L7 A0

// 舵机引脚
#define Servo_PIN1 8
// #define Servo_PIN2 9
// #define Servo_PIN3 10

//----------------------------------定义常值----------------------------------
#define PERIOD 10

// 电机速度环 PID 参数
#define Kp 15.0
#define Ti 70.0
#define Td 15.0  // 原30，减半以抑制10ms下的微分放大噪声

// 转向环 PD 参数 (权重模式专属，需要根据实际情况微调)
#define Kp_steer 0.3  // 转向比例系数：决定转弯的猛烈程度 (已调至原来的2倍) 0.3
#define Kd_steer 0.40  // 转向微分系数：适配10ms控制周期，由于 dError 变小，原0.10放大至0.20

#define V 3.0       // 基础速度
#define MAX_V 5.0   // 基础速度上限 (当一侧电机超速时，保证差速整体回缩)

//----------------------------------全局变量----------------------------------
// 舵机定义
Servo myservo1; // 舵机1控制机械臂转动
// Servo myservo2; // 舵机2控制夹爪转动
// Servo myservo3; // 舵机3控制夹爪张开

float target1 = 2.0, t1;    //左 目标速度
float target2 = 2.0, t2;    //右 目标速度
volatile long encoderVal1;  //编码器1值
float velocity1;            //转速1
volatile long encoderVal2;  //编码器2值
float velocity2;            //转速2

// PID 控制器相关中间变量
float T = PERIOD;
float q0 = Kp * (1 + T / Ti + Td / T);
float q1 = -Kp * (1 + 2 * Td / T);
float q2 = Kp * Td / T;
float u1, ek11, ek12;
float u2, ek21, ek22;

// 状态机与寻线相关变量
typedef enum {
    STRAIGHT,
    GENTLE_CURVE,
    SHARP_TURN,
    EDGE,
    RECOVERING,
    OUTLINE_DEFAULT,
    OUTLINE_SHARP
} STATE;

STATE current_state = STRAIGHT;
STATE last_state = STRAIGHT;
int SHARPlastside = 0;
int OUTCounter = 0;
int recCounter = 0;
float last_reliable_error = 0;

#define sharpROT 8.0

float previous_error = 0;   // 记录上一次的偏差 (用于求微分)

bool stopped = false;        // 全黑线停车标志

volatile bool started = false;        // 启动序列完成标志（先复位舵机、再夹取、再出发）
volatile bool servo_reset_pending = false;  // 全黑线停车后触发的舵机复位请求

// 【新增】用于在主循环中打印调试的全局变量
int sensorState[8] = {0};   // 存储 8 个光电管的实时状态
float current_error_out = 0;// 存储实时计算出的加权偏差值

//----------------------------------测速与转向控制逻辑----------------------------------
void control(void)
{
  // 启动前保持电机静止
  if (!started) {
    target1 = target2 = t1 = t2 = 0;
    return;
  }

  // 1. 读取 8 个传感器的状态
  sensorState[0] = (digitalRead(L0) == LOW) ? 1 : 0;
  sensorState[1] = (digitalRead(L1) == LOW) ? 1 : 0;
  sensorState[2] = (digitalRead(L2) == LOW) ? 1 : 0;
  sensorState[3] = (digitalRead(L3) == LOW) ? 1 : 0;
  sensorState[4] = (digitalRead(L4) == LOW) ? 1 : 0;
  sensorState[5] = (digitalRead(L5) == LOW) ? 1 : 0;
  sensorState[6] = (digitalRead(L6) == LOW) ? 1 : 0;
  sensorState[7] = (digitalRead(L7) == LOW) ? 1 : 0;

  // 1.5 过滤不连续的噪点灯条，只保留与上一帧重合度最高的有效段
  int currentLine[8];
  for(int i = 0; i < 8; i++) {
    currentLine[i] = (sensorState[i] == 0) ? 1 : 0; // 1表示压线
  }

  int segments[4][2]; // 存储段的起点和终点
  int num_segments = 0;
  int in_segment = 0;
  for(int i = 0; i <= 8; i++) {
    if(i < 8 && currentLine[i] == 1) {
      if(!in_segment) {
        segments[num_segments][0] = i;
        in_segment = 1;
      }
    } else {
      if(in_segment) {
        segments[num_segments][1] = i - 1;
        num_segments++;
        in_segment = 0;
      }
    }
  }

  static int prevLine[8] = {0}; // 记录上一帧的有效灯条
  if(num_segments > 1) {
    int max_overlap = -1;
    int best_segment = 0;
    for(int s = 0; s < num_segments; s++) {
      int overlap = 0;
      for(int i = segments[s][0]; i <= segments[s][1]; i++) {
        if(prevLine[i] == 1) overlap++;
      }
      // 选择重合度最大的段；如果重合度相同，则选择更宽的段
      if(overlap > max_overlap || (overlap == max_overlap && (segments[s][1] - segments[s][0]) > (segments[best_segment][1] - segments[best_segment][0]))) {
        max_overlap = overlap;
        best_segment = s;
      }
    }
    // 舍去非最佳段的灯条
    for(int i = 0; i < 8; i++) {
      if(i >= segments[best_segment][0] && i <= segments[best_segment][1]) {
        // 保留
      } else {
        currentLine[i] = 0;
        sensorState[i] = 1; // 恢复为无效状态
      }
    }
  }

  // 更新上一帧记录
  for(int i = 0; i < 8; i++) {
    prevLine[i] = currentLine[i];
  }

  // 2. 统计传感器信息 (兼容原版 sensorState[i] == 0 为压线的逻辑)
  float centroid = 0;
  int LEDCounter = 0;
  int LCounter = 0, RCounter = 0; 
  int Lmost = 8, Rmost = -1;

  for(int i = 0; i < 8; i++) {
    if(sensorState[i] == 0) { // == 0 表示压线
      if(i < 4) LCounter++; else RCounter++;
      if(i < Lmost) Lmost = i;
      if(i > Rmost) Rmost = i;
      centroid += i;
      LEDCounter++;
    }
  }
  static float last_valid_centroid = 3.5;
  if(LEDCounter > 0) {
    centroid /= LEDCounter;
    last_valid_centroid = centroid;
  } else {
    centroid = last_valid_centroid; // 丢线降噪期间，冻结质心为上一有效位置
  }

  // 2.5 丢线恢复侧过滤 (防误识别邻道线)
  // 当处于丢线状态且刚重新看到线时，判断线出现的位置是否合理。
  if (LEDCounter > 0 && (current_state == OUTLINE_SHARP || current_state == OUTLINE_DEFAULT)) {
    // last_reliable_error > 0 表示丢线前线在左侧，车正往左转找线，线理应从左侧(L0~L3)回到视野
    // 如果 centroid > 3.5 (说明线在右半边)，则大概率是扫描到了旁边赛道的线或是干扰，应当忽略。
    // 这里设置 > 1.0 和 < -1.0 的容差，避免直道颠簸跳线时被误杀。
    if (last_reliable_error > 1.0 && centroid > 3.5) {
      LEDCounter = 0; // 无视右侧的线，继续保持丢线寻找状态
      // 【修复】清空 prevLine，防止 1.5 节的噪点过滤器在下一帧锁定此错误线
      for(int i = 0; i < 8; i++) prevLine[i] = 0;
    } else if (last_reliable_error < -1.0 && centroid < 3.5) {
      LEDCounter = 0; // 无视左侧的线，继续保持丢线寻找状态
      // 【修复】清空 prevLine，防止 1.5 节的噪点过滤器在下一帧锁定此错误线
      for(int i = 0; i < 8; i++) prevLine[i] = 0;
    }
  }

  // 3. 状态机判断
  if(LEDCounter == 0) {
    OUTCounter++;
    if(OUTCounter >= 8) { // 连续8帧没看到线，才真正判定为丢线(出线)
      if(current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT) {
        if(last_state == SHARP_TURN || last_state == RECOVERING) {
          current_state = OUTLINE_SHARP;
        } else {
          current_state = OUTLINE_DEFAULT;
        }
      }
    } else {
      current_state = last_state; // 降噪：8帧以内保持原有状态继续执行
    }
  } else {
    OUTCounter = 0;
    if(abs(LCounter - RCounter) <= 2 && LEDCounter <= 3 && Lmost != 0 && Rmost != 7) {
      current_state = STRAIGHT;
    } else if(((LCounter >= 3 && Lmost == 0 && Rmost != 7) ||  
               (RCounter >= 3 && Rmost == 7 && Lmost != 0))
              && LEDCounter >= 4) {
      current_state = SHARP_TURN;
      if(last_state != SHARP_TURN) SHARPlastside = 0;
      SHARPlastside += LCounter > RCounter ? 1 : -1; // 左侧压线加1，右侧减1
    } else if(LEDCounter >= 3) {
      current_state = GENTLE_CURVE;
    } else if(LEDCounter >= 1) {
      current_state = EDGE;
    } else {
      current_state = STRAIGHT; 
    }
  }

  // 道路交叉 / 全黑线停车
  if(LEDCounter == 8) {
    stopped = true;           // 触发停车标志
    servo_reset_pending = true; // 触发舵机复位
    target1 = 0;
    target2 = 0;
    t1 = 0;
    t2 = 0;
    current_state = STRAIGHT;
    SHARPlastside = 0;
  }

  // 状态恢复检测：从急弯转出至RECOVERING
  if((last_state == SHARP_TURN  && current_state != SHARP_TURN && current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT) 
    || (last_state == OUTLINE_SHARP && current_state != OUTLINE_SHARP && current_state != OUTLINE_DEFAULT)) {
    current_state = RECOVERING;
    recCounter = 0;
  }
  
  // RECOVERING状态计时
  if(last_state == RECOVERING) {
    recCounter++;
    if(current_state != SHARP_TURN && current_state != OUTLINE_DEFAULT && current_state != OUTLINE_SHARP) current_state = RECOVERING; 
    if(recCounter >= 8) { // 恢复 8*10=80ms 后回到正常状态
      current_state = STRAIGHT;
      SHARPlastside = 0;
    }
  }

  last_state = current_state;
  if(current_state != SHARP_TURN && current_state != OUTLINE_SHARP && current_state != RECOVERING) SHARPlastside = 0;

  // 4. 计算输出偏差与状态基础速度比例
  float result = 0;
  float speed_ratio = 1.0;
  float base_error = (3.5 - centroid); // 基础偏差 [-3.5, 3.5]，左偏为正

  // 记录最近20个有效帧的历史偏差
  static float history_error[20] = {0};
  static int history_idx = 0;
  static int history_count = 0;
  
  if (LEDCounter > 0) {
    history_error[history_idx] = base_error;
    history_idx = (history_idx + 1) % 20;
    if (history_count < 20) history_count++;
  }
  
  switch(current_state) {
    case STRAIGHT:
      result = base_error * 1.3;
      speed_ratio = 1.0;
      break;
    case GENTLE_CURVE:
      result = base_error * 1.8;
      speed_ratio = 0.8;
      break;
    case SHARP_TURN:
      if(SHARPlastside > 0) result = sharpROT;
      else if(SHARPlastside < 0) result = -sharpROT;
      else result = 0;
      speed_ratio = 0.2;
      break;
    case OUTLINE_SHARP:
    case OUTLINE_DEFAULT:
      {
        float avg_error = 0;
        if (history_count > 0) {
          for (int i = 0; i < history_count; i++) {
            avg_error += history_error[i];
          }
          avg_error /= history_count;
        }
        
        // 出线后，根据出现前20个有效帧的均值方向，打死舵 (sharpROT)
        if (avg_error > 0) result = sharpROT;
        else if (avg_error < 0) result = -sharpROT;
        else result = sharpROT; // 均值为0或无历史时默认给一个方向防死锁
        
        speed_ratio = 0.2;
      }
      break;
    case EDGE:
      result = base_error * 2.3;
      speed_ratio = 0.8;
      break;
    case RECOVERING:
      result = base_error * 1.5;  //0.8
      speed_ratio = 0.8; // 恢复期保持相对稳定的基础速度
      break;
  }

  // 限制速度比例范围
  //if (speed_ratio < 0.2) speed_ratio = 0.2;
  if (speed_ratio > 1.0) speed_ratio = 1.0;

  if(LEDCounter > 0) {
    last_reliable_error = result;
  }
  if(LEDCounter == 0) speed_ratio = 0.2;

  current_error_out = result; // 将偏差赋值给全局变量供串口打印

  // 5. 转向 PD 控制计算转向调整量 (turn_adjust)
  float raw_dError = result - previous_error; // 偏差的变化率
  static float filtered_dError = 0;
  // 对微分项进行低通滤波，吸收由于光电管离散引起的 result 阶跃突变（消除转向高频抖动）
  filtered_dError = filtered_dError * 0.6 + raw_dError * 0.4; 
  float turn_adjust = (Kp_steer * result) + (Kd_steer * filtered_dError);
  previous_error = result; // 保存本次偏差供下次计算微分使用

  // 6. 根据状态机分配的 speed_ratio 计算动态基础速度
  float current_V = V * speed_ratio;

  // 根据动态基础速度和转向调整量，计算左右轮目标速度
  target1 = current_V - turn_adjust; 
  target2 = current_V + turn_adjust;

  
  // 限制一侧电机达到上限时，保持左右电机差速不变，将较高一侧降到上限
  if (target1 > MAX_V) {
    float diff = target1 - MAX_V;
    target1 -= diff;
    target2 -= diff;
  } else if (target2 > MAX_V) {
    float diff = target2 - MAX_V;
    target1 -= diff;
    target2 -= diff;
  }

  // 7. 对目标速度进行低通滤波 (适配 10ms 周期，保持相同时间常数)
  target1 = target1 * 0.37 + t1 * 0.63;
  target2 = target2 * 0.37 + t2 * 0.63;

  // ----------------------- 测速与电机驱动输出 -----------------------
  float raw_vel1 = (encoderVal1 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal1 = 0;
  // 对测速加入低通滤波，解决10ms下编码器脉冲数太少导致的量化噪声（消除电机高频震荡）
  velocity1 = velocity1 * 0.6 + raw_vel1 * 0.4;

  float raw_vel2 = -(encoderVal2 / 780.0) * 3.1415 * 2.0 * (1000 / PERIOD);
  encoderVal2 = 0;
  velocity2 = velocity2 * 0.6 + raw_vel2 * 0.4;

  // 计算左电机 PID 输出并控制方向
  int output1 = pidController1(target1, velocity1);
  int output2 = pidController2(target2, velocity2);

  if (stopped) {
    output1 = 0;
    output2 = 0;
    u1 = 0;
    u2 = 0;
  }

  if (output1 >= 0) {
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, output1);
  } else {
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, -output1);
  }

  // 计算右电机 PID 输出并控制方向
  if (output2 >= 0) {
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, output2);
  } else {
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, -output2);
  }
    
  t1 = target1;
  t2 = target2;
}

//----------------------------------主函数----------------------------------
void setup() {
  TCCR1B = TCCR1B & B11111000 | B00000001;
  MsTimer2::set(PERIOD, control);
  MsTimer2::start();
  
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);
  
  attachInterrupt(0, getEncoder1, CHANGE);
  attachInterrupt(1, getEncoder2, CHANGE);
  
  Serial.begin(9600);
  
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // digitalWrite(DIR1, HIGH);       
  // analogWrite(PWM1, 50);
  // digitalWrite(DIR2, LOW);       
  // analogWrite(PWM2, 50);
  
  // 初始化 8 个光电管管脚为输入模式
  pinMode(L0, INPUT);
  pinMode(L1, INPUT);
  pinMode(L2, INPUT);
  pinMode(L3, INPUT);
  pinMode(L4, INPUT);
  pinMode(L5, INPUT);
  pinMode(L6, INPUT);
  pinMode(L7, INPUT);

  // 初始化舵机引脚
  myservo1.attach(Servo_PIN1);
  // myservo2.attach(Servo_PIN2);
  // myservo3.attach(Servo_PIN3);
}

void loop() {
  // 启动序列：先复位舵机 → 再夹取 → 等待1秒 → 出发
  if (!started) {
    servo_Reset();
    // servo_Control();
    delay(1000);
    started = true;
    return;
  }

  // 全黑线停车后触发舵机复位
  if (servo_reset_pending) {
    servo_Reset();
    servo_reset_pending = false;
    delay(100);
  }

  // 1. 打印光电管数组状态 (例如: Sensors: 0 0 0 1 1 0 0 0 )
  // Serial.print("Sensors: ");
  // for (int i = 0; i < 8; i++) {
  //   Serial.print(sensorState[i]);
  //   Serial.print(" ");
  // }

  // // 2. 打印计算出的加权偏差值
  // Serial.print(" | Error: ");
  // if (current_error_out >= 0) Serial.print(" "); // 占位符让正负数对齐
  // Serial.print(current_error_out);

  // 3. 打印当前左右轮真实转速
  // Serial.print("\t| Vel_L: ");
//   Serial.print(velocity1);
//   Serial.print(",");
//   Serial.println(velocity2);
  
  // 【重要】加一个小延时，防止串口数据狂刷导致电脑或 Arduino 卡死
  delay(100); 
}

//----------------------------------编码器中断函数----------------------------------
void getEncoder1(void)
{
  if (digitalRead(ENCODER_A1) == LOW)
  {
    if (digitalRead(ENCODER_B1) == LOW)
      encoderVal1--;
    else
      encoderVal1++;
  }
  else
  {
    if (digitalRead(ENCODER_B1) == LOW)
      encoderVal1++;
    else
      encoderVal1--;
  }
}

void getEncoder2(void)
{
  if (digitalRead(ENCODER_A2) == LOW)
  {
    if (digitalRead(ENCODER_B2) == LOW)
      encoderVal2--;
    else
      encoderVal2++;
  }
  else
  {
    if (digitalRead(ENCODER_B2) == LOW)
      encoderVal2++;
    else
      encoderVal2--;
  }
}

//----------------------------------PID控制器----------------------------------
int pidController1(float targetVelocity, float currentVelocity)
{
  float ek10;
  ek10 = targetVelocity - currentVelocity;
  u1 = u1 + q0 * ek10 + q1 * ek11 + q2 * ek12;
  
  if (u1 > 255) u1 = 255;
  if (u1 < -255) u1 = -255;
  
  ek12 = ek11;
  ek11 = ek10;
  return (int)u1;
}

int pidController2(float targetVelocity, float currentVelocity)
{
  float ek20;
  ek20 = targetVelocity - currentVelocity;
  u2 = u2 + q0 * ek20 + q1 * ek21 + q2 * ek22;
  
  if (u2 > 255) u2 = 255;
  if (u2 < -255) u2 = -255;
  
  ek22 = ek21;
  ek21 = ek20;
  return (int)u2;
}

// // 控制舵机转动和夹取
// void servo_Control(void)
// {
//     // servo.write(angle)可以直接写入舵机转动角度，若是 180°舵机，则 angle取值在 0 - 180之间
//     // 装配舵机时，若舵机的当前角度不确定，180°舵机可能无法转动到期望的位置。为避免这个问题，可以先使舵机转动到特定角度，比如 0°或 90°再进行装配。
//     myservo3.write(100); // 张开夹爪
//     delay(1000);
//     myservo3.write(170); // 收紧夹爪
// }

// 使舵机回到初始位置
void servo_Reset(void)
{
    myservo1.write(120);
    delay(500); // 等待舵机转动到位
    // myservo2.write(120);
    // delay(500); // 等待舵机转动到位
    // myservo3.write(100);
}