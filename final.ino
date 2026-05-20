/**
  ******************************************************************************
  * @file           : final.ino
  * @brief          : 基于状态机的循迹小车控制程序（Arduino版）
  ******************************************************************************
  * mini_car_race
  * v2.0.0
  * 参考 STM32 版本 (main.c) 的状态判断与输出计算架构
  * 传感器：8路数字光电管（L4 L3 L2 L1 R1 R2 R3 R4），低电平有效
  * 控制周期：10ms（MsTimer2 定时中断）
  ******************************************************************************
  */

#include <MsTimer2.h>

/* ======================== 引脚定义 ======================== */
/* 编码器引脚 */
#define ENCODER_A1 2
#define ENCODER_B1 5
#define ENCODER_A2 3
#define ENCODER_B2 4

/* 电机 PWM 引脚 (Mega2560 Timer1 对应 11, 12) */
#define PWM1 11
#define PWM2 12

/* 电机方向引脚 */
#define DIR1 6
#define DIR2 7

/* 8 路循迹传感器引脚（低电平 = 检测到黑线，L4/R4 为最外侧） */
#define L4_PIN A7   // 左4（最外侧）
#define L3_PIN A6   // 左3
#define L2_PIN A5   // 左2
#define L1_PIN A4   // 左1（靠近中心）
#define R1_PIN A3   // 右1（靠近中心）
#define R2_PIN A2   // 右2
#define R3_PIN A1   // 右3
#define R4_PIN A0   // 右4（最外侧）

/* ======================== 控制参数 ======================== */
#define PERIOD 10               // 控制周期（ms）

/* ---------- 速度PID参数（左轮 / 右轮各自独立）---------- */
/* 左轮PID参数 */
#define L_Kp  7.0f              // 比例系数
#define L_Ki  0.14f             // 积分系数（= Kp * T/Ti = 7 * 10/500）
#define L_Kd  7.0f              // 微分系数（= Kp * Td/T = 7 * 10/10）
#define L_INTEGRAL_MAX  80.0f   // 积分限幅（防积分饱和）

/* 右轮PID参数 */
#define R_Kp  7.0f              // 比例系数
#define R_Ki  0.14f             // 积分系数（= Kp * T/Ti = 7 * 10/500）
#define R_Kd  7.0f              // 微分系数（= Kp * Td/T = 7 * 10/10）
#define R_INTEGRAL_MAX  80.0f   // 积分限幅（防积分饱和）

/* ---------- 赛道状态与输出参数 ---------- */
#define BASE_SPEED      6.0f    // [speed] 默认基础速度（m/s 量级）
#define MAX_SPEED        10.0f  // [speed] 最大速度限幅
#define STEERING_GAIN   0.8f    // 转向偏差→速度差映射增益
#define SHARP_OUTPUT    2.4f    // [sharp] 急弯态的固定差速输出
#define RIGHT_ANGLE_OUTPUT 3.0f // [right_angle] 直角弯的固定差速输出（更大）
#define SHARP_SPEED     0.7f    // [sharp] 急弯降速系数
#define RIGHT_ANGLE_SPEED 0.3f  // [right_angle] 直角弯降速系数（更慢）
#define OUTLINE_MAX      3.0f   // [outline] 出界时保持的最大差速
#define TIME_RECOVERING 8       // [recovering] 恢复态持续周期数（8×10ms=80ms）
#define MAX_OUTLINE_TIME 30     // [stop] 连续出界超过此次数则停车
#define SHARP_RATE       1.5f   // [rate] 重心变化率阈值，超过此值判定为急弯（传感器间距宽，靠速率补位）

/* 传感器位置索引（用于重心计算） */
/* 索引：L4=0, L3=1, L2=2, L1=3, R1=4, R2=5, R3=6, R4=7  中心=3.5 */
#define SENSOR_CENTER 3.5f

/* ==================== 数据结构 ==================== */

/**
 * @brief 标准PID控制器结构体（用于轮速控制环）
 * @note  每个电机拥有独立的PID实例，互不干扰
 */
struct PIDController {
  float targetVal;      // 目标速度
  float currentError;   // 当前误差
  float preError;       // 上一次误差
  float derivative;     // 微分项（误差变化率）
  float integral;       // 积分项（误差累积）
  float output;         // 控制器输出（PWM占空比）
  float Kp, Ki, Kd;     // 比例、积分、微分系数
  float integralMax;    // 积分限幅（每个电机独立）
};

/* 光电管传感器数据结构 */
struct SensorInfo {
  bool  MUX[8];         // 8路传感器布尔值（true=在线上）
  int8_t LEDCounter;    // 总亮灯数（检测到线的传感器数）
  int8_t LCounter;      // 左半侧亮灯数（索引0~3）
  int8_t RCounter;      // 右半侧亮灯数（索引4~7）
  int8_t Lmost;         // 最左侧亮灯索引（无则为8）
  int8_t Rmost;         // 最右侧亮灯索引（无则为-1）
  float centroid;       // 亮灯重心位置（加权平均，范围0~7）
};

/* ==================== 赛道状态机定义 ==================== */
typedef enum {
  STRAIGHT,             // 直道
  GENTLE_CURVE,         // 缓弯
  SHARP_TURN,           // 急弯
  RIGHT_ANGLE,          // 直角弯（≥4个传感器同时在线→线横穿车底）
  EDGE,                 // 边缘（仅少量传感器在线上）
  RECOVERING,           // 恢复态（急弯/出界后的消抖过渡）
  OUTLINE               // 出界（所有传感器均不在线上）
} STATE;

/* ==================== 全局变量 ==================== */

/* 左右轮PID控制器（各自独立的 Kp/Ki/Kd/积分限幅，初始化在 setup() 中完成） */
struct PIDController pidL;
struct PIDController pidR;

/* 编码器脉冲累计（由中断服务函数更新，控制函数读取后清零） */
volatile long encoderVal1 = 0;
volatile long encoderVal2 = 0;

/* 速度测量值（由编码器脉冲换算） */
float velocity1 = 0;
float velocity2 = 0;

/* 上一次控制周期的目标速度（用于平滑过渡） */
float prevTarget1 = 0;
float prevTarget2 = 0;

/* 赛道状态相关 */
STATE current_state = STRAIGHT;       // 当前状态
STATE last_state    = STRAIGHT;       // 上一周期状态
float speed_factor  = 1.0f;           // 速度系数（急弯时降速）
float last_reliable_error = 0;        // 上一次有效偏差（出界时保持）
int8_t sharp_last_side = 0;           // 急弯方向记忆（-1=左弯, +1=右弯, 0=未知）
uint8_t recCounter = 0;               // 恢复态计数器
uint8_t outlineCounter = 0;           // 出界累计计数器
bool STOPFlag = false;                // 紧急停车标志

/* 上一周期重心值（用于计算重心变化率，辅助弯道急缓判断） */
float prev_centroid = SENSOR_CENTER;

/* 平滑后的目标速度 */
float t1 = 0, t2 = 0;

/* ==================== 函数声明 ==================== */
void readSensors(struct SensorInfo *s);
void judgeState(struct SensorInfo *s);
float computeOutput(struct SensorInfo *s);
void computePID(struct PIDController *pid, float measuredVal);
int  applyMotorOutput(struct PIDController *pid, int dirPin, int pwmPin, int *lastPwm);

/* ==================== 传感器读取 ==================== */
/**
 * @brief  读取8路光电管并更新传感器数据结构
 * @param  s 传感器数据结构体指针
 * @note   传感器低电平有效（LOW = 检测到黑线）
 *         计算亮灯分布、重心、左右统计，供状态判断使用
 */
void readSensors(struct SensorInfo *s) {
  /* 读取8路传感器（LOW = 在线上，从左到右：L4 L3 L2 L1 R1 R2 R3 R4） */
  s->MUX[0] = (digitalRead(L4_PIN) == LOW);
  s->MUX[1] = (digitalRead(L3_PIN) == LOW);
  s->MUX[2] = (digitalRead(L2_PIN) == LOW);
  s->MUX[3] = (digitalRead(L1_PIN) == LOW);
  s->MUX[4] = (digitalRead(R1_PIN) == LOW);
  s->MUX[5] = (digitalRead(R2_PIN) == LOW);
  s->MUX[6] = (digitalRead(R3_PIN) == LOW);
  s->MUX[7] = (digitalRead(R4_PIN) == LOW);

  /* 统计初始化 */
  s->centroid    = 0;
  s->LEDCounter  = 0;
  s->LCounter    = 0;
  s->RCounter    = 0;
  s->Lmost       = 8;
  s->Rmost       = -1;

  /* 遍历8路传感器，计算统计数据 */
  for (int i = 0; i < 8; i++) {
    if (s->MUX[i]) {
      /* 左右侧计数：索引0~3为左侧，4~7为右侧 */
      if (i < 4)       s->LCounter++;
      else             s->RCounter++;

      /* 最左/最右亮灯索引 */
      if (i < s->Lmost) s->Lmost = i;
      if (i > s->Rmost) s->Rmost = i;

      /* 重心累加 */
      s->centroid += i;
      s->LEDCounter++;
    }
  }

  /* 计算重心平均值（有效亮灯数>0时） */
  if (s->LEDCounter > 0) {
    s->centroid /= s->LEDCounter;
  }
}

/* ==================== 状态判断模块 ==================== */
/**
 * @brief  根据传感器数据判断当前赛道状态
 * @param  s 传感器数据结构体指针
 * @note   状态优先级：出界 > 终点(8灯) > 直角弯(≥4灯) > 急弯 > 缓弯 > 直道
 *         急弯判断：传感器位置(外侧灯亮) + 重心变化率(>SHARP_RATE) 双重判定
 *         直道判断：仅中间两个灯(L1/R1)亮即可，输出由重心自动偏左/偏右
 *         状态切换时触发 RECOVERING 消抖过渡
 */
void judgeState(struct SensorInfo *s) {

  /* ---- 1. 出界判断（所有传感器均不在线上）---- */
  if (s->LEDCounter == 0) {
    outlineCounter++;
    if (outlineCounter > MAX_OUTLINE_TIME) {
      STOPFlag = true;    // 长时间出界→紧急停车
    }
    /* 保持上一状态不变（由 computeOutput 使用 last_reliable_error） */
    return;
  } else {
    outlineCounter = 0;   // 检测到线，清零出界计数器
  }

  /* ---- 2. 赛道状态判定（有传感器在线）---- */
  if (s->LEDCounter == 8) {
    /* 全部传感器均在线 → 到达终点，停车 */
    STOPFlag = true;
    return;
  }
  else if (s->LEDCounter >= 4) {
    /* 直角弯：≥4个传感器同时在线 → 黑线横穿车底，即将急转 */
    current_state = RIGHT_ANGLE;
    if (last_state != RIGHT_ANGLE) sharp_last_side = 0;
    /* 累计方向记忆（哪侧传感器更多就往哪边拐） */
    sharp_last_side += (s->LCounter > s->RCounter) ? -1 : 1;
  }
  else {
    /* ---- 基于重心变化率 + 传感器位置的双重判断 ---- */
    float d_centroid = s->centroid - prev_centroid;  // 本周期重心偏移速率

    if (s->Lmost >= 3 && s->Rmost <= 4) {
      /* 直道：只有中间两个灯(L1=3 或 R1=4)亮
       * 中间偏左(L1)亮 → centroid=3 → steerting偏左
       * 中间偏右(R1)亮 → centroid=4 → steering偏右 */
      current_state = STRAIGHT;
      sharp_last_side = 0;
    }
    else if ((s->Lmost == 0 && s->Rmost < 7) ||
             (s->Rmost == 7 && s->Lmost > 0) ||
             fabs(d_centroid) > SHARP_RATE) {
      /* 急弯：最外侧传感器(L4/R4)亮起，或重心变化速率超过阈值
       * 单传感器也可触发（传感器间距宽）
       * 速率超阈值说明黑线正在快速偏移，即使未到最外侧也预判为急弯 */
      current_state = SHARP_TURN;
      if (last_state != SHARP_TURN) sharp_last_side = 0;
      /* 累计方向记忆（消抖：防止出界前瞬间的不可靠读数） */
      sharp_last_side += (s->LCounter > s->RCounter) ? -1 : 1;
    }
    else if (s->LEDCounter >= 1) {
      /* 缓弯：≥1个传感器在线，重心变化率不高 */
      current_state = GENTLE_CURVE;
      sharp_last_side = 0;
    }
    else {
      current_state = STRAIGHT;
      sharp_last_side = 0;
    }
  }

  /* 更新上一周期重心值 */
  prev_centroid = s->centroid;

  /* ---- 3. 状态恢复检测：从急弯/直角弯/出界转出→进入 RECOVERING 消抖 ---- */
  if ((last_state == SHARP_TURN  && current_state != SHARP_TURN)  ||
      (last_state == RIGHT_ANGLE && current_state != RIGHT_ANGLE) ||
      (last_state == OUTLINE     && current_state != OUTLINE)) {
    current_state = RECOVERING;
    recCounter = 0;
  }

  /* RECOVERING 状态计时维持 */
  if (last_state == RECOVERING) {
    recCounter++;
    if (current_state != SHARP_TURN && current_state != RIGHT_ANGLE && current_state != OUTLINE) {
      current_state = RECOVERING;   // 维持恢复态（除非再次进入急弯/直角弯/出界）
    }
    if (recCounter >= TIME_RECOVERING) {
      current_state = STRAIGHT;     // 计时满→回到直道
      sharp_last_side = 0;
    }
  }

  /* 非急弯/直角弯/出界/恢复态时，清零方向记忆 */
  if (current_state != SHARP_TURN  && current_state != RIGHT_ANGLE &&
      current_state != OUTLINE     && current_state != RECOVERING) {
    sharp_last_side = 0;
  }

  last_state = current_state;
}

/* ==================== 输出计算模块 ==================== */
/**
 * @brief  根据赛道状态计算转向差速输出
 * @param  s 传感器数据结构体指针
 * @return 转向差速值（正值=右转，负值=左转），叠加到基础速度上
 * @note   不同状态使用不同的偏差增益和速度系数
 *         参考 main.c 中的 computeMUXVal() 输出计算逻辑
 */
float computeOutput(struct SensorInfo *s) {
  float result = 0;
  /* 基础偏差：重心偏离中心的程度（范围约 -2.8 ~ +2.8） */
  float base_error = (s->centroid - SENSOR_CENTER) * STEERING_GAIN;

  switch (current_state) {

    case STRAIGHT:
      /* 直道：正常比例响应 */
      result       = base_error * 1.3f;
      speed_factor = 1.0f;
      break;

    case GENTLE_CURVE:
      /* 缓弯：适度增强响应 */
      result       = base_error * 1.6f;
      speed_factor = 1.0f;
      break;

    case SHARP_TURN:
      /* 急弯：固定满舵输出，方向由 sharp_last_side 决定 */
      result       = (sharp_last_side < 0) ? -SHARP_OUTPUT : SHARP_OUTPUT;
      result       = (sharp_last_side == 0) ? 0 : result;
      speed_factor = SHARP_SPEED;
      break;

    case RIGHT_ANGLE:
      /* 直角弯：最大差速输出（接近原地旋转），方向由 sharp_last_side 决定 */
      result       = (sharp_last_side < 0) ? -RIGHT_ANGLE_OUTPUT : RIGHT_ANGLE_OUTPUT;
      result       = (sharp_last_side == 0) ? 0 : result;
      speed_factor = RIGHT_ANGLE_SPEED;
      break;

    case OUTLINE:
      /* 出界：保持上一次有效偏差，并限幅 */
      result       = last_reliable_error;
      if (result >  OUTLINE_MAX) result =  OUTLINE_MAX;
      if (result < -OUTLINE_MAX) result = -OUTLINE_MAX;
      speed_factor = 1.0f;
      break;

    case EDGE:
      /* 边缘：强化响应，尽快拉回线中心 */
      result       = base_error * 2.0f;
      speed_factor = 1.0f;
      break;

    case RECOVERING:
      /* 恢复态：抑制过冲，缓慢回调 */
      result       = base_error * 0.6f;
      speed_factor = 1.0f;
      break;

    default:
      result       = base_error;
      speed_factor = 1.0f;
      break;
  }

  /* 保存有效误差（供出界时保持） */
  if (s->LEDCounter > 0) {
    last_reliable_error = result;
  }

  return result;
}

/* ==================== PID 控制器计算 ==================== */
/**
 * @brief  位置式PID控制器（用于轮速环）
 * @param  pid        PID控制器结构体指针
 * @param  measuredVal 当前测量速度
 * @note   每个电机调用各自独立的PID实例
 *         包含积分限幅防饱和、输出硬限幅
 */
void computePID(struct PIDController *pid, float measuredVal) {
  /* 误差更新 */
  pid->preError     = pid->currentError;
  pid->currentError = pid->targetVal - measuredVal;

  /* 微分更新（误差变化率） */
  pid->derivative   = pid->currentError - pid->preError;

  /* 积分更新（误差累积） */
  pid->integral    += pid->currentError;

  /* 积分限幅（防止积分饱和，每个电机独立限幅值） */
  if (pid->integral >  pid->integralMax) pid->integral =  pid->integralMax;
  if (pid->integral < -pid->integralMax) pid->integral = -pid->integralMax;

  /* PID输出 = 比例项 + 积分项 + 微分项 */
  pid->output = pid->Kp * pid->currentError
              + pid->Ki * pid->integral
              + pid->Kd * pid->derivative;

  /* 输出限幅（适配 analogWrite 的 0~255 范围） */
  if (pid->output >  255.0f) pid->output =  255.0f;
  if (pid->output < -255.0f) pid->output = -255.0f;
}

/* ==================== 电机输出应用 ==================== */
/**
 * @brief  将PID输出值应用到指定电机
 * @param  pid      PID控制器结构体指针
 * @param  dirPin   方向控制引脚
 * @param  pwmPin   PWM输出引脚
 * @param  lastPwm  上一次PWM值指针（用于变化率限幅）
 * @return 实际写入的PWM值
 * @note   正值=正转，负值=反转
 *         变化率限幅防止电流冲击（每次最多变化50）
 */
int applyMotorOutput(struct PIDController *pid, int dirPin, int pwmPin, int *lastPwm) {
  int pwm = (int)pid->output;

  /* 紧急停车：强制输出0 */
  if (STOPFlag) pwm = 0;

  /* 变化率限幅（每次最多变化50，防止电流冲击） */
  if (pwm - *lastPwm >  50) pwm = *lastPwm + 50;
  if (pwm - *lastPwm < -50) pwm = *lastPwm - 50;
  *lastPwm = pwm;

  /* 方向与PWM输出 */
  if (pwm >= 0) {
    digitalWrite(dirPin, HIGH);   // 正转方向
    analogWrite(pwmPin, pwm);
  } else {
    digitalWrite(dirPin, LOW);    // 反转方向
    analogWrite(pwmPin, -pwm);
  }

  return pwm;
}

/* ==================== 定时控制函数（每 PERIOD ms 执行一次） ==================== */
/**
 * @brief  MsTimer2 定时中断回调（10ms周期）
 * @note   主控制回环：
 *          1. 读取8路传感器 → 填充 SensorInfo
 *          2. 状态判断 → judgeState()
 *          3. 输出计算 → computeOutput() 得到转向差速
 *          4. 差速模型分配左右轮目标速度
 *          5. 编码器→速度换算
 *          6. 左右轮PID计算 → computePID()
 *          7. PWM+方向输出 → applyMotorOutput()
 */
void control(void) {
  struct SensorInfo sensor;
  float steering;           // 转向差速值
  float targetL, targetR;   // 左右轮目标速度

  /* ---- 1. 读取传感器 ---- */
  readSensors(&sensor);

  /* ---- 2. 状态判断 ---- */
  judgeState(&sensor);

  /* ---- 3. 输出计算（转向差速） ---- */
  steering = computeOutput(&sensor);

  /* ---- 4. 差速模型：基础速度 ± 转向调整 ---- */
  targetL = BASE_SPEED * speed_factor - steering;
  targetR = BASE_SPEED * speed_factor + steering;

  /* 目标速度限幅 */
  if (targetL >  MAX_SPEED) targetL =  MAX_SPEED;
  if (targetL < -MAX_SPEED) targetL = -MAX_SPEED;
  if (targetR >  MAX_SPEED) targetR =  MAX_SPEED;
  if (targetR < -MAX_SPEED) targetR = -MAX_SPEED;

  /* 速度平滑过渡（低通滤波，减少阶跃冲击） */
  targetL = targetL * 0.6f + prevTarget1 * 0.4f;
  targetR = targetR * 0.6f + prevTarget2 * 0.4f;
  prevTarget1 = targetL;
  prevTarget2 = targetR;

  /* 写入PID目标值 */
  pidL.targetVal = targetL;
  pidR.targetVal = targetR;

  /* ---- 5. 编码器→速度换算 ---- */
  /* velocity = ( pulses / 780 pulses_per_rev ) * 2*pi*r * (1000ms / PERIODms )
   *          = pulses * 6.283 / 780 * 100  （当 PERIOD=10 时）
   * 780 = 编码器线数，2*pi*r = 轮周长（约 6.283 单位长度） */
  velocity1 = (encoderVal1 / 780.0) * 3.1415 * 2.0 * (1000.0 / PERIOD);
  velocity2 = (encoderVal2 / 780.0) * 3.1415 * 2.0 * (1000.0 / PERIOD);

  /* 清零编码器脉冲累计（为下一周期做准备） */
  encoderVal1 = 0;
  encoderVal2 = 0;

  /* ---- 6. 左右轮独立PID计算 ---- */
  computePID(&pidL, velocity1);   // 左轮PID
  computePID(&pidR, velocity2);   // 右轮PID（注意：右轮取反，因电机对向安装）

  /* ---- 7. 电机输出 ---- */
  static int lastPwm1 = 0, lastPwm2 = 0;
  applyMotorOutput(&pidL, DIR1, PWM1, &lastPwm1);   // 左电机
  /* 右轮电机：因对向安装，需取反后输出 */
  pidR.output = -pidR.output;
  applyMotorOutput(&pidR, DIR2, PWM2, &lastPwm2);   // 右电机
  pidR.output = -pidR.output;   // 恢复原始值

  t1 = targetL;
  t2 = targetR;
}

/* ==================== 初始化函数 ==================== */
void setup() {
  /* 设置 Timer1 的 PWM 频率（提高至约 31kHz，减少电机啸叫）
   * TCCR1B = TCCR1B & B11111000 | B00000001 → 不分频，约 31kHz PWM */
  TCCR1B = TCCR1B & B11111000 | B00000001;

  /* 注册 MsTimer2 定时器回调（稍后在 setup 末尾启动） */
  MsTimer2::set(PERIOD, control);

  /* 编码器引脚配置（上拉输入，增强抗干扰能力） */
  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);

  /* 编码器中断绑定（CHANGE 模式=2倍频） */
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), getEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), getEncoder2, CHANGE);

  /* 串口初始化（调试输出） */
  Serial.begin(9600);

  /* 左轮PID参数初始化 */
  pidL.Kp = L_Kp;  pidL.Ki = L_Ki;  pidL.Kd = L_Kd;
  pidL.integralMax = L_INTEGRAL_MAX;
  pidL.targetVal = 0;  pidL.currentError = 0;  pidL.preError = 0;
  pidL.derivative = 0;  pidL.integral = 0;  pidL.output = 0;

  /* 右轮PID参数初始化 */
  pidR.Kp = R_Kp;  pidR.Ki = R_Ki;  pidR.Kd = R_Kd;
  pidR.integralMax = R_INTEGRAL_MAX;
  pidR.targetVal = 0;  pidR.currentError = 0;  pidR.preError = 0;
  pidR.derivative = 0;  pidR.integral = 0;  pidR.output = 0;

  /* 电机PWM引脚 */
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);

  /* 电机方向引脚 */
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  digitalWrite(DIR1, HIGH);   // 左电机初始方向：正转
  digitalWrite(DIR2, LOW);    // 右电机初始方向（对向安装取反）

  /* 8路循迹传感器引脚 */
  pinMode(L4_PIN, INPUT); pinMode(L3_PIN, INPUT);
  pinMode(L2_PIN, INPUT); pinMode(L1_PIN, INPUT);
  pinMode(R1_PIN, INPUT); pinMode(R2_PIN, INPUT);
  pinMode(R3_PIN, INPUT); pinMode(R4_PIN, INPUT);

  /* 所有初始化完成，启动定时中断 */
  MsTimer2::start();
}

/* ==================== 主循环 ==================== */
void loop() {
  /* 定期通过串口输出调试信息：
   * 格式：当前状态  左轮速度  右轮速度 */
  Serial.print("State: ");
  Serial.print(current_state);
  Serial.print("\tV_L: ");
  Serial.print(velocity1);
  Serial.print("\tV_R: ");
  Serial.println(velocity2);
  delay(100);   // 100ms 输出一次，避免刷屏
}

/* ==================== 编码器中断服务函数 ==================== */

/**
 * @brief  编码器1（左轮）中断服务函数
 * @note   CHANGE 模式4倍频：A/B相边沿均触发
 *         根据A/B相电平关系判断旋转方向
 */
void getEncoder1(void) {
  if (digitalRead(ENCODER_A1) == LOW) {
    if (digitalRead(ENCODER_B1) == LOW) {
      encoderVal1--;
    } else {
      encoderVal1++;
    }
  } else {
    if (digitalRead(ENCODER_B1) == LOW) {
      encoderVal1++;
    } else {
      encoderVal1--;
    }
  }
}

/**
 * @brief  编码器2（右轮）中断服务函数
 * @note   与编码器1逻辑相同，独立计数，互不影响
 */
void getEncoder2(void) {
  if (digitalRead(ENCODER_A2) == LOW) {
    if (digitalRead(ENCODER_B2) == LOW) {
      encoderVal2--;
    } else {
      encoderVal2++;
    }
  } else {
    if (digitalRead(ENCODER_B2) == LOW) {
      encoderVal2++;
    } else {
      encoderVal2--;
    }
  }
}
