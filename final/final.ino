/**
 * Arduino 差速轮循迹小车
 * 高级状态机实现 + 噪声滤波
 * 控制周期: 10ms
 */

#include <MsTimer2.h>

// ==================== 引脚配置 ====================
// 编码器引脚（用于测量车轮转速）
#define ENCODER_A1 2    // 左轮编码器A相
#define ENCODER_B1 5    // 左轮编码器B相
#define ENCODER_A2 3    // 右轮编码器A相
#define ENCODER_B2 4    // 右轮编码器B相

// 电机控制引脚
#define PWM1 11         // 左电机PWM控制（速度）
#define PWM2 12         // 右电机PWM控制（速度）
#define DIR1 6          // 左电机方向控制
#define DIR2 7          // 右电机方向控制

// 8个光电传感器引脚（从左到右排列）
#define L4_PIN A7       // 最左侧传感器
#define L3_PIN A6       // 左3传感器
#define L2_PIN A5       // 左2传感器
#define L1_PIN A4       // 左1传感器
#define R1_PIN A3       // 右1传感器
#define R2_PIN A2       // 右2传感器
#define R3_PIN A1       // 右3传感器
#define R4_PIN A0       // 最右侧传感器

// ==================== 控制参数配置 ====================
#define CTRL_PERIOD 10  // 控制周期10ms (100Hz控制频率)

// 速度环PID增益参数（用于控制电机转速）
#define P_GAIN 15.0     // 比例增益：响应误差大小
#define I_TIME 70.0     // 积分时间：消除稳态误差
#define D_TIME 15.0     // 微分时间：减少超调，提高稳定性

// 转向PD控制器增益（用于左右轮差速转向）
#define STEER_P 0.30    // 转向比例增益：控制转向灵敏度
#define STEER_D 0.20    // 转向微分增益：抑制转向震荡

// 速度限制参数
#define BASE_VEL 3.0    // 基准速度（rad/s）
#define VEL_LIMIT 5.0   // 最大速度限制（rad/s）

// ==================== 数据结构定义 ====================

// 增量式PID控制器结构体（用于速度闭环控制）
struct VelocityController {
  float setpoint;      // 目标速度设定值
  float err[3];        // 误差数组: err[0]=当前误差, err[1]=上次误差, err[2]=上上次误差
  float control;       // PID输出的控制量（PWM值）
  float coeff[3];      // PID系数: [0]=P项系数, [1]=I项系数, [2]=D项系数
};

// 轨迹状态枚举（状态机的各种状态）
enum TrackState {
  STATE_STRAIGHT,      // 直线状态：传感器居中，小幅偏差
  STATE_GENTLE,        // 缓弯状态：中等偏差，多传感器检测到
  STATE_SHARP,         // 急弯状态：大幅偏差，边缘传感器触发
  STATE_EDGE,          // 边缘状态：只有少数传感器检测到线
  STATE_RECOVER,       // 恢复状态：从急弯或丢线后重新找到线
  STATE_OUT_DEFAULT,   // 默认丢线状态：普通丢线，保持原方向搜索
  STATE_OUT_SHARP      // 急弯丢线状态：从急弯丢线，需要继续转向
};

// 传感器数据聚合结构体
struct SensorData {
  int reading[8];      // 8个传感器的原始读数（0=检测到黑线，1=白色地面）
  int active_count;    // 检测到黑线的传感器总数
  int left_count;      // 左侧（0-3）检测到线的传感器数
  int right_count;     // 右侧（4-7）检测到线的传感器数
  int leftmost_idx;    // 最左侧检测到线的传感器索引
  int rightmost_idx;   // 最右侧检测到线的传感器索引
  float weighted_pos;  // 加权位置（0-7，3.5为中心）
};

// ==================== 全局状态变量 ====================

VelocityController leftCtrl, rightCtrl;  // 左右轮速度控制器

volatile long leftPulse = 0;   // 左轮编码器脉冲计数（中断中更新）
volatile long rightPulse = 0;  // 右轮编码器脉冲计数（中断中更新）

float leftSpeed = 0;   // 左轮实际速度（经过滤波）
float rightSpeed = 0;  // 右轮实际速度（经过滤波）

float prevLeftSetpoint = 0;   // 上一次左轮目标速度（用于低通滤波）
float prevRightSetpoint = 0;  // 上一次右轮目标速度（用于低通滤波）

TrackState currentMode = STATE_STRAIGHT;   // 当前轨迹状态
TrackState previousMode = STATE_STRAIGHT;  // 上一次轨迹状态

float lastValidError = 0;   // 最后一次有效的偏差值（用于丢线时的方向判断）
float prevSteerError = 0;   // 上一次转向误差（用于微分计算）
int sharpDirection = 0;     // 急弯方向累积值（正=左转，负=右转）
int lostCounter = 0;        // 丢线计数器（连续丢线帧数）
int recoverTimer = 0;       // 恢复状态计时器

bool emergencyStop = false; // 紧急停止标志（检测到终点线时触发）

// 传感器状态缓冲区（用于主循环调试输出）
int sensorBuffer[8] = {0};
float errorBuffer = 0;

// 历史误差跟踪（用于丢线后恢复方向判断）
float errorHistory[20] = {0};  // 存储最近20次的偏差值
int historyIndex = 0;           // 历史数组的当前索引
int historySize = 0;            // 历史数组的有效数据量

// 上一次有效的线段位置（用于噪声滤波）
int prevSegment[8] = {0};

// ==================== 函数声明 ====================
void updateControl();  // 主控制更新函数（10ms周期调用）
void acquireSensorData(SensorData* data);  // 采集传感器数据
void filterNoiseSegments(SensorData* data);  // 滤除噪声线段
void determineTrackState(SensorData* data);  // 判断轨迹状态
float calculateSteeringOutput(SensorData* data);  // 计算转向输出
void updateVelocityPID(VelocityController* ctrl, float measured);  // 更新速度PID
void driveMotor(VelocityController* ctrl, int dirPin, int pwmPin);  // 驱动电机
void leftEncoderISR();   // 左轮编码器中断服务函数
void rightEncoderISR();  // 右轮编码器中断服务函数

// ==================== 传感器采集与滤波 ====================

/**
 * 采集8个光电传感器的数据
 * @param data 传感器数据结构指针
 * 说明：传感器检测到黑线时输出LOW，白色地面时输出HIGH
 *       这里将LOW转换为1，HIGH转换为0，方便后续处理
 */
void acquireSensorData(SensorData* data) {
  // 读取原始传感器值（LOW = 检测到黑线）
  data->reading[0] = (digitalRead(L4_PIN) == LOW) ? 1 : 0;  // 最左
  data->reading[1] = (digitalRead(L3_PIN) == LOW) ? 1 : 0;
  data->reading[2] = (digitalRead(L2_PIN) == LOW) ? 1 : 0;
  data->reading[3] = (digitalRead(L1_PIN) == LOW) ? 1 : 0;
  data->reading[4] = (digitalRead(R1_PIN) == LOW) ? 1 : 0;
  data->reading[5] = (digitalRead(R2_PIN) == LOW) ? 1 : 0;
  data->reading[6] = (digitalRead(R3_PIN) == LOW) ? 1 : 0;
  data->reading[7] = (digitalRead(R4_PIN) == LOW) ? 1 : 0;  // 最右

  // 复制到缓冲区用于调试输出
  for (int i = 0; i < 8; i++) {
    sensorBuffer[i] = data->reading[i];
  }
}

/**
 * 噪声线段滤波函数
 * @param data 传感器数据结构指针
 * 功能：当检测到多个不连续的线段时，只保留与上一次位置重叠最多的线段
 * 目的：防止相邻赛道或地面污渍被误识别为黑线
 */
void filterNoiseSegments(SensorData* data) {
  // 反转数据用于处理（1=检测到线，0=未检测到）
  int activeLine[8];
  for (int i = 0; i < 8; i++) {
    activeLine[i] = (data->reading[i] == 0) ? 1 : 0;
  }

  // 线段检测：找出所有连续的线段
  int segments[4][2];  // 最多4个线段，每个记录[起始位置, 结束位置]
  int segmentCount = 0;
  int inSegment = 0;

  for (int i = 0; i <= 8; i++) {
    if (i < 8 && activeLine[i] == 1) {
      if (!inSegment) {
        segments[segmentCount][0] = i;  // 线段起始
        inSegment = 1;
      }
    } else {
      if (inSegment) {
        segments[segmentCount][1] = i - 1;  // 线段结束
        segmentCount++;
        inSegment = 0;
      }
    }
  }

  // 多线段噪声滤波：只保留与历史位置重叠最多的线段
  if (segmentCount > 1) {
    int maxOverlap = -1;  // 最大重叠数
    int bestIdx = 0;      // 最佳线段索引

    // 遍历每个线段，计算与上次位置的重叠度
    for (int s = 0; s < segmentCount; s++) {
      int overlap = 0;
      for (int i = segments[s][0]; i <= segments[s][1]; i++) {
        if (prevSegment[i] == 1) overlap++;  // 计算重叠传感器数
      }

      int segWidth = segments[s][1] - segments[s][0];
      int bestWidth = segments[bestIdx][1] - segments[bestIdx][0];

      // 选择重叠最多的线段；若重叠相同，选择更宽的线段
      if (overlap > maxOverlap || (overlap == maxOverlap && segWidth > bestWidth)) {
        maxOverlap = overlap;
        bestIdx = s;
      }
    }

    // 丢弃非最佳线段
    for (int i = 0; i < 8; i++) {
      if (i >= segments[bestIdx][0] && i <= segments[bestIdx][1]) {
        // 保留最佳线段
      } else {
        activeLine[i] = 0;
        data->reading[i] = 1;  // 标记为未检测到线
      }
    }
  }

  // 更新历史线段位置记忆
  for (int i = 0; i < 8; i++) {
    prevSegment[i] = activeLine[i];
  }
}

/**
 * 计算传感器统计数据
 * @param data 传感器数据结构指针
 * 功能：计算激活传感器数量、左右分布、边界位置、加权中心位置
 */
void computeSensorStatistics(SensorData* data) {
  data->active_count = 0;    // 检测到线的传感器总数
  data->left_count = 0;      // 左侧检测到线的数量
  data->right_count = 0;     // 右侧检测到线的数量
  data->leftmost_idx = 8;    // 最左边界（初始化为无效值）
  data->rightmost_idx = -1;  // 最右边界（初始化为无效值）
  data->weighted_pos = 0;    // 加权位置

  // 遍历所有传感器，统计信息
  for (int i = 0; i < 8; i++) {
    if (data->reading[i] == 0) {  // 检测到黑线
      if (i < 4) data->left_count++;   // 左半部分
      else data->right_count++;        // 右半部分

      if (i < data->leftmost_idx) data->leftmost_idx = i;
      if (i > data->rightmost_idx) data->rightmost_idx = i;

      data->weighted_pos += i;  // 累加位置用于加权平均
      data->active_count++;
    }
  }

  // 计算加权平均位置（黑线的中心位置）
  static float lastValidPos = 3.5;  // 默认中心位置
  if (data->active_count > 0) {
    data->weighted_pos /= data->active_count;  // 加权平均
    lastValidPos = data->weighted_pos;
  } else {
    // 没有检测到线时，使用上次有效位置
    data->weighted_pos = lastValidPos;
  }
}

// ==================== 状态机逻辑 ====================

/**
 * 判断当前轨迹状态
 * @param data 传感器数据结构指针
 * 功能：根据传感器数据判断小车当前处于什么状态（直线、缓弯、急弯、丢线等）
 *       这是整个控制系统的核心决策部分
 */
void determineTrackState(SensorData* data) {
  // ===== 丢线检测 =====
  if (data->active_count == 0) {
    lostCounter++;  // 丢线计数器递增
    if (lostCounter >= 8) {  // 连续8个周期（80ms）都丢线
      // 根据之前的状态决定丢线后的策略
      if (currentMode != STATE_OUT_SHARP && currentMode != STATE_OUT_DEFAULT) {
        if (previousMode == STATE_SHARP || previousMode == STATE_RECOVER) {
          // 从急弯丢线：继续按急弯方向转
          currentMode = STATE_OUT_SHARP;
        } else {
          // 普通丢线：按历史偏差方向搜索
          currentMode = STATE_OUT_DEFAULT;
        }
      }
    } else {
      // 丢线时间不足，保持之前状态
      currentMode = previousMode;
    }
    return;
  } else {
    lostCounter = 0;  // 检测到线，清零丢线计数
  }

  // ===== 恢复侧滤波（防止误检测相邻赛道）=====
  // 如果正在丢线状态，突然检测到相反方向的线，可能是相邻赛道
  if (data->active_count > 0 && (currentMode == STATE_OUT_SHARP || currentMode == STATE_OUT_DEFAULT)) {
    if (lastValidError > 1.0 && data->weighted_pos > 3.5) {
      // 上次向左偏，但现在检测到右侧线 -> 可能是相邻赛道，忽略
      data->active_count = 0;
      for (int i = 0; i < 8; i++) prevSegment[i] = 0;
      return;
    } else if (lastValidError < -1.0 && data->weighted_pos < 3.5) {
      // 上次向右偏，但现在检测到左侧线 -> 可能是相邻赛道，忽略
      data->active_count = 0;
      for (int i = 0; i < 8; i++) prevSegment[i] = 0;
      return;
    }
  }

  // ===== 终点线检测 =====
  // 所有传感器都检测到线 = 宽黑色终点线
  if (data->active_count == 8) {
    emergencyStop = true;  // 触发紧急停止
    leftCtrl.setpoint = 0;
    rightCtrl.setpoint = 0;
    prevLeftSetpoint = 0;
    prevRightSetpoint = 0;
    currentMode = STATE_STRAIGHT;
    sharpDirection = 0;
    return;
  }

  // ===== 状态判断逻辑 =====
  // 直线状态：左右传感器数量接近，总数少，不在边缘
  if (abs(data->left_count - data->right_count) <= 2 &&
      data->active_count <= 3 &&
      data->leftmost_idx != 0 &&
      data->rightmost_idx != 7) {
    currentMode = STATE_STRAIGHT;
  }
  // 急弯状态：大量传感器在一侧，且触及边缘传感器
  else if (((data->left_count >= 3 && data->leftmost_idx == 0 && data->rightmost_idx != 7) ||
            (data->right_count >= 3 && data->rightmost_idx == 7 && data->leftmost_idx != 0)) &&
           data->active_count >= 4) {
    currentMode = STATE_SHARP;
    // 累积急弯方向（用于判断左转还是右转）
    if (previousMode != STATE_SHARP) sharpDirection = 0;
    sharpDirection += data->left_count > data->right_count ? 1 : -1;
  }
  // 缓弯状态：较多传感器检测到线
  else if (data->active_count >= 3) {
    currentMode = STATE_GENTLE;
  }
  // 边缘状态：少量传感器检测到线
  else if (data->active_count >= 1) {
    currentMode = STATE_EDGE;
  }
  else {
    currentMode = STATE_STRAIGHT;
  }

  // ===== 恢复状态转换 =====
  // 从急弯或丢线状态恢复到正常状态时，需要平滑过渡
  if ((previousMode == STATE_SHARP && currentMode != STATE_SHARP &&
       currentMode != STATE_OUT_SHARP && currentMode != STATE_OUT_DEFAULT) ||
      (previousMode == STATE_OUT_SHARP && currentMode != STATE_OUT_SHARP &&
       currentMode != STATE_OUT_DEFAULT)) {
    currentMode = STATE_RECOVER;
    recoverTimer = 0;
  }

  // ===== 恢复状态计时 =====
  if (previousMode == STATE_RECOVER) {
    recoverTimer++;
    if (currentMode != STATE_SHARP && currentMode != STATE_OUT_DEFAULT &&
        currentMode != STATE_OUT_SHARP) {
      currentMode = STATE_RECOVER;
    }
    if (recoverTimer >= 8) {  // 恢复8个周期（80ms）后回到正常
      currentMode = STATE_STRAIGHT;
      sharpDirection = 0;
    }
  }

  previousMode = currentMode;  // 更新历史状态

  // 清除急弯方向累积（非急弯相关状态）
  if (currentMode != STATE_SHARP && currentMode != STATE_OUT_SHARP &&
      currentMode != STATE_RECOVER) {
    sharpDirection = 0;
  }
}

// ==================== 转向输出计算 ====================

/**
 * 计算转向输出并设置左右轮目标速度
 * @param data 传感器数据结构指针
 * @return 转向误差值
 * 功能：根据当前状态和传感器位置，计算左右轮的目标速度
 *       核心思想：差速转向（左右轮速度差产生转向）
 */
float calculateSteeringOutput(SensorData* data) {
  float output = 0;        // 转向输出值
  float speedRatio = 1.0;  // 速度比例（不同状态下速度不同）
  float baseError = (3.5 - data->weighted_pos);  // 基础偏差（3.5是中心，负值=偏右，正值=偏左）

  // ===== 更新误差历史 =====
  // 记录最近20次的偏差，用于丢线时判断搜索方向
  if (data->active_count > 0) {
    errorHistory[historyIndex] = baseError;
    historyIndex = (historyIndex + 1) % 20;  // 循环队列
    if (historySize < 20) historySize++;
  }

  // ===== 根据状态计算转向输出和速度比例 =====
  switch (currentMode) {
    case STATE_STRAIGHT:  // 直线状态
      output = baseError * 1.3;  // 较小的转向增益
      speedRatio = 1.0;          // 全速前进
      break;

    case STATE_GENTLE:  // 缓弯状态
      output = baseError * 1.8;  // 中等转向增益
      speedRatio = 0.8;          // 略微降速
      break;

    case STATE_SHARP:  // 急弯状态
      // 根据累积的转向方向决定转向输出
      if (sharpDirection > 0) output = 8.0;   // 左转
      else if (sharpDirection < 0) output = -8.0;  // 右转
      else output = 0;
      speedRatio = 0.2;  // 大幅降速以完成急弯
      break;

    case STATE_OUT_SHARP:    // 急弯丢线状态
    case STATE_OUT_DEFAULT:  // 默认丢线状态
      {
        // 计算历史偏差的平均值，判断应该往哪个方向搜索
        float avgError = 0;
        if (historySize > 0) {
          for (int i = 0; i < historySize; i++) {
            avgError += errorHistory[i];
          }
          avgError /= historySize;
        }

        // 根据历史平均偏差决定搜索方向
        if (avgError > 0) output = 8.0;       // 历史偏左，继续左转搜索
        else if (avgError < 0) output = -8.0;  // 历史偏右，继续右转搜索
        else output = 8.0;                     // 无历史数据，默认左转

        speedRatio = 0.2;  // 低速搜索
      }
      break;

    case STATE_EDGE:  // 边缘状态
      output = baseError * 2.3;  // 较大的转向增益
      speedRatio = 0.8;          // 略微降速
      break;

    case STATE_RECOVER:  // 恢复状态
      output = baseError * 0.8;  // 较小的转向增益，平滑过渡
      speedRatio = 0.8;          // 略微降速
      break;
  }

  // 速度比例限制
  if (speedRatio > 1.0) speedRatio = 1.0;
  if (data->active_count == 0) speedRatio = 0.2;  // 丢线时低速

  // 记录有效的转向误差（用于丢线恢复判断）
  if (data->active_count > 0) {
    lastValidError = output;
  }

  errorBuffer = output;  // 保存到缓冲区用于调试

  // ===== 转向PD控制 =====
  // 使用PD控制器平滑转向输出，防止震荡
  float rawDerivative = output - prevSteerError;  // 计算原始微分项
  static float filteredDerivative = 0;
  filteredDerivative = filteredDerivative * 0.6 + rawDerivative * 0.4;  // 微分项低通滤波

  float steeringAdjust = (STEER_P * output) + (STEER_D * filteredDerivative);  // PD控制输出
  prevSteerError = output;

  // ===== 计算左右轮目标速度 =====
  // 差速转向：左轮减速、右轮加速 -> 左转；反之 -> 右转
  float dynamicSpeed = BASE_VEL * speedRatio;
  leftCtrl.setpoint = dynamicSpeed - steeringAdjust;   // 左轮速度
  rightCtrl.setpoint = dynamicSpeed + steeringAdjust;  // 右轮速度

  // ===== 速度限制（保持差速比例）=====
  // 如果某个轮子超速，两个轮子同时减速，保持差速比例不变
  if (leftCtrl.setpoint > VEL_LIMIT) {
    float excess = leftCtrl.setpoint - VEL_LIMIT;
    leftCtrl.setpoint -= excess;
    rightCtrl.setpoint -= excess;
  } else if (rightCtrl.setpoint > VEL_LIMIT) {
    float excess = rightCtrl.setpoint - VEL_LIMIT;
    leftCtrl.setpoint -= excess;
    rightCtrl.setpoint -= excess;
  }

  return output;
}

// ==================== 速度PID控制器 ====================

/**
 * 更新速度PID控制器
 * @param ctrl 速度控制器结构指针
 * @param measured 测量的实际速度
 * 功能：使用增量式PID算法，根据速度误差计算控制量（PWM值）
 *       增量式PID的优点：只输出增量，避免积分饱和问题
 */
void updateVelocityPID(VelocityController* ctrl, float measured) {
  // 更新误差历史（保存最近3次误差）
  ctrl->err[2] = ctrl->err[1];  // 上上次误差
  ctrl->err[1] = ctrl->err[0];  // 上次误差
  ctrl->err[0] = ctrl->setpoint - measured;  // 当前误差 = 目标值 - 实际值

  // 增量式PID公式：Δu = Kp*[e(k) - e(k-1)] + Ki*e(k) + Kd*[e(k) - 2e(k-1) + e(k-2)]
  // 等效于：Δu = coeff[0]*err[0] + coeff[1]*err[1] + coeff[2]*err[2]
  ctrl->control += ctrl->coeff[0] * ctrl->err[0] +
                   ctrl->coeff[1] * ctrl->err[1] +
                   ctrl->coeff[2] * ctrl->err[2];

  // 控制量限幅（PWM范围：-255 ~ 255）
  if (ctrl->control > 255) ctrl->control = 255;
  if (ctrl->control < -255) ctrl->control = -255;
}

/**
 * 驱动电机
 * @param ctrl 速度控制器结构指针
 * @param dirPin 方向控制引脚
 * @param pwmPin PWM控制引脚
 * 功能：根据PID控制量输出PWM信号驱动电机
 */
void driveMotor(VelocityController* ctrl, int dirPin, int pwmPin) {
  int pwmValue = (int)ctrl->control;

  // 紧急停止检查
  if (emergencyStop) {
    pwmValue = 0;
    ctrl->control = 0;
  }

  // 设置电机方向和速度
  if (pwmValue >= 0) {
    digitalWrite(dirPin, HIGH);      // 正向旋转
    analogWrite(pwmPin, pwmValue);   // 输出PWM
  } else {
    digitalWrite(dirPin, LOW);       // 反向旋转
    analogWrite(pwmPin, -pwmValue);  // 输出PWM（取绝对值）
  }
}

// ==================== 主控制循环 (10ms定时中断) ====================

/**
 * 主控制更新函数（每10ms被定时器中断调用一次）
 * 功能：完整的控制流程
 *   1. 采集传感器数据
 *   2. 滤除噪声
 *   3. 计算统计信息
 *   4. 判断状态
 *   5. 计算转向输出
 *   6. 测量实际速度
 *   7. 更新PID控制器
 *   8. 输出PWM信号
 */
void updateControl() {
  SensorData sensor;

  // ===== 步骤1-3: 传感器数据处理 =====
  acquireSensorData(&sensor);      // 读取8个传感器
  filterNoiseSegments(&sensor);    // 滤除多余线段
  computeSensorStatistics(&sensor); // 计算位置和统计信息

  // ===== 步骤4: 状态机判断 =====
  determineTrackState(&sensor);

  // ===== 步骤5: 计算转向和目标速度 =====
  calculateSteeringOutput(&sensor);

  // ===== 目标速度低通滤波（平滑速度变化，防止突变）=====
  leftCtrl.setpoint = leftCtrl.setpoint * 0.37 + prevLeftSetpoint * 0.63;
  rightCtrl.setpoint = rightCtrl.setpoint * 0.37 + prevRightSetpoint * 0.63;

  // ===== 步骤6: 测量实际速度 =====
  // 左轮速度计算：脉冲数 / 编码器线数 * 轮周长 * 频率
  // 编码器：780线/圈，轮直径：2cm，轮周长 = 2πr = π*2 = 2π cm
  float rawLeftVel = (leftPulse / 780.0) * 3.1415 * 2.0 * (1000 / CTRL_PERIOD);
  leftPulse = 0;  // 清零脉冲计数
  leftSpeed = leftSpeed * 0.6 + rawLeftVel * 0.4;  // 低通滤波

  // 右轮速度计算（方向相反，所以加负号）
  float rawRightVel = -(rightPulse / 780.0) * 3.1415 * 2.0 * (1000 / CTRL_PERIOD);
  rightPulse = 0;  // 清零脉冲计数
  rightSpeed = rightSpeed * 0.6 + rawRightVel * 0.4;  // 低通滤波

  // ===== 步骤7: 更新PID控制器 =====
  updateVelocityPID(&leftCtrl, leftSpeed);
  updateVelocityPID(&rightCtrl, rightSpeed);

  // ===== 步骤8: 输出PWM控制信号 =====
  driveMotor(&leftCtrl, DIR1, PWM1);
  driveMotor(&rightCtrl, DIR2, PWM2);

  // 保存当前目标速度用于下次滤波
  prevLeftSetpoint = leftCtrl.setpoint;
  prevRightSetpoint = rightCtrl.setpoint;
}

// ==================== 初始化和主循环 ====================

/**
 * Arduino初始化函数（上电后执行一次）
 * 功能：配置所有硬件和软件参数
 */
void setup() {
  // 设置PWM频率为31250Hz（提高电机控制精度）
  TCCR1B = TCCR1B & B11111000 | B00000001;

  // 启动10ms定时器中断
  MsTimer2::set(CTRL_PERIOD, updateControl);
  MsTimer2::start();

  // ===== 配置编码器引脚 =====
  pinMode(ENCODER_A1, INPUT);
  pinMode(ENCODER_B1, INPUT);
  pinMode(ENCODER_A2, INPUT);
  pinMode(ENCODER_B2, INPUT);

  // 绑定编码器中断（用于精确测速）
  // 中断0 -> 数字引脚2（左轮编码器A相）
  // 中断1 -> 数字引脚3（右轮编码器A相）
  attachInterrupt(0, leftEncoderISR, CHANGE);
  attachInterrupt(1, rightEncoderISR, CHANGE);

  // 初始化串口（用于调试输出）
  Serial.begin(9600);

  // ===== 配置电机控制引脚 =====
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);

  // ===== 配置传感器引脚 =====
  pinMode(L4_PIN, INPUT);
  pinMode(L3_PIN, INPUT);
  pinMode(L2_PIN, INPUT);
  pinMode(L1_PIN, INPUT);
  pinMode(R1_PIN, INPUT);
  pinMode(R2_PIN, INPUT);
  pinMode(R3_PIN, INPUT);
  pinMode(R4_PIN, INPUT);

  // ===== 初始化左轮PID控制器 =====
  float dt = CTRL_PERIOD;  // 采样周期（ms）
  // 增量式PID系数计算公式：
  // coeff[0] = Kp * (1 + Ts/Ti + Td/Ts)  -- 当前误差系数
  // coeff[1] = -Kp * (1 + 2*Td/Ts)       -- 上次误差系数
  // coeff[2] = Kp * Td/Ts                 -- 上上次误差系数
  leftCtrl.coeff[0] = P_GAIN * (1 + dt / I_TIME + D_TIME / dt);
  leftCtrl.coeff[1] = -P_GAIN * (1 + 2 * D_TIME / dt);
  leftCtrl.coeff[2] = P_GAIN * D_TIME / dt;
  leftCtrl.setpoint = 0;
  leftCtrl.err[0] = 0;
  leftCtrl.err[1] = 0;
  leftCtrl.err[2] = 0;
  leftCtrl.control = 0;

  // ===== 初始化右轮PID控制器 =====
  rightCtrl.coeff[0] = P_GAIN * (1 + dt / I_TIME + D_TIME / dt);
  rightCtrl.coeff[1] = -P_GAIN * (1 + 2 * D_TIME / dt);
  rightCtrl.coeff[2] = P_GAIN * D_TIME / dt;
  rightCtrl.setpoint = 0;
  rightCtrl.err[0] = 0;
  rightCtrl.err[1] = 0;
  rightCtrl.err[2] = 0;
  rightCtrl.control = 0;
}

/**
 * Arduino主循环函数（无限循环执行）
 * 说明：实际控制逻辑在10ms定时中断中执行（updateControl函数）
 *       这里只是保持程序运行，可以添加调试代码
 */
void loop() {
  delay(100);  // 延时100ms，降低CPU占用

  // 可以在这里添加调试输出，例如：
  // Serial.print("Mode: "); Serial.println(currentMode);
  // Serial.print("Sensors: ");
  // for(int i=0; i<8; i++) Serial.print(sensorBuffer[i]);
  // Serial.println();
}

// ==================== 编码器中断处理函数 ====================

/**
 * 左轮编码器中断服务函数
 * 功能：根据A相和B相的状态判断旋转方向，更新脉冲计数
 * 原理：正交编码器有A、B两相信号，相位差90度
 *       通过判断A相变化时B相的状态，可以确定旋转方向
 */
void leftEncoderISR() {
  if (digitalRead(ENCODER_A1) == LOW) {
    // A相为低电平时
    if (digitalRead(ENCODER_B1) == LOW) leftPulse--;  // B也为低 -> 反转
    else leftPulse++;  // B为高 -> 正转
  } else {
    // A相为高电平时
    if (digitalRead(ENCODER_B1) == LOW) leftPulse++;  // B为低 -> 正转
    else leftPulse--;  // B也为高 -> 反转
  }
}

/**
 * 右轮编码器中断服务函数
 * 功能：与左轮相同，根据A、B相状态判断旋转方向
 */
void rightEncoderISR() {
  if (digitalRead(ENCODER_A2) == LOW) {
    // A相为低电平时
    if (digitalRead(ENCODER_B2) == LOW) rightPulse--;  // B也为低 -> 反转
    else rightPulse++;  // B为高 -> 正转
  } else {
    // A相为高电平时
    if (digitalRead(ENCODER_B2) == LOW) rightPulse++;  // B为低 -> 正转
    else rightPulse--;  // B也为高 -> 反转
  }
}
