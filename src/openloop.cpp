#include <SimpleFOC.h>
#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

// #define OPENLOOP 
// #define test
#define MOTOR_NAME "AK10_9"
#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.090) /* Unit in ohm. */
#define MOTOR_KV (100)                 /* Unit in rpm/V. */

// DRV8302 控制腳位
#define INH_A (3)
#define INH_B (5)
#define INH_C (6)

#define Enable (PC0)  // DRV8302 啟用腳位
#define M_PWM_PIN (PC1)
#define M_OC (PB1)  // DRV8302 過流保護腳位
#define OC_ADJ (PB2)  // DRV8302 過流保護調整腳位

// AS5047P sensor腳位
/*
   * SPI SCLK: D13 pin.
   * SPI MISO: D12 pin.
   * SPI MOSI: D11 pin.
   */
#define AS5047P_SPI_CS (10)
/* AS5047P encoder. */
#define AS5047P_REG_ANGLECOM (0x3FFF) /* Measured angle with dynamic angle error compensation(DAEC). */
#define AS5047P_REG_ANGLEUNC (0x3FFE) /* Measured angle without DAEC. */

#define OPENLOOP

const float VEL_FWD   = 30.0f;   // 觸發時正轉速度 (rad/s)
const float VEL_BACK  = 25.0f;   // 回程反轉速度 (rad/s)
const float ACCEL     = 80.0f;   // 加速斜率 (rad/s^2)
const float DECEL     = 120.0f;  // 減速斜率 (rad/s^2)
const uint32_t HOLD_MS = 120;    // 訊號逾時視窗 (ms)
const float ANGLE_MAX = 50.0f;    // 允許前進的最大虛擬角度 (rad) 可視行程限制

float target_vel = 0.0f;         // 下給馬達的速度命令
float virt_angle = 0.0f;         // 累積的「虛擬角度」（只在軟體裡算）
uint32_t last_rx_us = 0;


HardwareSerial Serial1(USART1);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS); 
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C,Enable);
MagneticSensorSPI angleSensor = MagneticSensorSPI(AS5047P_SPI_CS, 14, AS5047P_REG_ANGLECOM);

// Can
struct can_frame canMsg;
MCP2515 mcp2515(10);

// float center_angle = 0;   // 中心位置
// float spring_k = 5;     // 彈簧係數 (越大回復力越強)
// float max_angle = 10;  // 90° (單位: 弧度)

Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }


void onReadAngle(char *)
{
#ifndef OPENLOOP
  angleSensor.update();
  float angle = angleSensor.getAngle();

  angle -= motor.sensor_offset;
  // angle *= -1;

  char sign;
  if (angle >= 0)
  {
    sign = '+';
  }
  else
  {
    sign = '-';
  }

  angle = fabs(angle); /* Absolute value. */

  /* Send. */
  Serial.printf("%c%d%d%d.%d%d%d\r\n",
                sign,
                (int)((int)(angle) / 100 % 10),
                (int)((int)(angle) / 10 % 10),
                (int)((int)(angle) / 1 % 10),
                (int)((int)(angle * 10) / 1 % 10),
                (int)((int)(angle * 100) / 1 % 10),
                (int)((int)(angle * 1000) / 1 % 10));

#endif
}

// float angleError(float a, float b) {
//   float diff = a - b;
//   while (diff >  _PI) diff -= _2PI;
//   while (diff < -_PI) diff += _2PI;
//   return diff;
// }

void drv8302Setup() 
{
  // 設定 PWM 模式為 3PWM
  pinMode(M_PWM_PIN, OUTPUT);
  digitalWrite(M_PWM_PIN, HIGH); // HIGH = 3PWM 模式

  // 設定 OC 模式為 cycle-by-cycle 模式
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);  // LOW = 每次過電流都限制，不關閉

  // OC trip 門檻設為最大（除非你要用電阻分壓調整）
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH); // HIGH = 最大電流門檻
}

void setup() 
{
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");
  command.add('A', onReadAngle, "angle");

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  canMsg.can_id  = 0x11;   // 訊息 ID
  canMsg.can_dlc = 2;      // 資料長度
  canMsg.data[0] = 0x00;   // 預設資料
  canMsg.data[1] = 0x11;

#ifndef OPENLOOP
  /* Configure angle/Position sensor. https://docs.simplefoc.com/magnetic_sensor_spi */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor); 
#endif
  drv8302Setup();
  driver.voltage_power_supply = 15;
  driver.pwm_frequency = 20000; // 設定 PWM 頻率為 20kHz
  driver.init();
  motor.linkDriver(&driver);

  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  motor.KV_rating = MOTOR_KV ;
  motor.voltage_limit = 6; 
  motor.velocity_limit = 10; // 在開迴路模式下無作用
  motor.current_limit = 2; // 在開迴路模式下無作用
  //motor.motion_downsample = 5;

  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; /* SVPWM. */
  motor.torque_controller = TorqueControlType::voltage;     /* No current sensing so only voltage controller. */
#ifdef OPENLOOP
  motor.controller = MotionControlType::velocity_openloop;
#else
  motor.controller = MotionControlType::angle;
#endif  

  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.001;
  // motor.PID_velocity.output_ramp = 10; /* Unit in volts/s. */
  motor.LPF_velocity.Tf = 0.01;
  //motor.velocity_limit = 15; /* Unit in rad/s. */

  /*Angle/Position control loop setup. */
  motor.P_angle.P = 5;
  //motor.P_angle.I = 0.5;
  //motor.P_angle.D = 0.05;
  //motor.P_angle.output_ramp = 500; /* Acceleration limit(?), unit in rad/s^2. */

  motor.init();
  motor.initFOC(); /* Start FOC and aligh encoder. */

#ifndef OPENLOOP
  angleSensor.update();
  motor.target = angleSensor.getAngle(); /* Set the initial target value. */
#else
  motor.target = 0;
#endif
  Serial.println(motor.target, 3);

  Serial.println(F("那ㄟ溝派起阿"));
  // center_angle = angleSensor.getAngle();
  _delay(1000);
}

void loop() 
{
  
  // 它會根據 motor.target 的值來執行 FOC 控制

  motor.loopFOC();
  // angleSensor.update();
  // float current_angle = angleSensor.getAngle();

  // // 計算與中心的最短角度差
  // float error = angleError(current_angle, center_angle);

  // // 限制範圍到 ±90°
  // if (error >  max_angle) error =  max_angle;
  // if (error < -max_angle) error = -max_angle;

  // 線性彈簧: 誤差 / 最大角度 * 最大扭力
  // float torque_voltage = - (error / max_angle) * spring_k;

  static uint32_t t_prev = micros();
  uint32_t t_now = micros();
  float dt = (t_now - t_prev) * 1e-6f;
  t_prev = t_now;

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    last_rx_us = t_now;
  }
  bool active = (t_now - last_rx_us) <= (HOLD_MS * 1000UL);

  // --- 決策：要前進、回程、或停 ---
  float tgt_vel;
  if (active) {
    // 觸發：前進到 ANGLE_MAX 為止
    tgt_vel = (virt_angle < ANGLE_MAX) ? VEL_FWD : 0.0f;
  } else {
    // 未觸發：若還沒回到 0 就反轉回去
    tgt_vel = (virt_angle > 0.0f) ? -VEL_BACK : 0.0f;
  }

  // 以加減速斜率逼近目標速度（避免抖/跳）
  float slope = (fabsf(tgt_vel) > fabsf(target_vel)) ? ACCEL : DECEL;
  float dv = slope * dt;
  if (fabsf(tgt_vel - target_vel) <= dv) target_vel = tgt_vel;
  else target_vel += copysignf(dv, tgt_vel - target_vel);

  // 依實際下給的速度，更新虛擬角度並限幅到 [0, ANGLE_MAX]
  virt_angle += target_vel * dt;
  if (virt_angle < 0.0f) virt_angle = 0.0f;
  if (virt_angle > ANGLE_MAX) virt_angle = ANGLE_MAX;

  motor.move(target_vel);

  // // 偵測用
  // Serial.print("error_deg: "); Serial.print(error * 180.0 / PI, 1);
  // Serial.print(", torque: "); Serial.println(torque_voltage, 2);
  // motor.move(torque_voltage);
  // angleSensor.update();
  command.run();
  
}