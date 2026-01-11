#include "svpwm.hpp"
#include <algorithm>
#include <cmath>

namespace wibot::motor {

Svpwm::Svpwm(float busVoltage) : _busVoltage(busVoltage) {}

Svpwm::~Svpwm() {}

void Svpwm::setBusVoltage(float voltage) { _busVoltage = voltage; }

SvpwmOutput Svpwm::calculate(float vAlpha, float vBeta) {
  // 限制电压到线性调制区
  limitVoltage(vAlpha, vBeta);

  // 判断所在扇区
  uint8_t sector = determineSector(vAlpha, vBeta);

  // 归一化电压 (相对于母线电压)
  float vAlphaNorm = vAlpha / _busVoltage;
  float vBetaNorm = vBeta / _busVoltage;

  // 根据扇区计算 X, Y, Z
  float x = vBetaNorm;
  float y = kSQRT3_2 * vAlphaNorm - 0.5f * vBetaNorm;
  float z = -kSQRT3_2 * vAlphaNorm - 0.5f * vBetaNorm;

  // 计算各矢量作用时间
  float t1 = 0.0f;
  float t2 = 0.0f;
  switch (sector) {
  case 1:
    t1 = -z;
    t2 = -y;
    break;
  case 2:
    t1 = z;
    t2 = x;
    break;
  case 3:
    t1 = -x;
    t2 = -z;
    break;
  case 4:
    t1 = y;
    t2 = x;
    break;
  case 5:
    t1 = -x;
    t2 = -y;
    break;
  case 6:
    t1 = z;
    t2 = y;
    break;
  default:
    break;
  }

  // 限制 t1 和 t2 在 [0, 1] 范围内
  t1 = std::max(0.0f, std::min(1.0f, t1));
  t2 = std::max(0.0f, std::min(1.0f, t2));

  // 计算零矢量时间
  float t0 = 1.0f - t1 - t2;

  // 如果 t0 < 0，说明过调制，需要重新分配
  if (t0 < 0.0f) {
    float sum = t1 + t2;
    if (sum > 0.0f) {
      t1 = t1 / sum;
      t2 = t2 / sum;
    }
    t0 = 0.0f;
  }

  // 七段式 SVPWM：将零矢量时间平均分配到两端
  float t0_half = t0 * 0.5f;

  SvpwmOutput output;
  output.sector = sector;

  // 根据扇区生成三相占空比
  switch (sector) {
  case 1: // U+V-
    output.dutyU = t0_half + t1 + t2;
    output.dutyV = t0_half + t2;
    output.dutyW = t0_half;
    break;

  case 2: // V+U-
    output.dutyU = t0_half + t1;
    output.dutyV = t0_half + t1 + t2;
    output.dutyW = t0_half;
    break;

  case 3: // V+W-
    output.dutyU = t0_half;
    output.dutyV = t0_half + t1 + t2;
    output.dutyW = t0_half + t2;
    break;

  case 4: // W+V-
    output.dutyU = t0_half;
    output.dutyV = t0_half + t1;
    output.dutyW = t0_half + t1 + t2;
    break;

  case 5: // W+U-
    output.dutyU = t0_half + t2;
    output.dutyV = t0_half;
    output.dutyW = t0_half + t1 + t2;
    break;

  case 6: // U+W-
    output.dutyU = t0_half + t1 + t2;
    output.dutyV = t0_half;
    output.dutyW = t0_half + t1;
    break;

  default:
    output.dutyU = 0.5f;
    output.dutyV = 0.5f;
    output.dutyW = 0.5f;
    break;
  }

  // ADC 触发时机设置：中心对齐模式在顶点触发
  output.dutyAdcTrig = 1.0f;

  return output;
}

SvpwmOutput Svpwm::calculatePolar(float angle, float magnitude) {
  // 将极坐标转换为 α-β 坐标
  auto sincos = _math.sincos(angle, magnitude);

  return calculate(sincos.v2, sincos.v1);
}

uint8_t Svpwm::determineSector(float vAlpha, float vBeta) {
  // 使用三个判断变量
  // N = 4*C + 2*B + A
  // A = sign(Vbeta)
  // B = sign(√3/2 * Valpha - 1/2 * Vbeta)
  // C = sign(-√3/2 * Valpha - 1/2 * Vbeta)

  uint8_t sector;

  // 判断 A: Vbeta 的符号
  int a = (vBeta > 0) ? 1 : 0;

  // 判断 B: (√3/2 * Valpha - 1/2 * Vbeta) 的符号
  float temp1 = kSQRT3_2 * vAlpha - 0.5f * vBeta;
  int b = (temp1 > 0) ? 1 : 0;

  // 判断 C: (-√3/2 * Valpha - 1/2 * Vbeta) 的符号
  float temp2 = -kSQRT3_2 * vAlpha - 0.5f * vBeta;
  int c = (temp2 > 0) ? 1 : 0;

  // 计算扇区编号
  int n = 4 * c + 2 * b + a;

  // N 到扇区的映射表
  // N:      1  2  3  4  5  6
  // Sector: 2  6  1  4  3  5
  static const uint8_t sectorMap[7] = {0, 2, 6, 1, 4, 3, 5};

  if (n >= 1 && n <= 6) {
    sector = sectorMap[n];
  } else {
    sector = 1; // 默认扇区1
  }

  return sector;
}
float Svpwm::getMaxModulationVoltage() const {
  // SVPWM 线性调制区的最大电压为母线电压的 1/√3 倍
  // 这是由于六边形边界的限制
  constexpr float kInvSqrt3 = 0.57735026919f; // 1/√3
  return _busVoltage * kInvSqrt3;
}
void Svpwm::limitVoltage(float &vAlpha, float &vBeta) const {
  // 计算电压矢量幅值
  float magnitude = std::sqrt(vAlpha * vAlpha + vBeta * vBeta);

  // 获取最大可调制电压 
  float maxVoltage = getMaxModulationVoltage();

  // 如果超出线性区，则缩放到最大值
  if (magnitude > maxVoltage) {
    float scale = maxVoltage / magnitude;
    vAlpha *= scale;
    vBeta *= scale;
  }
}

} // namespace wibot::motor
