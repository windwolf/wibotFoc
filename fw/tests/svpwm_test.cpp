#include "svpwm_test.hpp"
#include "components/svpwm.hpp"

#include "math/index.hpp"

#include "logger.hpp"
LOGGER("svpwm_test")

using namespace wibot;
using namespace wibot::motor;

static volatile bool test_failed = false;

constexpr float kTol = 1e-5f;

static void fail(const char *message) {
  LOG_E("[FAIL] %s", message);
  test_failed = true;
  while (1) {
  } // 嵌入式环境：停止执行
}

static void expect(bool condition, const char *message) {
  if (!condition) {
    fail(message);
  }
}

static void expectNear(float actual, float expected, const char *message,
                       float tol = kTol) {
  if (fabs(actual - expected) > tol) {
    LOG_E("[FAIL] %s expected=%f actual=%f", message, expected, actual);
    test_failed = true;
    while (1) {
    }
  }
}

static void expectInRange(float value, float min, float max,
                          const char *message) {
  if (value < min - kTol || value > max + kTol) {
    LOG_E("[FAIL] %s value=%f range=[%f, %f]", message, value, min, max);
    test_failed = true;
    while (1) {
    }
  }
}

static void test_determine_sector() {
  struct Case {
    float vAlpha;
    float vBeta;
    uint8_t expected;
  };

  const Case cases[] = {{0.5f, 0.5f, 1},   // a=1, b=1, c=0 -> n=3
                        {0.0f, 1.0f, 2},   // a=1, b=0, c=0 -> n=1
                        {1.0f, 0.0f, 6},   // a=0, b=1, c=0 -> n=2
                        {-0.5f, 0.5f, 3},  // a=1, b=0, c=1 -> n=5
                        {-1.0f, 0.0f, 4},  // a=0, b=0, c=1 -> n=4
                        {0.0f, -1.0f, 5}}; // a=0, b=1, c=1 -> n=6

  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
    const Case &c = cases[i];
    uint8_t sector = Svpwm::determineSector(c.vAlpha, c.vBeta);
    if (sector != c.expected) {
      LOG_E("[FAIL] Sector mismatch for (alpha=%f, beta=%f) expected=%d "
            "actual=%d",
            c.vAlpha, c.vBeta, (int)c.expected, (int)sector);
      test_failed = true;
      while (1) {
      }
    }
  }
}

static void test_limit_voltage() {
  Svpwm svpwm(24.0f);
  float vAlpha = 20.0f;
  float vBeta = 20.0f;

  svpwm.limitVoltage(vAlpha, vBeta);
  Math math;
  float magnitude = math.sqrt<f32>(vAlpha * vAlpha + vBeta * vBeta);
  float maxAllowed = svpwm.getMaxModulationVoltage();
  expect(magnitude <= maxAllowed + 1e-4f,
         "limitVoltage should clamp vector into linear region");
  expectNear(vAlpha, vBeta, "limitVoltage should scale components evenly");
}

static void test_zero_vector_duties() {
  Svpwm svpwm(24.0f);
  SvpwmOutput out = svpwm.calculate(0.0f, 0.0f);

  expect(out.sector == 1, "Zero vector should default to sector 1");
  expectNear(out.dutyU, 0.5f, "Zero vector U duty");
  expectNear(out.dutyV, 0.5f, "Zero vector V duty");
  expectNear(out.dutyW, 0.5f, "Zero vector W duty");
  expectNear(out.dutyAdcTrig, 1.0f, "ADC trigger should stay at edge");
}

static void test_axis_aligned_duties() {
  Svpwm svpwm(24.0f);

  // Alpha axis, positive
  SvpwmOutput alphaOut = svpwm.calculate(10.0f, 0.0f);
  expect(alphaOut.sector == 6, "Alpha axis should fall in sector 6");
  expectNear(alphaOut.dutyU, 0.68042195f, "Alpha axis U duty");
  expectNear(alphaOut.dutyV, 0.31957805f, "Alpha axis V duty");
  expectNear(alphaOut.dutyW, 0.31957805f, "Alpha axis W duty");
  expectInRange(alphaOut.dutyU, 0.0f, 1.0f, "Alpha axis dutyU range");
  expectInRange(alphaOut.dutyV, 0.0f, 1.0f, "Alpha axis dutyV range");
  expectInRange(alphaOut.dutyW, 0.0f, 1.0f, "Alpha axis dutyW range");

  // Beta axis, positive
  SvpwmOutput betaOut = svpwm.calculate(0.0f, 10.0f);
  expect(betaOut.sector == 2, "Beta axis should fall in sector 2");
  expectNear(betaOut.dutyU, 0.29166669f, "Beta axis U duty");
  expectNear(betaOut.dutyV, 0.70833331f, "Beta axis V duty");
  expectNear(betaOut.dutyW, 0.29166669f, "Beta axis W duty");
  expectInRange(betaOut.dutyU, 0.0f, 1.0f, "Beta axis dutyU range");
  expectInRange(betaOut.dutyV, 0.0f, 1.0f, "Beta axis dutyV range");
  expectInRange(betaOut.dutyW, 0.0f, 1.0f, "Beta axis dutyW range");
}

static void test_polar_vs_cartesian() {
  Svpwm svpwm(24.0f);
  float angle = wibot::kPI / 6.0f; // 30 degrees
  float magnitude = 8.0f;
  Math math;
  auto ab = math.sincos(angle, magnitude);
  float alpha = ab.v1;
  float beta = ab.v2;

  SvpwmOutput cart = svpwm.calculate(alpha, beta);
  SvpwmOutput polar = svpwm.calculatePolar(angle, magnitude);

  expect(cart.sector == polar.sector,
         "Polar and cartesian sectors should match");
  expectNear(cart.dutyU, polar.dutyU, "Polar/cartesian U duty");
  expectNear(cart.dutyV, polar.dutyV, "Polar/cartesian V duty");
  expectNear(cart.dutyW, polar.dutyW, "Polar/cartesian W duty");
  expectNear(cart.dutyAdcTrig, polar.dutyAdcTrig,
             "Polar/cartesian ADC trigger");
}

// 调用入口：供主程序调用测试
void SvpwmTest::runTests() {
  test_determine_sector();
  test_limit_voltage();
  test_zero_vector_duties();
  test_axis_aligned_duties();
  test_polar_vs_cartesian();

  if (!test_failed) {
    LOG_I("All SVPWM unit tests passed.");
  } else {
    LOG_E("SVPWM unit tests failed.");
  }
}
