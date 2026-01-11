#include "inv_park_test.hpp"
#include "dsp/transform/inv-park.hpp"
#include "pp/transform/inv-park-node.hpp"
#include "pp/pipeline.hpp"
#include "math/index.hpp"
#include <cmath>

#include "logger.hpp"
LOGGER("inv_park_test")

using namespace wibot;
using namespace wibot::motor;

static volatile bool test_failed = false;

constexpr float kTol = 1e-5f;

static void fail(const char* message) {
    LOG_E("[FAIL] %s", message);
    test_failed = true;
    while (1) {
    }  // 嵌入式环境：停止执行
}

static void expect(bool condition, const char* message) {
    if (!condition) {
        fail(message);
    }
}

static void expectNear(float actual, float expected, const char* message, float tol = kTol) {
    if (std::fabs(actual - expected) > tol) {
        LOG_E("[FAIL] %s expected=%f actual=%f", message, expected, actual);
        test_failed = true;
        while (1) {
        }
    }
}

/**
 * @brief 测试零输入
 */
static void test_zero_input() {
    LOG_I("test_zero_input");

    f32 alpha, beta;
    InvPark::transform(0.0f, 0.0f, 0.0f, alpha, beta);

    expectNear(alpha, 0.0f, "Zero input should produce zero alpha");
    expectNear(beta, 0.0f, "Zero input should produce zero beta");
}

/**
 * @brief 测试零角度（theta = 0）
 * 此时旋转坐标系与静止坐标系重合
 */
static void test_zero_angle() {
    LOG_I("test_zero_angle");

    const f32 d = 5.0f;
    const f32 q = 3.0f;
    f32       alpha, beta;

    InvPark::transform(d, q, 0.0f, alpha, beta);

    // theta = 0 时:
    // alpha = d * cos(0) - q * sin(0) = d * 1 - q * 0 = d
    // beta  = d * sin(0) + q * cos(0) = d * 0 + q * 1 = q
    expectNear(alpha, d, "At theta=0, alpha should equal d");
    expectNear(beta, q, "At theta=0, beta should equal q");
}

/**
 * @brief 测试90度角（theta = π/2）
 */
static void test_90_degree_angle() {
    LOG_I("test_90_degree_angle");

    const f32 d     = 4.0f;
    const f32 q     = 2.0f;
    const f32 theta = wibot::kPI / 2.0f;
    f32       alpha, beta;

    InvPark::transform(d, q, theta, alpha, beta);

    // theta = π/2 时:
    // alpha = d * cos(π/2) - q * sin(π/2) = d * 0 - q * 1 = -q
    // beta  = d * sin(π/2) + q * cos(π/2) = d * 1 + q * 0 = d
    expectNear(alpha, -q, "At theta=π/2, alpha should equal -q", 1e-4f);
    expectNear(beta, d, "At theta=π/2, beta should equal d", 1e-4f);
}

/**
 * @brief 测试180度角（theta = π）
 */
static void test_180_degree_angle() {
    LOG_I("test_180_degree_angle");

    const f32 d     = 6.0f;
    const f32 q     = -3.0f;
    const f32 theta = wibot::kPI;
    f32       alpha, beta;

    InvPark::transform(d, q, theta, alpha, beta);

    // theta = π 时:
    // alpha = d * cos(π) - q * sin(π) = d * (-1) - q * 0 = -d
    // beta  = d * sin(π) + q * cos(π) = d * 0 + q * (-1) = -q
    expectNear(alpha, -d, "At theta=π, alpha should equal -d", 1e-4f);
    expectNear(beta, -q, "At theta=π, beta should equal -q", 1e-4f);
}

/**
 * @brief 测试任意角度
 */
static void test_arbitrary_angle() {
    LOG_I("test_arbitrary_angle");

    const f32 d     = 10.0f;
    const f32 q     = 5.0f;
    const f32 theta = wibot::kPI / 4.0f;  // 45度
    f32       alpha, beta;

    InvPark::transform(d, q, theta, alpha, beta);

    const f32 cosTheta      = std::cos(theta);              // ≈ 0.707
    const f32 sinTheta      = std::sin(theta);              // ≈ 0.707
    const f32 expectedAlpha = d * cosTheta - q * sinTheta;  // ≈ 3.536
    const f32 expectedBeta  = d * sinTheta + q * cosTheta;  // ≈ 10.607

    expectNear(alpha, expectedAlpha, "Arbitrary angle alpha", 1e-3f);
    expectNear(beta, expectedBeta, "Arbitrary angle beta", 1e-3f);
}

/**
 * @brief 测试能量守恒（向量模长不变）
 */
static void test_magnitude_preservation() {
    LOG_I("test_magnitude_preservation");

    const f32 d     = 8.0f;
    const f32 q     = 6.0f;
    const f32 theta = wibot::kPI / 3.0f;  // 60度

    f32 alpha, beta;
    InvPark::transform(d, q, theta, alpha, beta);

    const f32 magnitudeDq = std::sqrt(d * d + q * q);
    const f32 magnitudeAb = std::sqrt(alpha * alpha + beta * beta);

    expectNear(magnitudeDq, magnitudeAb, "InvPark should preserve vector magnitude", 1e-3f);
}

/**
 * @brief 测试多个接口一致性
 */
static void test_interface_consistency() {
    LOG_I("test_interface_consistency");

    const f32 d     = 7.0f;
    const f32 q     = -4.0f;
    const f32 theta = wibot::kPI / 6.0f;

    // 方法1: 分量形式
    f32 alpha1, beta1;
    InvPark::transform(d, q, theta, alpha1, beta1);

    // 方法2: 向量形式
    Vector2f dqVec{d, q};
    auto     abVec = InvPark::transform(dqVec, theta);

    expectNear(alpha1, abVec.v1, "Interface consistency: alpha (vector)");
    expectNear(beta1, abVec.v2, "Interface consistency: beta (vector)");
}

/**
 * @brief 测试 Node 封装的基本功能
 */
static void test_node_basic() {
    LOG_I("test_node_basic");

    // 创建节点
    InvParkNode node;

    // 创建存储
    f32 d     = 12.0f;
    f32 q     = 9.0f;
    f32 theta = wibot::kPI / 4.0f;
    f32 alpha, beta;

    // 绑定输入输出
    node.inputs.d.ptr      = &d;
    node.inputs.q.ptr      = &q;
    node.inputs.theta.ptr  = &theta;
    node.outputs.alpha.ptr = &alpha;
    node.outputs.beta.ptr  = &beta;

    // 检查 ready
    expect(node.ready(), "Node should be ready after binding all ports");

    // 执行处理
    node.process();

    // 验证结果
    f32 expectedAlpha, expectedBeta;
    InvPark::transform(d, q, theta, expectedAlpha, expectedBeta);

    expectNear(alpha, expectedAlpha, "Node alpha output", 1e-3f);
    expectNear(beta, expectedBeta, "Node beta output", 1e-3f);
}

/**
 * @brief 测试 Node 在 Pipeline 中的使用
 */
static void test_node_in_pipeline() {
    LOG_I("test_node_in_pipeline");

    // 创建节点
    InvParkNode invParkNode;

    // 创建存储
    f32 d     = 15.0f;
    f32 q     = -7.5f;
    f32 theta = 2.0f * wibot::kPI / 3.0f;  // 120度
    f32 alpha = 0.0f;
    f32 beta  = 0.0f;

    // 构建 pipeline
    PipelineChain<8>        chain;
    PipelineChainBuilder<8> builder;
    builder.addNode(invParkNode);

    // 绑定输入输出（bind是静态方法用于输出）
    builder.bind(invParkNode.outputs.alpha, alpha);
    builder.bind(invParkNode.outputs.beta, beta);

    // 手动绑定输入（输入端口直接赋值指针）
    invParkNode.inputs.d.ptr     = &d;
    invParkNode.inputs.q.ptr     = &q;
    invParkNode.inputs.theta.ptr = &theta;

    // 构建 chain
    builder.build(chain);
    expect(chain.size() == 1, "Pipeline should contain 1 node");

    // 执行一次
    chain.tick();

    // 验证结果
    f32 expectedAlpha, expectedBeta;
    InvPark::transform(d, q, theta, expectedAlpha, expectedBeta);

    expectNear(alpha, expectedAlpha, "Pipeline alpha output", 1e-3f);
    expectNear(beta, expectedBeta, "Pipeline beta output", 1e-3f);
}

/**
 * @brief 测试完整FOC场景
 * 模拟：电流环PID输出 → InvPark → SVPWM
 */
static void test_foc_scenario() {
    LOG_I("test_foc_scenario");

    // 模拟电流环PID输出的dq轴电压指令
    const f32 ud = 10.0f;  // d轴电压（通常较小或为0）
    const f32 uq = 20.0f;  // q轴电压（控制转矩）

    // 转子电角度
    const f32 theta = wibot::kPI / 3.0f;  // 60度电角度

    // 执行反Park变换
    f32 uAlpha, uBeta;
    InvPark::transform(ud, uq, theta, uAlpha, uBeta);

    // 验证：电压幅值应该保持
    const f32 magnitudeDq = std::sqrt(ud * ud + uq * uq);
    const f32 magnitudeAb = std::sqrt(uAlpha * uAlpha + uBeta * uBeta);

    expectNear(magnitudeDq, magnitudeAb, "FOC scenario: voltage magnitude preservation", 1e-3f);

    LOG_I("FOC scenario: ud=%f, uq=%f → uα=%f, uβ=%f", ud, uq, uAlpha, uBeta);
}

// 调用入口：供主程序调用测试
void InvParkTest::runTests() {
    LOG_I("========== InvPark Transform Tests Start ==========");

    test_zero_input();
    test_zero_angle();
    test_90_degree_angle();
    test_180_degree_angle();
    test_arbitrary_angle();
    test_magnitude_preservation();
    test_interface_consistency();
    test_node_basic();
    test_node_in_pipeline();
    test_foc_scenario();

    if (!test_failed) {
        LOG_I("========== All InvPark Tests Passed! ==========");
    } else {
        LOG_E("========== InvPark Tests Failed! ==========");
    }
}
