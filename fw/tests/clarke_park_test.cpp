#include "clarke_park_test.hpp"
#include "dsp/transform/clarke-park.hpp"
#include "pp/transform/clarke-park-node.hpp"
#include "pp/pipeline.hpp"
#include "math/index.hpp"
#include <cmath>

#include "logger.hpp"
LOGGER("clarke_park_test")

using namespace wibot;
using namespace wibot::motor;

static volatile bool test_failed = false;

constexpr float kTol = 1e-5f;

static void fail(const char* message) {
    LOG_E("[FAIL] %s", message);
    test_failed = true;
    while (1) {
    }
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

// ==================== Clarke 变换测试 ====================

/**
 * @brief 测试Clarke变换：零输入
 */
static void test_clarke_zero_input() {
    LOG_I("test_clarke_zero_input");

    f32 alpha, beta;
    Clarke::transform(0.0f, 0.0f, alpha, beta);

    expectNear(alpha, 0.0f, "Clarke zero input: alpha should be 0");
    expectNear(beta, 0.0f, "Clarke zero input: beta should be 0");
}

/**
 * @brief 测试Clarke变换：对称三相电流
 * 
 * ia = I * cos(0°) = I
 * ib = I * cos(120°) = -I/2
 * ic = I * cos(240°) = -I/2
 * 
 * 期望：
 * alpha = ia = I
 * beta = (ia + 2*ib) / √3 = (I - I) / √3 = 0
 */
static void test_clarke_balanced_threephase() {
    LOG_I("test_clarke_balanced_threephase");

    const f32 I  = 10.0f;
    const f32 ia = I;
    const f32 ib = -I / 2.0f;

    auto result = Clarke::transform(ia, ib);

    expectNear(result.v1, I, "Clarke balanced: alpha = ia", 1e-4f);
    expectNear(result.v2, 0.0f, "Clarke balanced: beta = 0", 1e-4f);
}

/**
 * @brief 测试Clarke变换：A相最大电流
 * 
 * ia = I, ib = 0, ic = -I
 * alpha = I
 * beta = I / √3
 */
static void test_clarke_a_phase_max() {
    LOG_I("test_clarke_a_phase_max");

    const f32 I  = 5.0f;
    const f32 ia = I;
    const f32 ib = 0.0f;

    f32 alpha, beta;
    Clarke::transform(ia, ib, alpha, beta);

    expectNear(alpha, I, "Clarke A-max: alpha = ia");
    expectNear(beta, I / std::sqrt(3.0f), "Clarke A-max: beta = ia/√3", 1e-4f);
}

/**
 * @brief 测试Clarke变换：能量守恒
 * 
 * 对于平衡三相系统，应满足：
 * ia² + ib² + ic² = (3/2)(iα² + iβ²)
 */
static void test_clarke_power_invariance() {
    LOG_I("test_clarke_power_invariance");

    const f32 ia = 8.0f;
    const f32 ib = -3.0f;
    const f32 ic = -5.0f;  // ia + ib + ic = 0

    auto result = Clarke::transform(ia, ib);

    const f32 powerABC       = ia * ia + ib * ib + ic * ic;
    const f32 powerAlphaBeta = 1.5f * (result.v1 * result.v1 + result.v2 * result.v2);

    expectNear(powerABC, powerAlphaBeta, "Clarke power invariance", 1e-3f);
}

// ==================== Park 变换测试 ====================

/**
 * @brief 测试Park变换：零输入
 */
static void test_park_zero_input() {
    LOG_I("test_park_zero_input");

    f32 d, q;
    Park::transform(0.0f, 0.0f, 0.0f, d, q);

    expectNear(d, 0.0f, "Park zero input: d should be 0");
    expectNear(q, 0.0f, "Park zero input: q should be 0");
}

/**
 * @brief 测试Park变换：零角度（theta = 0）
 * 
 * theta = 0 时，dq坐标系与αβ坐标系重合
 * d = alpha * cos(0) + beta * sin(0) = alpha
 * q = -alpha * sin(0) + beta * cos(0) = beta
 */
static void test_park_zero_angle() {
    LOG_I("test_park_zero_angle");

    const f32 alpha = 5.0f;
    const f32 beta  = 3.0f;

    Vector2f dq = Park::transform(Vector2f{alpha, beta}, 0.0f);

    expectNear(dq.v1, alpha, "Park theta=0: d = alpha");
    expectNear(dq.v2, beta, "Park theta=0: q = beta");
}

/**
 * @brief 测试Park变换：90度角
 */
static void test_park_90_degree() {
    LOG_I("test_park_90_degree");

    const f32 alpha = 4.0f;
    const f32 beta  = 2.0f;
    const f32 theta = wibot::kPI / 2.0f;

    f32 d, q;
    Park::transform(alpha, beta, theta, d, q);

    // theta = π/2:
    // d = alpha * cos(π/2) + beta * sin(π/2) = beta
    // q = -alpha * sin(π/2) + beta * cos(π/2) = -alpha
    expectNear(d, beta, "Park theta=π/2: d = beta", 1e-4f);
    expectNear(q, -alpha, "Park theta=π/2: q = -alpha", 1e-4f);
}

/**
 * @brief 测试Park变换：能量守恒
 */
static void test_park_magnitude_preservation() {
    LOG_I("test_park_magnitude_preservation");

    const f32 alpha = 6.0f;
    const f32 beta  = 8.0f;
    const f32 theta = wibot::kPI / 3.0f;

    Vector2f dq = Park::transform(Vector2f{alpha, beta}, theta);

    const f32 magnitudeAlphaBeta = std::sqrt(alpha * alpha + beta * beta);
    const f32 magnitudeDq        = std::sqrt(dq.v1 * dq.v1 + dq.v2 * dq.v2);

    expectNear(magnitudeAlphaBeta, magnitudeDq, "Park magnitude preservation", 1e-3f);
}

// ==================== Clarke → Park 级联测试 ====================

/**
 * @brief 测试Clarke和Park级联变换
 * 
 * 三相 → Clarke → αβ → Park → dq
 */
static void test_clarke_park_cascade() {
    LOG_I("test_clarke_park_cascade");

    // 输入：平衡三相电流
    const f32 I          = 10.0f;
    const f32 theta_elec = wibot::kPI / 6.0f;  // 30度电角度

    // 生成三相电流（与电角度同步）
    const f32 ia = I * std::cos(theta_elec);
    const f32 ib = I * std::cos(theta_elec - 2.0f * wibot::kPI / 3.0f);

    // Clarke变换
    auto alphaBeta = Clarke::transform(ia, ib);

    // Park变换
    auto dq = Park::transform(alphaBeta, theta_elec);

    // 理想情况下，对于与电角度同步的电流：
    // id 应接近 I（所有电流在d轴）
    // iq 应接近 0
    expectNear(dq.v1, I, "Clarke→Park cascade: id ≈ I", 0.5f);
    expectNear(dq.v2, 0.0f, "Clarke→Park cascade: iq ≈ 0", 0.5f);
}

// ==================== Node 测试 ====================

/**
 * @brief 测试ClarkeNode基本功能
 */
static void test_clarke_node_basic() {
    LOG_I("test_clarke_node_basic");

    ClarkeNode node;

    f32 ia = 7.0f;
    f32 ib = -2.0f;
    f32 alpha, beta;

    // 绑定
    node.inputs.ia.ptr     = &ia;
    node.inputs.ib.ptr     = &ib;
    node.outputs.alpha.ptr = &alpha;
    node.outputs.beta.ptr  = &beta;

    expect(node.ready(), "ClarkeNode should be ready");

    // 处理
    node.process();

    // 验证
    f32 expectedAlpha, expectedBeta;
    Clarke::transform(ia, ib, expectedAlpha, expectedBeta);

    expectNear(alpha, expectedAlpha, "ClarkeNode alpha output");
    expectNear(beta, expectedBeta, "ClarkeNode beta output");
}

/**
 * @brief 测试ParkNode基本功能
 */
static void test_park_node_basic() {
    LOG_I("test_park_node_basic");

    ParkNode node;

    f32 alpha = 5.0f;
    f32 beta  = 3.0f;
    f32 theta = wibot::kPI / 4.0f;
    f32 d, q;

    // 绑定
    node.inputs.alpha.ptr = &alpha;
    node.inputs.beta.ptr  = &beta;
    node.inputs.theta.ptr = &theta;
    node.outputs.d.ptr    = &d;
    node.outputs.q.ptr    = &q;

    expect(node.ready(), "ParkNode should be ready");

    // 处理
    node.process();

    // 验证
    f32 expectedD, expectedQ;
    Park::transform(alpha, beta, theta, expectedD, expectedQ);

    expectNear(d, expectedD, "ParkNode d output");
    expectNear(q, expectedQ, "ParkNode q output");
}

/**
 * @brief 测试完整FOC前向变换链
 * 
 * 三相电流 → Clarke → Park
 */
static void test_foc_forward_chain() {
    LOG_I("test_foc_forward_chain");

    // 创建节点
    ClarkeNode clarkeNode;
    ParkNode   parkNode;

    // 数据存储
    f32 ia    = 8.0f;
    f32 ib    = -4.0f;
    f32 theta = wibot::kPI / 3.0f;
    f32 alpha = 0.0f;
    f32 beta  = 0.0f;
    f32 id    = 0.0f;
    f32 iq    = 0.0f;

    // 构建pipeline
    PipelineChain<8>        chain;
    PipelineChainBuilder<8> builder;

    builder.addNode(clarkeNode);
    builder.addNode(parkNode);

    // 绑定
    clarkeNode.inputs.ia.ptr = &ia;
    clarkeNode.inputs.ib.ptr = &ib;
    builder.bind(clarkeNode.outputs.alpha, alpha);
    builder.bind(clarkeNode.outputs.beta, beta);

    parkNode.inputs.alpha.ptr = &alpha;
    parkNode.inputs.beta.ptr  = &beta;
    parkNode.inputs.theta.ptr = &theta;
    builder.bind(parkNode.outputs.d, id);
    builder.bind(parkNode.outputs.q, iq);

    builder.build(chain);
    expect(chain.size() == 2, "FOC chain should contain 2 nodes");

    // 执行
    chain.tick();

    // 验证结果非零
    expect(std::fabs(id) > 0.1f || std::fabs(iq) > 0.1f,
           "FOC chain should produce non-zero output");

    LOG_I("FOC forward: ia=%f, ib=%f → α=%f, β=%f → id=%f, iq=%f", ia, ib, alpha, beta, id, iq);
}

// 调用入口
void ClarkeParkTest::runTests() {
    LOG_I("========== Clarke & Park Transform Tests Start ==========");

    test_clarke_zero_input();
    test_clarke_balanced_threephase();
    test_clarke_a_phase_max();
    test_clarke_power_invariance();

    test_park_zero_input();
    test_park_zero_angle();
    test_park_90_degree();
    test_park_magnitude_preservation();

    test_clarke_park_cascade();

    test_clarke_node_basic();
    test_park_node_basic();
    test_foc_forward_chain();

    if (!test_failed) {
        LOG_I("========== All Clarke & Park Tests Passed! ==========");
    } else {
        LOG_E("========== Clarke & Park Tests Failed! ==========");
    }
}
