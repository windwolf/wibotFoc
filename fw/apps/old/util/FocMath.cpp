//
// Created by zhouj on 2024/6/28.
//

#include "FocMath.hpp"

namespace wibot::motor {
void FocMath::abc2ab(Vector3f abc, Vector2f& ab) {
    ab.v1 = abc.v1 - (abc.v2 + abc.v3) / 2.0f;
    ab.v2 = (abc.v2 - abc.v3) * kSQRT3_2;
};
void FocMath::ab2dq(Vector2f ab, float theta, Vector2f& dq) {
    float sin;
    float cos;
    Math::sincos(theta, &sin, &cos);
    dq.v1 = ab.v1 * cos + ab.v2 * sin;
    dq.v2 = -ab.v1 * sin + ab.v2 * cos;
};

void FocMath::dq2ab(Vector2f dq, float theta, Vector2f& ab) {
    float sin;
    float cos;
    Math::sincos(theta, &sin, &cos);
    ab.v1 = dq.v1 * cos - dq.v2 * sin;
    ab.v2 = dq.v2 * cos + dq.v1 * sin;
};

uint8_t FocMath::getSection(float theta) {
    uint8_t section = 0;
    if (theta >= 0.0f && theta < kPI_3) {
        section = 1;
    } else if (theta >= kPI_3 && theta < k2PI_3) {
        section = 2;
    } else if (theta >= k2PI_3 && theta < kPI) {
        section = 3;
    } else if (theta >= kPI && theta < k4PI_3) {
        section = 4;
    } else if (theta >= k4PI_3 && theta < k5PI_3) {
        section = 5;
    } else if (theta >= k5PI_3 && theta < k2PI) {
        section = 6;
    }
    return section;
};

}  // namespace wibot::motor
