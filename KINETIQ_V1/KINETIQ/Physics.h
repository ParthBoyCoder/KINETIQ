#pragma once
#include <Arduino.h>
#include <math.h>
#include "Config.h"

namespace Physics {

struct FrictionResult {
  float velocityA;
  float velocityB;
  float acceleration;
  float muK;
  bool valid;
};

struct CollisionResult {
  float u1, v1, u2, v2;
  float initialMomentum;
  float finalMomentum;
  float momentumDifference;
  float momentumErrorPercent;
  float initialKE;
  float finalKE;
  float restitution;
  bool valid;
};

inline float speedFromGate(float effectiveLengthM, uint32_t blockedUs) {
  if (blockedUs == 0) return NAN;
  const float seconds = blockedUs / 1000000.0f;
  return effectiveLengthM / seconds;
}

inline FrictionResult calculateFriction(uint32_t timeA, uint32_t timeB) {
  FrictionResult r{};
  r.valid = false;

  if (FRICTION::OBJECT_LENGTH_A_M <= 0 ||
      FRICTION::OBJECT_LENGTH_B_M <= 0 ||
      FRICTION::GATE_SPACING_M <= 0 ||
      timeA == 0 || timeB == 0) return r;

  r.velocityA = speedFromGate(FRICTION::OBJECT_LENGTH_A_M, timeA);
  r.velocityB = speedFromGate(FRICTION::OBJECT_LENGTH_B_M, timeB);

  const float s = FRICTION::GATE_SPACING_M;
  r.acceleration = (r.velocityB * r.velocityB -
                    r.velocityA * r.velocityA) / (2.0f * s);

  const float theta = FRICTION::ANGLE_DEG * PI / 180.0f;
  const float denominator = FRICTION::G * cos(theta);

  if (fabs(denominator) < 0.0001f) return r;

  // Positive direction is downhill from gate A to gate B.
  r.muK = (FRICTION::G * sin(theta) - r.acceleration) / denominator;
  r.valid = isfinite(r.velocityA) && isfinite(r.velocityB) &&
            isfinite(r.acceleration) && isfinite(r.muK);
  return r;
}

inline CollisionResult calculateCollision(uint32_t aBeforeUs,
                                          uint32_t aAfterUs,
                                          uint32_t bBeforeUs,
                                          uint32_t bAfterUs) {
  CollisionResult r{};
  r.valid = false;

  if (COLLISION::BLOCK_1_LENGTH_M <= 0 ||
      COLLISION::BLOCK_2_LENGTH_M <= 0 ||
      COLLISION::BLOCK_1_MASS_KG <= 0 ||
      COLLISION::BLOCK_2_MASS_KG <= 0) return r;

  const float aBefore = speedFromGate(COLLISION::BLOCK_1_LENGTH_M, aBeforeUs);
  const float aAfter  = speedFromGate(COLLISION::BLOCK_1_LENGTH_M, aAfterUs);
  const float bBefore = speedFromGate(COLLISION::BLOCK_2_LENGTH_M, bBeforeUs);
  const float bAfter  = speedFromGate(COLLISION::BLOCK_2_LENGTH_M, bAfterUs);

  if (!isfinite(aBefore) || !isfinite(aAfter) ||
      !isfinite(bBefore) || !isfinite(bAfter)) return r;

  // Coordinate convention: rightward is positive.
  // Sensor A is left of the collision zone; sensor B is right.
  // Each block approaches the centre, then returns through its own gate.
  r.u1 = +aBefore;
  r.v1 = -aAfter;
  r.u2 = -bBefore;
  r.v2 = +bAfter;

  const float m1 = COLLISION::BLOCK_1_MASS_KG;
  const float m2 = COLLISION::BLOCK_2_MASS_KG;

  r.initialMomentum = m1 * r.u1 + m2 * r.u2;
  r.finalMomentum = m1 * r.v1 + m2 * r.v2;
  r.momentumDifference = r.finalMomentum - r.initialMomentum;

  const float initialMagnitude = fabs(r.initialMomentum);
  if (initialMagnitude > 0.000001f) {
    r.momentumErrorPercent =
      fabs(r.momentumDifference) / initialMagnitude * 100.0f;
  } else {
    r.momentumErrorPercent = NAN;
  }

  r.initialKE = 0.5f * m1 * r.u1 * r.u1 +
                0.5f * m2 * r.u2 * r.u2;
  r.finalKE = 0.5f * m1 * r.v1 * r.v1 +
              0.5f * m2 * r.v2 * r.v2;

  const float relativeInitial = fabs(r.u1 - r.u2);
  r.restitution = (relativeInitial > 0.000001f)
    ? fabs(r.v2 - r.v1) / relativeInitial
    : NAN;

  r.valid = isfinite(r.initialMomentum) && isfinite(r.finalMomentum) &&
            isfinite(r.initialKE) && isfinite(r.finalKE);
  return r;
}

} // namespace Physics
