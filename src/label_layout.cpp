#include "label_layout.h"

#include <math.h>
#include <string.h>

namespace RadarLabels {
namespace {

static constexpr uint32_t kStateTtlMs = 60000;
static constexpr float kAnchorJumpPx = 80.0f;
static constexpr float kMinimumSymbolGapPx = 2.0f;
static constexpr float kPreferredSymbolGapPx = 4.0f;
static constexpr float kNormalMaxGapPx = 64.0f;
static constexpr float kPriorityMaxGapPx = 96.0f;
static constexpr float kMaxMovementPxPerSecond = 64.0f;
static constexpr float kMovementDeadZonePx = 0.25f;
static constexpr bool kResolveOverlappingLabels = false;
static constexpr float kCourseAvoidDistancePx = 80.0f;
static constexpr float kCourseConeCosineSquared = 0.58682409f;
static constexpr float kInverseSqrtTwo = 0.70710678f;
static constexpr float kOrbitMinExcessPx = 0.75f;
static constexpr float kOrbitForceLimitPx = 2.0f;
static constexpr size_t kOrbitLabelsPerFrame = 16;
static constexpr float kLabelCollisionMarginPx = 2.5f;
static constexpr uint32_t kOrbitDirectionLockMs = 700;
static constexpr uint32_t kOrbitCooldownMs = 1500;
static constexpr uint32_t kOrbitGapCompactDelayMs = 1000;
static constexpr float kOrbitGapReturnPxPerSecond = 8.0f;
static constexpr size_t kClusterMaxLabels = 8;
static constexpr size_t kClusterMaxCandidates = 20;
static constexpr uint8_t kClusterTriggerFrames = 3;
static constexpr uint32_t kClusterRetryMs = 500;
static constexpr size_t kClusterSearchNodeLimit = 12000;
static constexpr size_t kCollisionSearchesPerFrame = 8;
static constexpr uint8_t kHideAfterConflictFrames = 6;
static constexpr uint8_t kShowAfterCleanFrames = 20;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr float kDegreesToRadians = kPi / 180.0f;
static constexpr float kOrbitArrivalAngleRad = 1.0f * kDegreesToRadians;

static float clampFloat(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static bool isOwnAircraftObstacle(
    const AircraftObstacle &obstacle,
    float anchorX,
    float anchorY
) {
    float dx = obstacle.x - anchorX;
    float dy = obstacle.y - anchorY;
    return dx * dx + dy * dy <= 16.0f;
}

static float minFloat(float a, float b) {
    return a < b ? a : b;
}

static float maxFloat(float a, float b) {
    return a > b ? a : b;
}

static float rectOverlapDepth(
    float ax,
    float ay,
    float aw,
    float ah,
    float bx,
    float by,
    float bw,
    float bh
) {
    float overlapX = minFloat(ax + aw, bx + bw) - maxFloat(ax, bx);
    float overlapY = minFloat(ay + ah, by + bh) - maxFloat(ay, by);
    if (overlapX <= 0.0f || overlapY <= 0.0f) return 0.0f;
    return minFloat(overlapX, overlapY);
}

static float rectOverlapDepthWithMargin(
    float ax,
    float ay,
    float aw,
    float ah,
    float bx,
    float by,
    float bw,
    float bh,
    float margin
) {
    return rectOverlapDepth(
        ax - margin,
        ay - margin,
        aw + margin * 2.0f,
        ah + margin * 2.0f,
        bx,
        by,
        bw,
        bh
    );
}

static bool deadlineActive(uint32_t nowMs, uint32_t deadlineMs) {
    return static_cast<int32_t>(deadlineMs - nowMs) > 0;
}

static float pointToRectDistanceSquared(
    float px,
    float py,
    float x,
    float y,
    float width,
    float height
) {
    float nearestX = clampFloat(px, x, x + width);
    float nearestY = clampFloat(py, y, y + height);
    float dx = px - nearestX;
    float dy = py - nearestY;
    return dx * dx + dy * dy;
}

static float approximateLength(float x, float y) {
    float ax = fabsf(x);
    float ay = fabsf(y);
    float largest = maxFloat(ax, ay);
    float smallest = minFloat(ax, ay);
    return largest + smallest * 0.41421356f;
}

static float normalizeAngle(float angle) {
    while (angle > kPi) angle -= 2.0f * kPi;
    while (angle <= -kPi) angle += 2.0f * kPi;
    return angle;
}

static void clampToBounds(
    float &x,
    float &y,
    float width,
    float height,
    const LabelLayoutBounds &bounds
) {
    float maxX = maxFloat(bounds.left, bounds.right - width);
    float maxY = maxFloat(bounds.top, bounds.bottom - height);
    x = clampFloat(x, bounds.left, maxX);
    y = clampFloat(y, bounds.top, maxY);
}

static void positionAtDirection(
    const LabelLayoutInput &input,
    const LabelLayoutBounds &bounds,
    float directionX,
    float directionY,
    float symbolGap,
    float &x,
    float &y,
    float &clampDistance
) {
    float halfWidth = input.width * 0.5f;
    float halfHeight = input.height * 0.5f;
    float targetDistance = input.symbolRadius + symbolGap;
    float low = 0.0f;
    float high = targetDistance +
        sqrtf(halfWidth * halfWidth + halfHeight * halfHeight) + 2.0f;
    float targetSquared = targetDistance * targetDistance;
    for (size_t step = 0; step < 10; step++) {
        float distance = (low + high) * 0.5f;
        float candidateX = input.anchorX + directionX * distance - halfWidth;
        float candidateY = input.anchorY + directionY * distance - halfHeight;
        float actualSquared = pointToRectDistanceSquared(
            input.anchorX,
            input.anchorY,
            candidateX,
            candidateY,
            input.width,
            input.height
        );
        if (actualSquared < targetSquared) {
            low = distance;
        } else {
            high = distance;
        }
    }
    x = input.anchorX + directionX * high - halfWidth;
    y = input.anchorY + directionY * high - halfHeight;
    float unclampedX = x;
    float unclampedY = y;
    clampToBounds(x, y, input.width, input.height, bounds);
    clampDistance = fabsf(x - unclampedX) + fabsf(y - unclampedY);
}

static float symbolGapAtPosition(
    const LabelLayoutInput &input,
    float x,
    float y
) {
    float distanceSquared = pointToRectDistanceSquared(
        input.anchorX,
        input.anchorY,
        x,
        y,
        input.width,
        input.height
    );
    return maxFloat(
        kPreferredSymbolGapPx,
        sqrtf(distanceSquared) - input.symbolRadius
    );
}

static void courseVectors(
    const LabelLayoutInput &input,
    float &forwardX,
    float &forwardY,
    float &rightX,
    float &rightY
) {
    float radians = input.courseDeg * 0.01745329251994329577f;
    forwardX = sinf(radians);
    forwardY = -cosf(radians);
    rightX = cosf(radians);
    rightY = sinf(radians);
}

static bool isInsideForwardCone(
    const LabelLayoutInput &input,
    float centerX,
    float centerY,
    float forwardX,
    float forwardY
) {
    if (!input.courseValid) return false;
    float dx = centerX - input.anchorX;
    float dy = centerY - input.anchorY;
    float distanceSquared = dx * dx + dy * dy;
    if (distanceSquared < 0.000001f ||
        distanceSquared > kCourseAvoidDistancePx * kCourseAvoidDistancePx) {
        return false;
    }
    float forwardProjection = dx * forwardX + dy * forwardY;
    return forwardProjection > 0.0f &&
        forwardProjection * forwardProjection >=
            kCourseConeCosineSquared * distanceSquared;
}

static bool inputContainsId(
    const LabelLayoutInput *inputs,
    size_t inputCount,
    uint32_t id
) {
    for (size_t i = 0; i < inputCount; i++) {
        if (inputs[i].id == id) return true;
    }
    return false;
}

}  // namespace

void LabelLayout::reset() {
    memset(states_, 0, sizeof(states_));
    memset(work_, 0, sizeof(work_));
    orbitCursor_ = 0;
    collisionSearchCursor_ = 0;
}

void LabelLayout::solve(
    const LabelLayoutInput *inputs,
    size_t inputCount,
    const AircraftObstacle *aircraftObstacles,
    size_t aircraftObstacleCount,
    const LabelRectObstacle *staticObstacles,
    size_t staticObstacleCount,
    const LabelLayoutBounds &bounds,
    uint32_t nowMs,
    uint32_t layoutRevision,
    float deltaSeconds,
    LabelLayoutOutput *outputs,
    LabelLayoutMetrics *metrics
) {
    if (outputs == nullptr) return;
    if (inputCount > kMaxLabels) inputCount = kMaxLabels;
    if (aircraftObstacleCount > kMaxAircraftObstacles) {
        aircraftObstacleCount = kMaxAircraftObstacles;
    }
    if (staticObstacleCount > kMaxStaticObstacles) {
        staticObstacleCount = kMaxStaticObstacles;
    }

    for (size_t i = 0; i < inputCount; i++) {
        outputs[i] = LabelLayoutOutput();
    }
    if (metrics != nullptr) *metrics = LabelLayoutMetrics();
    if (inputs == nullptr || inputCount == 0) return;

    for (size_t i = 0; i < kMaxLabels; i++) {
        if (states_[i].occupied && nowMs - states_[i].lastSeenMs > kStateTtlMs) {
            states_[i] = State();
        }
    }

    size_t workCount = 0;
    for (size_t inputIndex = 0; inputIndex < inputCount; inputIndex++) {
        const LabelLayoutInput &input = inputs[inputIndex];
        if (input.id == 0 || input.width <= 0.0f || input.height <= 0.0f) continue;

        State *state = nullptr;
        size_t hashIndex = static_cast<size_t>(input.id % kMaxLabels);
        for (size_t probe = 0; probe < kMaxLabels; probe++) {
            size_t stateIndex = (hashIndex + probe) % kMaxLabels;
            if (states_[stateIndex].occupied && states_[stateIndex].id == input.id) {
                state = &states_[stateIndex];
                break;
            }
        }

        bool isNew = false;
        if (state == nullptr) {
            size_t replacement = kMaxLabels;
            uint32_t oldestAge = 0;
            for (size_t probe = 0; probe < kMaxLabels; probe++) {
                size_t stateIndex = (hashIndex + probe) % kMaxLabels;
                if (!states_[stateIndex].occupied) {
                    replacement = stateIndex;
                    break;
                }
                if (inputContainsId(inputs, inputCount, states_[stateIndex].id)) continue;
                uint32_t age = nowMs - states_[stateIndex].lastSeenMs;
                if (replacement == kMaxLabels || age > oldestAge) {
                    replacement = stateIndex;
                    oldestAge = age;
                }
            }
            if (replacement == kMaxLabels) continue;
            states_[replacement] = State();
            state = &states_[replacement];
            state->occupied = true;
            state->id = input.id;
            state->visible = true;
            isNew = true;
        }

        WorkItem &work = work_[workCount++];
        work = WorkItem();
        work.input = &input;
        work.state = state;
        work.outputIndex = inputIndex;
        work.isNew = isNew;
        state->lastSeenMs = nowMs;
    }

    // Stable ICAO ordering keeps initialization and pairwise relaxation independent
    // from the distance sort used by the renderer.
    for (size_t i = 1; i < workCount; i++) {
        WorkItem value = work_[i];
        size_t j = i;
        while (j > 0 && work_[j - 1].input->id > value.input->id) {
            work_[j] = work_[j - 1];
            j--;
        }
        work_[j] = value;
    }

    bool hasNewLabel = false;
    for (size_t i = 0; i < workCount; i++) {
        WorkItem &work = work_[i];
        State &state = *work.state;
        const LabelLayoutInput &input = *work.input;
        courseVectors(
            input,
            work.forwardX,
            work.forwardY,
            work.rightX,
            work.rightY
        );

        float anchorDx = input.anchorX - state.anchorX;
        float anchorDy = input.anchorY - state.anchorY;
        float anchorMovementSquared = anchorDx * anchorDx + anchorDy * anchorDy;
        bool reinitialize = work.isNew ||
            state.layoutRevision != layoutRevision ||
            anchorMovementSquared > kAnchorJumpPx * kAnchorJumpPx;

        if (!reinitialize) {
            float centerX = state.x + state.width * 0.5f + anchorDx;
            float centerY = state.y + state.height * 0.5f + anchorDy;
            work.x = centerX - input.width * 0.5f;
            work.y = centerY - input.height * 0.5f;
            clampToBounds(work.x, work.y, input.width, input.height, bounds);
        } else {
            work.isNew = true;
            hasNewLabel = true;
            state.orbitTargetActive = false;
            state.orbitTargetAngle = 0.0f;
            state.orbitTargetGap = kPreferredSymbolGapPx;
            state.orbitDirection = 0;
            state.orbitLockUntilMs = 0;
            state.orbitCooldownUntilMs = 0;
            state.orbitGapCompactAfterMs = 0;
            state.clusterRetryAfterMs = 0;
            state.clusterConflictFrames = 0;
            state.orbitAngleValid = false;

            const float directions[8][2] = {
                {work.rightX, work.rightY},
                {-work.rightX, -work.rightY},
                {
                    (work.rightX - work.forwardX) * kInverseSqrtTwo,
                    (work.rightY - work.forwardY) * kInverseSqrtTwo
                },
                {
                    (-work.rightX - work.forwardX) * kInverseSqrtTwo,
                    (-work.rightY - work.forwardY) * kInverseSqrtTwo
                },
                {-work.forwardX, -work.forwardY},
                {
                    (work.rightX + work.forwardX) * kInverseSqrtTwo,
                    (work.rightY + work.forwardY) * kInverseSqrtTwo
                },
                {
                    (-work.rightX + work.forwardX) * kInverseSqrtTwo,
                    (-work.rightY + work.forwardY) * kInverseSqrtTwo
                },
                {work.forwardX, work.forwardY},
            };

            float bestScore = 1.0e30f;
            float bestX = input.anchorX;
            float bestY = input.anchorY;
            for (size_t candidateIndex = 0; candidateIndex < 8; candidateIndex++) {
                float directionX = directions[candidateIndex][0];
                float directionY = directions[candidateIndex][1];

                float halfWidth = input.width * 0.5f;
                float halfHeight = input.height * 0.5f;
                float extent = fabsf(directionX) * halfWidth +
                    fabsf(directionY) * halfHeight;
                float centerDistance = input.symbolRadius +
                    kPreferredSymbolGapPx + extent;
                float candidateX = input.anchorX + directionX * centerDistance - halfWidth;
                float candidateY = input.anchorY + directionY * centerDistance - halfHeight;
                float unclampedX = candidateX;
                float unclampedY = candidateY;
                clampToBounds(candidateX, candidateY, input.width, input.height, bounds);

                float score = static_cast<float>(candidateIndex) * 0.01f;
                score += (fabsf(candidateX - unclampedX) + fabsf(candidateY - unclampedY)) * 80.0f;
                if (isInsideForwardCone(
                        input,
                        candidateX + halfWidth,
                        candidateY + halfHeight,
                        work.forwardX,
                        work.forwardY)) {
                    score += 300.0f;
                }
                for (size_t obstacleIndex = 0;
                     obstacleIndex < aircraftObstacleCount;
                     obstacleIndex++) {
                    const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
                    if (!isOwnAircraftObstacle(obstacle, input.anchorX, input.anchorY)) {
                        continue;
                    }
                    float nearestX = clampFloat(
                        obstacle.x,
                        candidateX,
                        candidateX + input.width
                    );
                    float nearestY = clampFloat(
                        obstacle.y,
                        candidateY,
                        candidateY + input.height
                    );
                    float dx = obstacle.x - nearestX;
                    float dy = obstacle.y - nearestY;
                    float distanceSquared = dx * dx + dy * dy;
                    float required = obstacle.radius + kMinimumSymbolGapPx;
                    float requiredSquared = required * required;
                    if (distanceSquared < requiredSquared) {
                        score += (requiredSquared - distanceSquared) *
                            (60.0f / required);
                    }
                }
                for (size_t obstacleIndex = 0;
                     obstacleIndex < staticObstacleCount;
                     obstacleIndex++) {
                    const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
                    score += rectOverlapDepth(
                        candidateX,
                        candidateY,
                        input.width,
                        input.height,
                        obstacle.x,
                        obstacle.y,
                        obstacle.width,
                        obstacle.height
                    ) * 80.0f;
                }
                for (size_t previous = 0; previous < i; previous++) {
                    score += rectOverlapDepth(
                        candidateX,
                        candidateY,
                        input.width,
                        input.height,
                        work_[previous].x,
                        work_[previous].y,
                        work_[previous].input->width,
                        work_[previous].input->height
                    ) * 40.0f;
                }
                if (score < bestScore) {
                    bestScore = score;
                    bestX = candidateX;
                    bestY = candidateY;
                }
            }
            work.x = bestX;
            work.y = bestY;
            float radialX = work.x + input.width * 0.5f - input.anchorX;
            float radialY = work.y + input.height * 0.5f - input.anchorY;
            state.orbitTargetAngle = atan2f(radialY, radialX);
            state.orbitTargetGap = symbolGapAtPosition(input, work.x, work.y);
            state.orbitAngleValid = true;
        }

        work.baseX = work.x;
        work.baseY = work.y;
        state.anchorX = input.anchorX;
        state.anchorY = input.anchorY;
        state.width = input.width;
        state.height = input.height;
        state.layoutRevision = layoutRevision;
    }

    auto placementHasHardConflict = [&](const WorkItem &work, float x, float y) {
        const LabelLayoutInput &input = *work.input;
        if (isInsideForwardCone(
                input,
                x + input.width * 0.5f,
                y + input.height * 0.5f,
                work.forwardX,
                work.forwardY)) {
            return true;
        }
        for (size_t obstacleIndex = 0;
             obstacleIndex < aircraftObstacleCount;
             obstacleIndex++) {
            const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
            if (!isOwnAircraftObstacle(obstacle, input.anchorX, input.anchorY)) {
                continue;
            }
            float required = obstacle.radius + kMinimumSymbolGapPx;
            if (pointToRectDistanceSquared(
                    obstacle.x,
                    obstacle.y,
                    x,
                    y,
                    input.width,
                    input.height) < required * required) {
                return true;
            }
        }
        for (size_t obstacleIndex = 0;
             obstacleIndex < staticObstacleCount;
             obstacleIndex++) {
            const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
            if (rectOverlapDepthWithMargin(
                    x,
                    y,
                    input.width,
                    input.height,
                    obstacle.x,
                    obstacle.y,
                    obstacle.width,
                    obstacle.height,
                    1.0f) > 0.0f) {
                return true;
            }
        }
        return false;
    };

    for (size_t i = 0; i < workCount; i++) {
        WorkItem &work = work_[i];
        State &state = *work.state;
        if (!state.orbitAngleValid) continue;
        float targetX = 0.0f;
        float targetY = 0.0f;
        float clampDistance = 0.0f;
        bool canCompact = !state.orbitTargetActive &&
            !deadlineActive(nowMs, state.orbitCooldownUntilMs) &&
            !deadlineActive(nowMs, state.orbitGapCompactAfterMs) &&
            state.orbitTargetGap > kPreferredSymbolGapPx;
        if (canCompact) {
            float preferredX = 0.0f;
            float preferredY = 0.0f;
            positionAtDirection(
                *work.input,
                bounds,
                cosf(state.orbitTargetAngle),
                sinf(state.orbitTargetAngle),
                kPreferredSymbolGapPx,
                preferredX,
                preferredY,
                clampDistance
            );
            if (!placementHasHardConflict(work, preferredX, preferredY)) {
                float compactDelta = kOrbitGapReturnPxPerSecond *
                    clampFloat(deltaSeconds, 0.0f, 0.1f);
                state.orbitTargetGap = maxFloat(
                    kPreferredSymbolGapPx,
                    state.orbitTargetGap - compactDelta
                );
            }
        }
        positionAtDirection(
            *work.input,
            bounds,
            cosf(state.orbitTargetAngle),
            sinf(state.orbitTargetAngle),
            state.orbitTargetGap,
            targetX,
            targetY,
            clampDistance
        );
        if (placementHasHardConflict(work, targetX, targetY)) {
            state.orbitTargetActive = false;
            state.orbitAngleValid = false;
            state.orbitCooldownUntilMs = 0;
            state.orbitGapCompactAfterMs = 0;
        }
    }

    auto updateAircraftForce = [&](WorkItem &work) {
        const LabelLayoutInput &input = *work.input;
        work.aircraftForceX = 0;
        work.aircraftForceY = 0;
        for (size_t obstacleIndex = 0;
             obstacleIndex < aircraftObstacleCount;
             obstacleIndex++) {
            const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
            if (!isOwnAircraftObstacle(obstacle, input.anchorX, input.anchorY)) {
                continue;
            }
            float required = obstacle.radius + kMinimumSymbolGapPx;
            bool insideX = obstacle.x >= work.x &&
                obstacle.x <= work.x + input.width;
            bool insideY = obstacle.y >= work.y &&
                obstacle.y <= work.y + input.height;
            if (insideX && insideY) {
                float leftDistance = obstacle.x - work.x;
                float rightDistance = work.x + input.width - obstacle.x;
                float topDistance = obstacle.y - work.y;
                float bottomDistance = work.y + input.height - obstacle.y;
                if (leftDistance <= rightDistance &&
                    leftDistance <= topDistance &&
                    leftDistance <= bottomDistance) {
                    work.aircraftForceX -= (leftDistance + required) * 0.7f;
                } else if (rightDistance <= topDistance &&
                           rightDistance <= bottomDistance) {
                    work.aircraftForceX += (rightDistance + required) * 0.7f;
                } else if (topDistance <= bottomDistance) {
                    work.aircraftForceY -= (topDistance + required) * 0.7f;
                } else {
                    work.aircraftForceY += (bottomDistance + required) * 0.7f;
                }
                continue;
            }

            float nearestX = clampFloat(obstacle.x, work.x, work.x + input.width);
            float nearestY = clampFloat(obstacle.y, work.y, work.y + input.height);
            float pushX = nearestX - obstacle.x;
            float pushY = nearestY - obstacle.y;
            float distanceSquared = pushX * pushX + pushY * pushY;
            if (distanceSquared >= required * required) continue;
            float distance = sqrtf(distanceSquared);
            if (distance < 0.001f) continue;
            float strength = (required - distance) * 0.7f;
            work.aircraftForceX += (pushX / distance) * strength;
            work.aircraftForceY += (pushY / distance) * strength;
        }
    };

    const size_t iterations = hasNewLabel ? 12 : 3;
    size_t orbitStart = workCount > 0 ? orbitCursor_ % workCount : 0;
    for (size_t iteration = 0; iteration < iterations; iteration++) {
        // Aircraft move much more slowly than labels. Reusing the aggregate force
        // for three relaxation steps preserves all obstacles while avoiding a
        // 64x64 scan on every sub-step.
        if (iteration % 3 == 0) {
            for (size_t i = 0; i < workCount; i++) {
                if (work_[i].state->orbitAngleValid) {
                    work_[i].aircraftForceX = 0.0f;
                    work_[i].aircraftForceY = 0.0f;
                } else {
                    updateAircraftForce(work_[i]);
                }
            }
        }
        for (size_t i = 0; i < workCount; i++) {
            WorkItem &work = work_[i];
            work.forceX = work.aircraftForceX;
            work.forceY = work.aircraftForceY;
            const LabelLayoutInput &input = *work.input;
            if (work.state->orbitAngleValid) {
                work.forceX = 0.0f;
                work.forceY = 0.0f;
                continue;
            }

            float centerX = work.x + input.width * 0.5f;
            float centerY = work.y + input.height * 0.5f;
            float fromAnchorX = centerX - input.anchorX;
            float fromAnchorY = centerY - input.anchorY;
            float nearestX = clampFloat(input.anchorX, work.x, work.x + input.width);
            float nearestY = clampFloat(input.anchorY, work.y, work.y + input.height);
            float edgeX = nearestX - input.anchorX;
            float edgeY = nearestY - input.anchorY;
            float edgeDistance = approximateLength(edgeX, edgeY);
            float directionX = edgeX;
            float directionY = edgeY;
            float directionDistance = edgeDistance;
            float springForceX = 0.0f;
            float springForceY = 0.0f;
            if (directionDistance < 0.001f) {
                directionX = fromAnchorX;
                directionY = fromAnchorY;
                directionDistance = sqrtf(
                    directionX * directionX + directionY * directionY
                );
            }
            if (directionDistance > 0.001f) {
                float unitX = directionX / directionDistance;
                float unitY = directionY / directionDistance;
                float targetDistance = input.symbolRadius + kPreferredSymbolGapPx;
                float spring = (targetDistance - edgeDistance) * 0.28f;
                springForceX = unitX * spring;
                springForceY = unitY * spring;
                work.forceX += springForceX;
                work.forceY += springForceY;
            }

            if (isInsideForwardCone(
                    input,
                    centerX,
                    centerY,
                    work.forwardX,
                    work.forwardY)) {
                float side = fromAnchorX * work.rightX +
                    fromAnchorY * work.rightY;
                float sideSign = fabsf(side) > 0.01f
                    ? (side > 0.0f ? 1.0f : -1.0f)
                    : ((input.id & 1U) ? 1.0f : -1.0f);
                work.forceX += work.rightX * sideSign * 1.5f -
                    work.forwardX * 0.45f;
                work.forceY += work.rightY * sideSign * 1.5f -
                    work.forwardY * 0.45f;
            }

            for (size_t obstacleIndex = 0;
                 obstacleIndex < staticObstacleCount;
                 obstacleIndex++) {
                const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
                float overlapX = minFloat(work.x + input.width, obstacle.x + obstacle.width) -
                    maxFloat(work.x, obstacle.x);
                float overlapY = minFloat(work.y + input.height, obstacle.y + obstacle.height) -
                    maxFloat(work.y, obstacle.y);
                if (overlapX <= 0.0f || overlapY <= 0.0f) continue;
                float obstacleCenterX = obstacle.x + obstacle.width * 0.5f;
                float obstacleCenterY = obstacle.y + obstacle.height * 0.5f;
                if (overlapX < overlapY) {
                    work.forceX += centerX < obstacleCenterX
                        ? -(overlapX + 1.0f)
                        : overlapX + 1.0f;
                } else {
                    work.forceY += centerY < obstacleCenterY
                        ? -(overlapY + 1.0f)
                        : overlapY + 1.0f;
                }
            }

            float targetDistance = input.symbolRadius + kPreferredSymbolGapPx;
            float excessDistance = edgeDistance - targetDistance;
            if (iteration % 3 == 0 &&
                excessDistance > kOrbitMinExcessPx &&
                (fromAnchorX != 0.0f || fromAnchorY != 0.0f)) {
                size_t orbitOffset = i >= orbitStart
                    ? i - orbitStart
                    : i + workCount - orbitStart;
                bool orbitScheduled = workCount <= kOrbitLabelsPerFrame ||
                    orbitOffset < kOrbitLabelsPerFrame || input.mustShow;
                if (!orbitScheduled) continue;
                float centerDistance = approximateLength(fromAnchorX, fromAnchorY);
                float radialX = fromAnchorX / centerDistance;
                float radialY = fromAnchorY / centerDistance;
                float avoidanceX = work.forceX - springForceX;
                float avoidanceY = work.forceY - springForceY;
                float outwardAvoidance = avoidanceX * radialX + avoidanceY * radialY;
                if (outwardAvoidance > 0.1f) {
                    float tangentX = -radialY;
                    float tangentY = radialX;
                    float tangentAvoidance = avoidanceX * tangentX +
                        avoidanceY * tangentY;
                    State &state = *work.state;
                    bool directionLocked = state.orbitDirection != 0 &&
                        deadlineActive(nowMs, state.orbitLockUntilMs);
                    float orbitSign = directionLocked
                        ? static_cast<float>(state.orbitDirection)
                        : (fabsf(tangentAvoidance) > outwardAvoidance * 0.2f + 0.1f
                            ? (tangentAvoidance > 0.0f ? 1.0f : -1.0f)
                            : ((input.id & 1U) ? 1.0f : -1.0f));
                    if (!directionLocked) {
                        state.orbitDirection = orbitSign > 0.0f ? 1 : -1;
                        state.orbitLockUntilMs = nowMs + kOrbitDirectionLockMs;
                    }
                    float orbitStrength = minFloat(
                        kOrbitForceLimitPx,
                        0.35f + excessDistance * 0.08f + outwardAvoidance * 0.15f
                    );
                    work.forceX += tangentX * orbitSign * orbitStrength;
                    work.forceY += tangentY * orbitSign * orbitStrength;
                }
            }
        }

        for (size_t i = 0; i < workCount; i++) {
            WorkItem &work = work_[i];
            float forceLength = approximateLength(work.forceX, work.forceY);
            if (forceLength > 6.0f) {
                work.forceX *= 6.0f / forceLength;
                work.forceY *= 6.0f / forceLength;
            }
            work.x += work.forceX;
            work.y += work.forceY;
            clampToBounds(
                work.x,
                work.y,
                work.input->width,
                work.input->height,
                bounds
            );

            float nearestX = clampFloat(
                work.input->anchorX,
                work.x,
                work.x + work.input->width
            );
            float nearestY = clampFloat(
                work.input->anchorY,
                work.y,
                work.y + work.input->height
            );
            float toAnchorX = work.input->anchorX - nearestX;
            float toAnchorY = work.input->anchorY - nearestY;
            float gapSquared = toAnchorX * toAnchorX + toAnchorY * toAnchorY;
            float maxGap = work.input->mustShow
                ? kPriorityMaxGapPx
                : kNormalMaxGapPx;
            if (gapSquared > maxGap * maxGap) {
                float gap = sqrtf(gapSquared);
                float correction = gap - maxGap;
                work.x += toAnchorX * correction / gap;
                work.y += toAnchorY * correction / gap;
                clampToBounds(
                    work.x,
                    work.y,
                    work.input->width,
                    work.input->height,
                    bounds
                );
            }
        }
    }

    if (workCount > kOrbitLabelsPerFrame) {
        orbitCursor_ = (orbitStart + kOrbitLabelsPerFrame) % workCount;
    } else {
        orbitCursor_ = 0;
    }

    deltaSeconds = clampFloat(deltaSeconds, 0.0f, 0.1f);
    float maxMovement = kMaxMovementPxPerSecond * deltaSeconds;
    auto setLimitedPosition = [&](WorkItem &work, float targetX, float targetY) {
        work.x = targetX;
        work.y = targetY;
        if (!work.isNew) {
            float dx = work.x - work.baseX;
            float dy = work.y - work.baseY;
            float distanceSquared = dx * dx + dy * dy;
            if (maxMovement <= 0.0f ||
                distanceSquared <= kMovementDeadZonePx * kMovementDeadZonePx) {
                work.x = work.baseX;
                work.y = work.baseY;
            } else if (distanceSquared > maxMovement * maxMovement) {
                float distance = approximateLength(dx, dy);
                work.x = work.baseX + dx * maxMovement / distance;
                work.y = work.baseY + dy * maxMovement / distance;
            }
        }
        clampToBounds(
            work.x,
            work.y,
            work.input->width,
            work.input->height,
            bounds
        );
    };

    for (size_t i = 0; i < workCount; i++) {
        setLimitedPosition(work_[i], work_[i].x, work_[i].y);
    }

    size_t collisionOrder[kMaxLabels];
    for (size_t i = 0; i < workCount; i++) collisionOrder[i] = i;
    for (size_t i = 1; i < workCount; i++) {
        size_t value = collisionOrder[i];
        size_t j = i;
        while (j > 0) {
            const LabelLayoutInput &left = *work_[collisionOrder[j - 1]].input;
            const LabelLayoutInput &right = *work_[value].input;
            bool rightBeforeLeft = right.mustShow != left.mustShow
                ? right.mustShow
                : right.id < left.id;
            if (!rightBeforeLeft) break;
            collisionOrder[j] = collisionOrder[j - 1];
            j--;
        }
        collisionOrder[j] = value;
    }

    struct OrbitRotation {
        float cosine;
        float sine;
        float angle;
        float turnPenalty;
        int8_t direction;
    };
    static constexpr OrbitRotation kOrbitRotations[] = {
        {1.0f, 0.0f, 0.0f, 0.0f, 0},
        {0.86602540f, 0.5f, 30.0f * kDegreesToRadians, 0.9f, 1},
        {0.86602540f, -0.5f, -30.0f * kDegreesToRadians, 0.9f, -1},
        {0.5f, 0.86602540f, 60.0f * kDegreesToRadians, 1.8f, 1},
        {0.5f, -0.86602540f, -60.0f * kDegreesToRadians, 1.8f, -1},
        {0.0f, 1.0f, 90.0f * kDegreesToRadians, 2.7f, 1},
        {0.0f, -1.0f, -90.0f * kDegreesToRadians, 2.7f, -1},
        {-0.5f, 0.86602540f, 120.0f * kDegreesToRadians, 3.6f, 1},
        {-0.5f, -0.86602540f, -120.0f * kDegreesToRadians, 3.6f, -1},
        {-0.86602540f, 0.5f, 150.0f * kDegreesToRadians, 4.5f, 1},
        {-0.86602540f, -0.5f, -150.0f * kDegreesToRadians, 4.5f, -1},
        {-1.0f, 0.0f, kPi, 5.4f, 0},
    };

    bool reserved[kMaxLabels] = {};
    float reservedX[kMaxLabels] = {};
    float reservedY[kMaxLabels] = {};
    size_t collisionSearchStart = workCount > 0
        ? collisionSearchCursor_ % workCount
        : 0;
    auto labelConflictDepth = [&](size_t workIndex, float x, float y) {
        const LabelLayoutInput &input = *work_[workIndex].input;
        float deepest = 0.0f;
        for (size_t otherIndex = 0; otherIndex < workCount; otherIndex++) {
            if (!reserved[otherIndex]) continue;
            const WorkItem &other = work_[otherIndex];
            float overlap = rectOverlapDepthWithMargin(
                x,
                y,
                input.width,
                input.height,
                reservedX[otherIndex],
                reservedY[otherIndex],
                other.input->width,
                other.input->height,
                kLabelCollisionMarginPx
            );
            if (overlap > deepest) deepest = overlap;
        }
        return deepest;
    };

    auto placementScore = [&](size_t workIndex,
                              float x,
                              float y,
                              float clampDistance,
                              float turnPenalty,
                              int8_t direction) {
        WorkItem &work = work_[workIndex];
        const LabelLayoutInput &input = *work.input;
        float score = turnPenalty + clampDistance * 60.0f;
        if (direction != 0 &&
            work.state->orbitDirection != 0 &&
            direction != work.state->orbitDirection &&
            deadlineActive(nowMs, work.state->orbitLockUntilMs)) {
            score += 120.0f;
        }

        if (isInsideForwardCone(
                input,
                x + input.width * 0.5f,
                y + input.height * 0.5f,
                work.forwardX,
                work.forwardY)) {
            score += 35.0f;
        }

        for (size_t obstacleIndex = 0;
             obstacleIndex < aircraftObstacleCount;
             obstacleIndex++) {
            const AircraftObstacle &obstacle = aircraftObstacles[obstacleIndex];
            if (!isOwnAircraftObstacle(obstacle, input.anchorX, input.anchorY)) {
                continue;
            }
            float required = obstacle.radius + kMinimumSymbolGapPx;
            float requiredSquared = required * required;
            float distanceSquared = pointToRectDistanceSquared(
                obstacle.x,
                obstacle.y,
                x,
                y,
                input.width,
                input.height
            );
            if (distanceSquared < requiredSquared) {
                score += 240.0f + (requiredSquared - distanceSquared) * 1.2f;
            }
        }

        for (size_t obstacleIndex = 0;
             obstacleIndex < staticObstacleCount;
             obstacleIndex++) {
            const LabelRectObstacle &obstacle = staticObstacles[obstacleIndex];
            float overlap = rectOverlapDepthWithMargin(
                x,
                y,
                input.width,
                input.height,
                obstacle.x,
                obstacle.y,
                obstacle.width,
                obstacle.height,
                1.0f
            );
            if (overlap > 0.0f) score += 220.0f + overlap * overlap * 12.0f;
        }

        for (size_t otherIndex = 0; otherIndex < workCount; otherIndex++) {
            if (!reserved[otherIndex]) continue;
            const WorkItem &other = work_[otherIndex];
            float overlap = rectOverlapDepthWithMargin(
                x,
                y,
                input.width,
                input.height,
                reservedX[otherIndex],
                reservedY[otherIndex],
                other.input->width,
                other.input->height,
                kLabelCollisionMarginPx
            );
            if (overlap > 0.0f) score += 300.0f + overlap * overlap * 18.0f;
        }
        return score;
    };

    auto advanceOrbitTarget = [&](size_t workIndex) {
        WorkItem &work = work_[workIndex];
        State &state = *work.state;
        if (!state.orbitAngleValid) return;

        bool moving = state.orbitTargetActive;
        bool coolingDown = !moving &&
            deadlineActive(nowMs, state.orbitCooldownUntilMs);
        work.orbiting = moving;
        work.coolingDown = coolingDown;
        const LabelLayoutInput &input = *work.input;
        float centerX = work.x + input.width * 0.5f;
        float centerY = work.y + input.height * 0.5f;
        float radialX = centerX - input.anchorX;
        float radialY = centerY - input.anchorY;
        float centerDistance = sqrtf(radialX * radialX + radialY * radialY);
        float currentAngle = centerDistance > 0.001f
            ? atan2f(radialY, radialX)
            : state.orbitTargetAngle;
        float remaining = normalizeAngle(state.orbitTargetAngle - currentAngle);
        if (fabsf(fabsf(remaining) - kPi) < 0.001f && state.orbitDirection != 0) {
            remaining = kPi * static_cast<float>(state.orbitDirection);
        }

        float nextAngle = state.orbitTargetAngle;
        if (moving) {
            float maxAngleStep = centerDistance > 1.0f
                ? maxMovement / centerDistance
                : fabsf(remaining);
            float angleStep = clampFloat(remaining, -maxAngleStep, maxAngleStep);
            nextAngle = currentAngle + angleStep;
        }
        float candidateX = 0.0f;
        float candidateY = 0.0f;
        float clampDistance = 0.0f;
        positionAtDirection(
            input,
            bounds,
            cosf(nextAngle),
            sinf(nextAngle),
            state.orbitTargetGap,
            candidateX,
            candidateY,
            clampDistance
        );
        setLimitedPosition(work, candidateX, candidateY);

        float targetX = 0.0f;
        float targetY = 0.0f;
        positionAtDirection(
            input,
            bounds,
            cosf(state.orbitTargetAngle),
            sinf(state.orbitTargetAngle),
            state.orbitTargetGap,
            targetX,
            targetY,
            clampDistance
        );
        centerX = work.x + input.width * 0.5f;
        centerY = work.y + input.height * 0.5f;
        float actualAngle = atan2f(
            centerY - input.anchorY,
            centerX - input.anchorX
        );
        float angleError = fabsf(normalizeAngle(
            state.orbitTargetAngle - actualAngle
        ));
        float positionError = approximateLength(work.x - targetX, work.y - targetY);
        if (moving &&
            angleError <= kOrbitArrivalAngleRad &&
            positionError <= 0.75f) {
            state.orbitTargetActive = false;
            state.orbitCooldownUntilMs = nowMs + kOrbitCooldownMs;
            work.coolingDown = true;
        }
        if (state.orbitDirection != 0) {
            state.orbitLockUntilMs = nowMs + kOrbitDirectionLockMs;
        }
    };

    for (size_t workIndex = 0; workIndex < workCount; workIndex++) {
        WorkItem &work = work_[workIndex];
        const LabelLayoutInput &input = *work.input;
        if (!work.state->orbitAngleValid &&
            !placementHasHardConflict(work, work.x, work.y)) {
            float radialX = work.x + input.width * 0.5f - input.anchorX;
            float radialY = work.y + input.height * 0.5f - input.anchorY;
            float settledAngle = atan2f(radialY, radialX);
            float maxGap = input.mustShow ? kPriorityMaxGapPx : kNormalMaxGapPx;
            float settledGap = clampFloat(
                symbolGapAtPosition(input, work.x, work.y),
                kPreferredSymbolGapPx,
                maxGap
            );
            float settledX = 0.0f;
            float settledY = 0.0f;
            float clampDistance = 0.0f;
            positionAtDirection(
                input,
                bounds,
                cosf(settledAngle),
                sinf(settledAngle),
                settledGap,
                settledX,
                settledY,
                clampDistance
            );
            if (!placementHasHardConflict(work, settledX, settledY)) {
                work.state->orbitTargetAngle = settledAngle;
                work.state->orbitTargetGap = settledGap;
                work.state->orbitGapCompactAfterMs =
                    nowMs + kOrbitGapCompactDelayMs;
                work.state->orbitAngleValid = true;
            }
        }
        advanceOrbitTarget(workIndex);
    }

    // Resolve a connected overlap as one assignment so an earlier reservation
    // can move when that creates a better plan for the whole local cluster.
    bool clusterPending[kMaxLabels] = {};
    bool clusterVisited[kMaxLabels] = {};
    bool componentMask[kMaxLabels] = {};
    float plannedX[kMaxLabels] = {};
    float plannedY[kMaxLabels] = {};
    for (size_t i = 0; i < workCount; i++) {
        WorkItem &work = work_[i];
        State &state = *work.state;
        if (state.orbitAngleValid) {
            float clampDistance = 0.0f;
            positionAtDirection(
                *work.input,
                bounds,
                cosf(state.orbitTargetAngle),
                sinf(state.orbitTargetAngle),
                state.orbitTargetGap,
                plannedX[i],
                plannedY[i],
                clampDistance
            );
        } else {
            plannedX[i] = work.x;
            plannedY[i] = work.y;
        }
        if (work.orbiting || work.coolingDown ||
            !(state.visible || work.input->mustShow)) {
            state.clusterConflictFrames = 0;
        }
    }

    auto canJoinCluster = [&](size_t workIndex) {
        if (!kResolveOverlappingLabels) {
            return false;
        }
        const WorkItem &work = work_[workIndex];
        return (work.state->visible || work.input->mustShow) &&
            work.state->orbitAngleValid &&
            !work.orbiting &&
            !work.coolingDown;
    };

    auto plannedLabelsOverlap = [&](size_t leftIndex, size_t rightIndex) {
        const WorkItem &left = work_[leftIndex];
        const WorkItem &right = work_[rightIndex];
        return rectOverlapDepthWithMargin(
            plannedX[leftIndex],
            plannedY[leftIndex],
            left.input->width,
            left.input->height,
            plannedX[rightIndex],
            plannedY[rightIndex],
            right.input->width,
            right.input->height,
            kLabelCollisionMarginPx
        ) > 0.0f;
    };

    for (size_t seed = 0; seed < workCount; seed++) {
        if (clusterVisited[seed] || !canJoinCluster(seed)) continue;

        size_t *queue = clusterQueue_;
        size_t *component = clusterComponent_;
        size_t queueHead = 0;
        size_t queueCount = 1;
        size_t componentCount = 0;
        queue[0] = seed;
        clusterVisited[seed] = true;
        while (queueHead < queueCount) {
            size_t current = queue[queueHead++];
            component[componentCount++] = current;
            for (size_t candidate = 0; candidate < workCount; candidate++) {
                if (clusterVisited[candidate] || !canJoinCluster(candidate)) continue;
                if (!plannedLabelsOverlap(current, candidate)) continue;
                clusterVisited[candidate] = true;
                queue[queueCount++] = candidate;
            }
        }

        if (componentCount < 2) {
            work_[seed].state->clusterConflictFrames = 0;
            continue;
        }

        bool ready = componentCount <= kClusterMaxLabels;
        bool retryReady = true;
        for (size_t member = 0; member < componentCount; member++) {
            size_t workIndex = component[member];
            clusterPending[workIndex] = true;
            State &state = *work_[workIndex].state;
            if (state.clusterConflictFrames < 255) {
                state.clusterConflictFrames++;
            }
            if (state.clusterConflictFrames < kClusterTriggerFrames) ready = false;
            if (deadlineActive(nowMs, state.clusterRetryAfterMs)) retryReady = false;
        }
        if (!ready || !retryReady) continue;

        memset(componentMask, 0, sizeof(componentMask));
        for (size_t member = 0; member < componentCount; member++) {
            componentMask[component[member]] = true;
        }

        uint8_t candidateCounts[kClusterMaxLabels] = {};
        static constexpr float kInnerOffsetsDeg[] = {
            0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180
        };
        static constexpr float kOuterOffsetsDeg[] = {
            0, 45, -45, 90, -90, 135, -135, 180
        };

        bool candidatesComplete = true;
        for (size_t member = 0; member < componentCount; member++) {
            size_t workIndex = component[member];
            WorkItem &work = work_[workIndex];
            State &state = *work.state;
            const LabelLayoutInput &input = *work.input;
            float baseAngle = state.orbitTargetAngle;
            float baseGap = maxFloat(kPreferredSymbolGapPx, state.orbitTargetGap);
            float maxGap = input.mustShow ? kPriorityMaxGapPx : kNormalMaxGapPx;
            float outerGap = minFloat(maxGap, maxFloat(16.0f, baseGap + 12.0f));

            auto addCandidate = [&](float offsetDeg, float gap, float ringPenalty) {
                if (candidateCounts[member] >= kClusterMaxCandidates) return;
                float offset = offsetDeg * kDegreesToRadians;
                float angle = normalizeAngle(baseAngle + offset);
                float x = 0.0f;
                float y = 0.0f;
                float clampDistance = 0.0f;
                positionAtDirection(
                    input,
                    bounds,
                    cosf(angle),
                    sinf(angle),
                    gap,
                    x,
                    y,
                    clampDistance
                );
                if (placementHasHardConflict(work, x, y)) return;
                for (size_t otherIndex = 0; otherIndex < workCount; otherIndex++) {
                    if (componentMask[otherIndex] ||
                        !(work_[otherIndex].state->visible ||
                          work_[otherIndex].input->mustShow)) {
                        continue;
                    }
                    const WorkItem &other = work_[otherIndex];
                    if (rectOverlapDepthWithMargin(
                            x,
                            y,
                            input.width,
                            input.height,
                            plannedX[otherIndex],
                            plannedY[otherIndex],
                            other.input->width,
                            other.input->height,
                            kLabelCollisionMarginPx) > 0.0f) {
                        return;
                    }
                }
                for (size_t existing = 0;
                     existing < candidateCounts[member];
                     existing++) {
                    if (fabsf(clusterCandidates_[member][existing].x - x) < 0.25f &&
                        fabsf(clusterCandidates_[member][existing].y - y) < 0.25f) {
                        return;
                    }
                }

                int8_t direction = offsetDeg > 0.0f ? 1 :
                    (offsetDeg < 0.0f ? -1 : 0);
                float directionPenalty = direction != 0 &&
                    state.orbitDirection != 0 &&
                    direction != state.orbitDirection &&
                    deadlineActive(nowMs, state.orbitLockUntilMs)
                    ? 8.0f
                    : 0.0f;
                ClusterCandidateScratch value{
                    x,
                    y,
                    angle,
                    gap,
                    fabsf(offsetDeg) * 0.04f +
                        fabsf(gap - baseGap) * 0.6f +
                        clampDistance * 80.0f + ringPenalty + directionPenalty
                };
                size_t insertAt = candidateCounts[member];
                while (insertAt > 0 &&
                       clusterCandidates_[member][insertAt - 1].cost > value.cost) {
                    clusterCandidates_[member][insertAt] =
                        clusterCandidates_[member][insertAt - 1];
                    insertAt--;
                }
                clusterCandidates_[member][insertAt] = value;
                candidateCounts[member]++;
            };

            for (float offsetDeg : kInnerOffsetsDeg) {
                addCandidate(offsetDeg, baseGap, 0.0f);
            }
            if (outerGap > baseGap + 0.5f) {
                for (float offsetDeg : kOuterOffsetsDeg) {
                    addCandidate(offsetDeg, outerGap, 6.0f);
                }
            }
            if (candidateCounts[member] == 0) candidatesComplete = false;
        }

        int8_t selected[kClusterMaxLabels];
        int8_t bestSelected[kClusterMaxLabels];
        uint8_t nextCandidate[kClusterMaxLabels] = {};
        size_t solveOrder[kClusterMaxLabels];
        for (size_t member = 0; member < componentCount; member++) {
            selected[member] = -1;
            bestSelected[member] = -1;
            solveOrder[member] = member;
        }
        for (size_t i = 1; i < componentCount; i++) {
            size_t value = solveOrder[i];
            size_t j = i;
            while (j > 0) {
                size_t left = solveOrder[j - 1];
                bool valueBeforeLeft = candidateCounts[value] != candidateCounts[left]
                    ? candidateCounts[value] < candidateCounts[left]
                    : (work_[component[value]].input->mustShow !=
                       work_[component[left]].input->mustShow
                        ? work_[component[value]].input->mustShow
                        : work_[component[value]].input->id <
                          work_[component[left]].input->id);
                if (!valueBeforeLeft) break;
                solveOrder[j] = solveOrder[j - 1];
                j--;
            }
            solveOrder[j] = value;
        }

        bool found = false;
        float bestCost = 1.0e30f;
        float depthCost[kClusterMaxLabels + 1] = {};
        size_t visitedNodes = 0;
        size_t depth = 0;
        while (candidatesComplete && visitedNodes < kClusterSearchNodeLimit) {
            if (depth == componentCount) {
                found = true;
                if (depthCost[depth] < bestCost) {
                    bestCost = depthCost[depth];
                    for (size_t member = 0; member < componentCount; member++) {
                        bestSelected[member] = selected[member];
                    }
                }
                depth--;
                selected[solveOrder[depth]] = -1;
                continue;
            }
            size_t member = solveOrder[depth];
            bool advanced = false;
            while (nextCandidate[member] < candidateCounts[member] &&
                   visitedNodes < kClusterSearchNodeLimit) {
                size_t candidateIndex = nextCandidate[member]++;
                visitedNodes++;
                const ClusterCandidateScratch &candidate =
                    clusterCandidates_[member][candidateIndex];
                float candidateCost = depthCost[depth] + candidate.cost;
                if (candidateCost >= bestCost) continue;
                bool overlaps = false;
                for (size_t otherMember = 0;
                     otherMember < componentCount;
                     otherMember++) {
                    if (selected[otherMember] < 0) continue;
                    const ClusterCandidateScratch &other = clusterCandidates_[otherMember][
                        static_cast<size_t>(selected[otherMember])
                    ];
                    const LabelLayoutInput &input =
                        *work_[component[member]].input;
                    const LabelLayoutInput &otherInput =
                        *work_[component[otherMember]].input;
                    if (rectOverlapDepthWithMargin(
                            candidate.x,
                            candidate.y,
                            input.width,
                            input.height,
                            other.x,
                            other.y,
                            otherInput.width,
                            otherInput.height,
                            kLabelCollisionMarginPx) > 0.0f) {
                        overlaps = true;
                        break;
                    }
                }
                if (overlaps) continue;
                selected[member] = static_cast<int8_t>(candidateIndex);
                depthCost[depth + 1] = candidateCost;
                depth++;
                if (depth < componentCount) {
                    nextCandidate[solveOrder[depth]] = 0;
                }
                advanced = true;
                break;
            }
            if (advanced) continue;
            selected[member] = -1;
            nextCandidate[member] = 0;
            if (depth == 0) break;
            depth--;
            selected[solveOrder[depth]] = -1;
        }

        if (!found) {
            for (size_t member = 0; member < componentCount; member++) {
                work_[component[member]].state->clusterRetryAfterMs =
                    nowMs + kClusterRetryMs;
            }
            continue;
        }

        for (size_t member = 0; member < componentCount; member++) {
            size_t workIndex = component[member];
            WorkItem &work = work_[workIndex];
            State &state = *work.state;
            const ClusterCandidateScratch &target = clusterCandidates_[member][
                static_cast<size_t>(bestSelected[member])
            ];
            float angleDelta = normalizeAngle(target.angle - state.orbitTargetAngle);
            float gapDelta = target.gap - state.orbitTargetGap;
            state.orbitTargetAngle = target.angle;
            state.orbitTargetGap = target.gap;
            state.orbitTargetActive = fabsf(angleDelta) > kOrbitArrivalAngleRad ||
                fabsf(gapDelta) > 0.75f;
            state.orbitAngleValid = true;
            state.orbitDirection = angleDelta > 0.0f ? 1 :
                (angleDelta < 0.0f ? -1 : 0);
            state.orbitLockUntilMs = nowMs + kOrbitDirectionLockMs;
            state.orbitCooldownUntilMs = 0;
            state.orbitGapCompactAfterMs = 0;
            state.clusterConflictFrames = 0;
            state.clusterRetryAfterMs = 0;
        }
        for (size_t member = 0; member < componentCount; member++) {
            advanceOrbitTarget(component[member]);
        }
    }

    for (size_t collisionIndex = 0; collisionIndex < workCount; collisionIndex++) {
        size_t workIndex = collisionOrder[collisionIndex];
        WorkItem &work = work_[workIndex];
        const LabelLayoutInput &input = *work.input;
        float conflictDepth = work.orbiting || work.coolingDown
            ? 0.0f
            : labelConflictDepth(workIndex, work.x, work.y);
        size_t searchOffset = collisionIndex >= collisionSearchStart
            ? collisionIndex - collisionSearchStart
            : collisionIndex + workCount - collisionSearchStart;
        bool searchScheduled = workCount <= kCollisionSearchesPerFrame ||
            searchOffset < kCollisionSearchesPerFrame || input.mustShow;
        if (kResolveOverlappingLabels &&
            conflictDepth > 0.0f &&
            searchScheduled &&
            !clusterPending[workIndex]) {
            float centerX = work.x + input.width * 0.5f;
            float centerY = work.y + input.height * 0.5f;
            float radialX = centerX - input.anchorX;
            float radialY = centerY - input.anchorY;
            float radialLength = sqrtf(radialX * radialX + radialY * radialY);
            if (radialLength < 0.001f) {
                radialX = work.rightX;
                radialY = work.rightY;
            } else {
                radialX /= radialLength;
                radialY /= radialLength;
            }

            float currentScore = placementScore(
                workIndex,
                work.x,
                work.y,
                0.0f,
                0.0f,
                0
            );
            float bestScore = currentScore;
            float bestOffset = 0.0f;
            float bestTargetAngle = atan2f(radialY, radialX);
            int8_t bestDirection = 0;

            auto evaluateCandidate = [&](float offset,
                                         float cosine,
                                         float sine,
                                         float turnPenalty,
                                         int8_t direction) {
                float directionX = radialX * cosine - radialY * sine;
                float directionY = radialX * sine + radialY * cosine;
                float candidateX = 0.0f;
                float candidateY = 0.0f;
                float clampDistance = 0.0f;
                positionAtDirection(
                    input,
                    bounds,
                    directionX,
                    directionY,
                    kPreferredSymbolGapPx,
                    candidateX,
                    candidateY,
                    clampDistance
                );
                float score = placementScore(
                    workIndex,
                    candidateX,
                    candidateY,
                    clampDistance,
                    turnPenalty,
                    direction
                );
                if (score < bestScore) {
                    bestScore = score;
                    bestOffset = offset;
                    bestTargetAngle = atan2f(directionY, directionX);
                    bestDirection = direction;
                }
            };

            for (const OrbitRotation &rotation : kOrbitRotations) {
                float offset = rotation.angle;
                int8_t direction = rotation.direction;
                if (fabsf(offset - kPi) < 0.001f) {
                    direction = work.state->orbitDirection != 0
                        ? work.state->orbitDirection
                        : ((input.id & 1U) ? 1 : -1);
                    offset = kPi * static_cast<float>(direction);
                }
                evaluateCandidate(
                    offset,
                    rotation.cosine,
                    rotation.sine,
                    rotation.turnPenalty,
                    direction
                );
            }

            float refinementBase = bestOffset;
            static constexpr int8_t kRefinementDirections[] = {-1, 1};
            for (int8_t refinementDirection : kRefinementDirections) {
                float offset = normalizeAngle(
                    refinementBase +
                    static_cast<float>(refinementDirection) *
                        10.0f * kDegreesToRadians
                );
                int8_t direction = offset > 0.0f ? 1 : -1;
                float degrees = fabsf(offset) / kDegreesToRadians;
                evaluateCandidate(
                    offset,
                    cosf(offset),
                    sinf(offset),
                    degrees * 0.03f,
                    direction
                );
            }

            float requiredImprovement = maxFloat(4.0f, currentScore * 0.08f);
            if (bestScore + requiredImprovement < currentScore) {
                work.state->orbitTargetAngle = bestTargetAngle;
                work.state->orbitTargetGap = kPreferredSymbolGapPx;
                work.state->orbitTargetActive = true;
                work.state->orbitAngleValid = true;
                work.state->orbitDirection = bestDirection;
                work.state->orbitLockUntilMs = nowMs + kOrbitDirectionLockMs;
                work.state->orbitCooldownUntilMs = 0;
                work.state->orbitGapCompactAfterMs = 0;
                advanceOrbitTarget(workIndex);
            }
        }
        if (work.state->visible || input.mustShow) {
            reserved[workIndex] = true;
            if (work.state->orbitAngleValid) {
                float clampDistance = 0.0f;
                positionAtDirection(
                    input,
                    bounds,
                    cosf(work.state->orbitTargetAngle),
                    sinf(work.state->orbitTargetAngle),
                    work.state->orbitTargetGap,
                    reservedX[workIndex],
                    reservedY[workIndex],
                    clampDistance
                );
            } else {
                reservedX[workIndex] = work.x;
                reservedY[workIndex] = work.y;
            }
        }
    }

    if (workCount > kCollisionSearchesPerFrame) {
        collisionSearchCursor_ =
            (collisionSearchStart + kCollisionSearchesPerFrame) % workCount;
    } else {
        collisionSearchCursor_ = 0;
    }

    for (size_t i = 0; i < workCount; i++) {
        work_[i].state->x = work_[i].x;
        work_[i].state->y = work_[i].y;
    }

    size_t priorityOrder[kMaxLabels];
    for (size_t i = 0; i < workCount; i++) priorityOrder[i] = i;
    for (size_t i = 1; i < workCount; i++) {
        size_t value = priorityOrder[i];
        size_t j = i;
        while (j > 0) {
            const LabelLayoutInput &left = *work_[priorityOrder[j - 1]].input;
            const LabelLayoutInput &right = *work_[value].input;
            bool rightBeforeLeft = right.mustShow != left.mustShow
                ? right.mustShow
                : (right.distanceKm != left.distanceKm
                    ? right.distanceKm < left.distanceKm
                    : right.id < left.id);
            if (!rightBeforeLeft) break;
            priorityOrder[j] = priorityOrder[j - 1];
            j--;
        }
        priorityOrder[j] = value;
    }

    bool accepted[kMaxLabels] = {};
    float maxOverlap = 0;
    for (size_t priorityIndex = 0; priorityIndex < workCount; priorityIndex++) {
        size_t workIndex = priorityOrder[priorityIndex];
        WorkItem &work = work_[workIndex];
        State &state = *work.state;
        const LabelLayoutInput &input = *work.input;
        bool conflict = false;

        float maxGap = input.mustShow ? kPriorityMaxGapPx : kNormalMaxGapPx;
        if (pointToRectDistanceSquared(
                input.anchorX,
                input.anchorY,
                work.x,
                work.y,
                input.width,
                input.height) > maxGap * maxGap) {
            conflict = true;
        }

        for (size_t acceptedIndex = 0; acceptedIndex < workCount; acceptedIndex++) {
            if (!accepted[acceptedIndex]) continue;
            const WorkItem &other = work_[acceptedIndex];
            float overlap = rectOverlapDepth(
                work.x,
                work.y,
                input.width,
                input.height,
                other.x,
                other.y,
                other.input->width,
                other.input->height
            );
            if (overlap > maxOverlap) maxOverlap = overlap;
            if (kResolveOverlappingLabels &&
                overlap > 0.5f &&
                !work.orbiting &&
                !other.orbiting &&
                !work.coolingDown &&
                !other.coolingDown) {
                conflict = true;
            }
        }

        if (input.mustShow) {
            state.visible = true;
            state.conflictFrames = 0;
            state.cleanFrames = 0;
        } else if (conflict) {
            if (state.conflictFrames < 255) state.conflictFrames++;
            state.cleanFrames = 0;
            if (state.conflictFrames >= kHideAfterConflictFrames) {
                state.visible = false;
            }
        } else {
            state.conflictFrames = 0;
            if (state.cleanFrames < 255) state.cleanFrames++;
            if (!state.visible && state.cleanFrames >= kShowAfterCleanFrames) {
                state.visible = true;
            }
        }

        if (state.visible) accepted[workIndex] = true;
        LabelLayoutOutput &output = outputs[work.outputIndex];
        output.x = work.x;
        output.y = work.y;
        output.visible = state.visible;
    }

    if (metrics != nullptr) {
        metrics->maxOverlapPx = maxOverlap;
        for (size_t i = 0; i < workCount; i++) {
            if (work_[i].state->visible) {
                metrics->visibleCount++;
            } else {
                metrics->hiddenCount++;
            }
        }
    }
}

}  // namespace RadarLabels
