#ifndef DETECT_MEASUREMENT_H
#define DETECT_MEASUREMENT_H

#include <cstdint>

#include <opencv2/core.hpp>

#include "common/types.h"
#include "detector.h"

namespace detect {

using TargetMeasurement = common::TargetMeasurement;

TargetMeasurement toMeasurement(const Detection& det, int64_t timestamp_ms);

}  // namespace detect

#endif  // DETECT_MEASUREMENT_H
