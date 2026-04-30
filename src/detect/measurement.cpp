#include "measurement.h"

namespace detect {

TargetMeasurement toMeasurement(const Detection& det, int64_t timestamp_ms) {
    TargetMeasurement out;
    out.valid = det.valid;
    out.timestamp = timestamp_ms;
    out.uv = det.center;
    out.confidence = det.confidence;
    out.class_id = det.class_id;
    out.label = det.label;
    out.bbox_area = static_cast<float>(det.bbox.width) * det.bbox.height;
    return out;
}

}  // namespace detect
