#include "src/geometry/kernel/point_3d.h"
#include "src/geometry/util/distance_3d.h"

/**
 * Metric used in VCCS supervoxel segmentation.
 *
 * Reference:
 *   Rusu, R.B., Cousins, S., 2011. 3d is here: Point cloud library (pcl),
 *   IEEE International Conference on Robotics and Automation, pp. 1–4.
 */
class VCCSMetric {
    public:
        explicit VCCSMetric(double resolution)
            : resolution_(resolution) {}
    
        double operator() (const cl::PointWithNormal& p1, const cl::PointWithNormal& p2) const {
            return 1.0 - std::fabs(p1.normal * p2.normal) + cl::geometry::Distance(p1, p2) / resolution_ * 0.4;
        }
    
    private:
        double resolution_;
};