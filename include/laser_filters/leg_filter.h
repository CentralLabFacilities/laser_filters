
#ifndef LASER_SCAN_LEG_FILTER_H
#define LASER_SCAN_LEG_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

extern std::vector<double> distances;
extern std::vector<double> angles;
extern boost::mutex leg_lock;

namespace laser_filters {

    class LaserScanLegFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        double angle_range;
        double dist_range;
        std::vector<double> distances_;
        std::vector<double> angles_;

        bool configure() {
            if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("angle_range"), angle_range)) {
                ROS_WARN("Warning: No angle range was set. Using default.\n");
                angle_range=5;
            }
            if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("dist_range"), dist_range)) {
                ROS_WARN("Warning: No distance range was set. Using default.\n");
                angle_range=10;
            }
            leg_lock.lock();
            distances_ = distances;
            angles_ = angles;
            leg_lock.unlock();
            return true;
        }

        virtual ~LaserScanLegFilter() {
        }

        bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
            filtered_scan = input_scan;

            int counter = 0;
            for (double ang : angles_) {
                double cut_min = ang - angle_range;
                int start = cut_min / input_scan.angle_increment;
                for (int i = start; i <= start + 10; ++i) {
                    if (input_scan.ranges[i] >= distances[counter] - dist_range &&
                            input_scan.ranges[i] <= distances[counter] + dist_range) {
                        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                    }
                }
                ++counter;
            }
            return true;
        }
    };
};
#endif