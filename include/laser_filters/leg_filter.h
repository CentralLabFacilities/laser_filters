
#ifndef LASER_SCAN_LEG_FILTER_H
#define LASER_SCAN_LEG_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include "../src/global.h"

namespace laser_filters {

    class LaserScanLegFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        double angle_range; //in rad
        double dist_range;
        std::vector<double> distances_;
        std::vector<double> angles_;

        bool configure() {
            if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("angle_range"), angle_range)) {
                ROS_WARN("Warning: No angle range was set. Using default.\n");
                angle_range = 5*(3.14159265358979323846/180); //convert to rad
            }
            if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("dist_range"), dist_range)) {
                ROS_WARN("Warning: No distance range was set. Using default.\n");
                dist_range = 0.1;
            }
            return true;
        }

        virtual ~LaserScanLegFilter() {
        }

        bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
            filtered_scan = input_scan;
            if (loadLegs()) {
                ROS_ERROR("start filtering legs \n");
                for (int j = 0; j<angles_.size(); ++j) {
                    double angle_diff = angles_[j] - input_scan.angle_min;
                    int steps = angle_diff / input_scan.angle_increment;
                    steps = std::min<int>(0,steps-(angle_range / input_scan.angle_increment));
                    
                    for (int i = steps; i <= steps + (angle_range / input_scan.angle_increment); ++i) {
                        if (input_scan.ranges[i] >= distances_[j] - dist_range &&
                                input_scan.ranges[i] <= distances_[j] + dist_range) {
                            filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                        }
                    }
                }
            }
            return true;
        }

        bool loadLegs() {
            leg_lock.lock();
            if (isInit) {
                distances_ = leg_distances;
                angles_ = leg_angles;
                leg_lock.unlock();
                return true;
            }
            leg_lock.unlock();
            return false;
        }
    };
};
#endif