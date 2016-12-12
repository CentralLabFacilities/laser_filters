
#ifndef LASER_SCAN_LEG_FILTER_H
#define LASER_SCAN_LEG_FILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include "../src/global.h"

namespace laser_filters {

    class LaserScanLegFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        double angle_range; //config in deg. gets converted to rad
        double dist_range;
        std::vector<double> distances_;
        std::vector<double> angles_;

        bool configure() {
            if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam(std::string("angle_range"), angle_range)) {
                ROS_WARN("Warning: No angle range was set. Using default.\n");
                angle_range = 5 * (3.14159265358979323846 / 180); //convert to rad
            } else {
                angle_range = angle_range * (3.14159265358979323846 / 180); //convert to rad
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
            int maxIndex = (input_scan.angle_max-input_scan.angle_min) / input_scan.angle_increment;
            if (loadLegs()) {
                for (int j = 0; j < angles_.size(); ++j) {
                    double angle_diff = -angles_[j] - input_scan.angle_min;
                    //Compute angle_range based o n distance
                    angle_range = 2*atan2(0.25,distances_[j]); 
                    int steps = angle_diff / input_scan.angle_increment;
                    int startsteps = std::max<int>(0, steps - (angle_range / input_scan.angle_increment));
                    for (int i = startsteps; i <= std::min<int>((steps + (angle_range / input_scan.angle_increment)), maxIndex - 1); ++i) {
                        if (input_scan.ranges[i] >= distances_[j] - dist_range &&
                                input_scan.ranges[i] <= distances_[j] + dist_range) {
                            filtered_scan.ranges[i] = input_scan.range_max + 1.0;
                        }
                    }
                }
            }
            return true;
        }

        bool loadLegs() {
            leg_lock.lock();
            if (isInit) {
                distances_.assign(leg_distances.begin(), leg_distances.end());
                angles_.assign(leg_angles.begin(), leg_angles.end());
                leg_lock.unlock();
                return true;
            }
            leg_lock.unlock();
            return false;
        }
    };
}
#endif