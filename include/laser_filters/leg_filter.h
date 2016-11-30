
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
            int maxIndex = input_scan.angle_max / input_scan.angle_increment;
            ROS_ERROR("maxIndex %u \n" , maxIndex);
            if (loadLegs()) {
                ROS_ERROR("c \n");
                for (int j = 0; j<angles_.size(); ++j) {
                    ROS_ERROR("d \n");
                    double angle_diff = angles_[j] - input_scan.angle_min;
                    ROS_ERROR("e \n");
                    int steps = angle_diff / input_scan.angle_increment;
                    ROS_ERROR("f \n");
                    steps = std::max<int>(0,steps-(angle_range / input_scan.angle_increment));
                    ROS_ERROR("g \n");
                    for (int i = steps; i <= std::min<int>((steps + (angle_range / input_scan.angle_increment)),maxIndex-1); ++i) {
                        ROS_ERROR("h \n");
                        if (input_scan.ranges[i] >= distances_[j] - dist_range &&
                                input_scan.ranges[i] <= distances_[j] + dist_range) {
                            ROS_ERROR("i \n");
                            filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                            ROS_ERROR("j \n");
                        }
                    }
                }
            }
            return true;
        }

        bool loadLegs() {
            leg_lock.lock();
            ROS_ERROR("k \n");
            if (isInit) {
                distances_.assign(leg_distances.begin(),leg_distances.end());
                ROS_ERROR("l \n");
                angles_.assign(leg_angles.begin(),leg_angles.end());
                ROS_ERROR("m \n");
                leg_lock.unlock();
                ROS_ERROR("n \n");
                return true;
            }
            ROS_ERROR("o \n");
            leg_lock.unlock();
            return false;
        }
    };
};
#endif