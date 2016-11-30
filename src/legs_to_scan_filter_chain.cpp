
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"
#include "tf/transform_listener.h"
#include "filters/filter_chain.h"
#include "bayes_people_tracker/PeopleTracker.h"
#include <boost/thread/mutex.hpp>
#include <vector>

#ifndef leg_glob
#define leg_glob
#endif
#include "global.h" 

class LegsToScanFilterChain {
    
protected:
    // Our NodeHandle
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Components for tf::MessageFilter
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
    message_filters::Subscriber<bayes_people_tracker::PeopleTracker> people_sub_;

    // Filter Chain
    filters::FilterChain<sensor_msgs::LaserScan> filter_chain_;

    // Components for publishing
    sensor_msgs::LaserScan msg_;
    ros::Publisher output_pub_;

    // Deprecation helpers
    ros::Timer deprecation_timer_;

public:
    // Constructor

    LegsToScanFilterChain() :
    private_nh_("~"),
    scan_sub_(nh_, "scan_merged", 50),
    people_sub_(nh_, "people_tracker/positions", 50), //50?
    filter_chain_("sensor_msgs::LaserScan") {
        // Configure filter chain
        filter_chain_.configure("scan_filter_chain", private_nh_);

        scan_sub_.registerCallback(boost::bind(&LegsToScanFilterChain::scancallback, this, _1));
        people_sub_.registerCallback(boost::bind(&LegsToScanFilterChain::legcallback, this, _1));

        // Advertise output
        output_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_no_legs", 1000);

        // Set up deprecation printout
        deprecation_timer_ = nh_.createTimer(ros::Duration(5.0), boost::bind(&LegsToScanFilterChain::deprecation_warn, this, _1));
    }

    // Destructor

    ~LegsToScanFilterChain() {
    }

    // Deprecation warning callback

    void deprecation_warn(const ros::TimerEvent& e) {
    }

    // Callback

    void scancallback(const sensor_msgs::LaserScan::ConstPtr& msg_in) {
        // Run the filter chain
        if (filter_chain_.update(*msg_in, msg_)) {
            //only publish result if filter succeeded
            output_pub_.publish(msg_);
        }
    }
    
    void legcallback(const bayes_people_tracker::PeopleTracker::ConstPtr& msg_in) {
        leg_lock.lock();
        ROS_ERROR("a \n");
        leg_distances.assign(msg_in->distances.begin(),msg_in->distances.end());
        ROS_ERROR("b \n");
        leg_angles.assign(msg_in->angles.begin(),msg_in->angles.end());
        ROS_ERROR("isInit set to true \n");
        isInit = true;
        leg_lock.unlock();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "scan_to_scan_filter_chain");

    LegsToScanFilterChain t;
    ros::spin();

    return 0;
}