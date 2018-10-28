#include <ros/ros.h>

#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> 

#include "gen/GazeboObjectState.hpp"

class MinLaserSubscriber
{
private:
    int domain_id;
    std::shared_ptr<ros::NodeHandle> nh;
public:
    MinLaserSubscriber(std::shared_ptr<ros::NodeHandle> nh, int domain_id):
        nh(nh), domain_id(domain_id)
    {

    }

    void startDDSCycle()
    {
        dds::domain::DomainParticipant participant(domain_id);
    }
};

int main(int argc, char* argv[])
{
    int domain_id = 0;
    int sample_count = 0; // infinite loop
    ros::init(argc, argv, "ros_subscriber_laser");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    MinLaserSubscriber m(nh, 0);
    return 0;
}