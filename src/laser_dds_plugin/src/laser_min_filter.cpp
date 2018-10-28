#include <ros/ros.h>

#include <algorithm>
#include <iostream>

#include <sensor_msgs/LaserScan.h>

#include <queue>

#include <dds/pub/ddspub.hpp>
#include <dds/core/ddscore.hpp>
#include <rti/util/util.hpp> 

#include "gen/GazeboObjectState.hpp"

class MinLaserSubscriber
{
private:
    std::shared_ptr<ros::NodeHandle> nh;
    // DDS
    std::shared_ptr<dds::domain::DomainParticipant> participant;
    std::shared_ptr<dds::topic::Topic<GazeboState::MinLaserDistance> > topic;
    std::shared_ptr<dds::pub::DataWriter<GazeboState::MinLaserDistance> > writer;
    // Laser
    ros::Subscriber lasersub;
    std::queue<sensor_msgs::LaserScan> laser_scan_queue;
public:
    MinLaserSubscriber(std::shared_ptr<ros::NodeHandle> nh):
        nh(nh)
    {

    }

    void InitLaserDistanceDDS(int domain_id)
    {
        participant = std::shared_ptr<dds::domain::DomainParticipant>(new dds::domain::DomainParticipant(domain_id));
        topic = std::shared_ptr<dds::topic::Topic<GazeboState::MinLaserDistance> >(
            new dds::topic::Topic<GazeboState::MinLaserDistance>(*participant, 
                "MinLaserDistance")
        );

        // Create a DataWriter with default Qos (Publisher created in-line)
        writer = std::shared_ptr<dds::pub::DataWriter<GazeboState::MinLaserDistance> >(
            new dds::pub::DataWriter<GazeboState::MinLaserDistance> 
                (dds::pub::Publisher(*participant), *topic));

        GazeboState::MinLaserDistance sample;
    }

    void InitLaserRos()
    {
        lasersub = nh->subscribe("/scan",10, &MinLaserSubscriber::CbLaserTopic, this);
    }

    void CbLaserTopic(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        laser_scan_queue.push(*msg);
    }

    void LaserUpdateThread()
    {
        
    }
};

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "ros_subscriber_laser");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    MinLaserSubscriber m(nh);
    int domain_id = 0;
    int sample_count = 0; // infinite loop
    m.InitLaserDistanceDDS(domain_id);
    
    if (argc >= 2) {
        domain_id = atoi(argv[1]);
    }
    if (argc >= 3) {
        sample_count = atoi(argv[2]);
    }

    // To turn on additional logging, include <rti/config/Logger.hpp> and
    // uncomment the following line:
    // rti::config::Logger::instance().verbosity(rti::config::Verbosity::STATUS_ALL);

    try {
        //publisher_main(domain_id, sample_count);
    } catch (const std::exception& ex) {
        // This will catch DDS exceptions
        std::cerr << "Exception in publisher_main(): " << ex.what() << std::endl;
        return -1;
    }

    // RTI Connext provides a finalize_participant_factory() method
    // if you want to release memory used by the participant factory singleton.
    // Uncomment the following line to release the singleton:
    //
    // dds::domain::DomainParticipant::finalize_participant_factory();

    return 0;
}