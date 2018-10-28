#include "gen/GazeboObjectState.hpp"

// DDS
#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> 
// General stuff
#include <thread>
#include <memory>
#include <ros/ros.h>

#include <std_msgs/Float64.h>

class ExamplePublisher
{
private:
    int domain_id;
    double distance;
    std::thread t_publisher;
    std::shared_ptr<ros::NodeHandle> nh;
    ros::Subscriber sub_distance;
public:
    ExamplePublisher(std::shared_ptr<ros::NodeHandle> nh, int domain_id = 0): nh(nh), domain_id(domain_id), distance(0.0)
    {
        sub_distance = nh->subscribe("/example/distance", 10, &ExamplePublisher::CbDistance, this);
    }

    void CbDistance(const std_msgs::Float64::ConstPtr& msg)
    {
        distance = msg->data;
    }

    void Initialize()
    {
        t_publisher = std::thread(&ExamplePublisher::PublisherThread, this);
        t_publisher.detach();
    }

    void PublisherThread()
    {
        dds::domain::DomainParticipant participant(domain_id);
        dds::topic::Topic<GazeboState::DistanceObjectState> topic(participant, "DistanceObjectGazebo");
        dds::pub::DataWriter<GazeboState::DistanceObjectState> writer(dds::pub::Publisher(participant), topic);
        GazeboState::DistanceObjectState sample;
        
        sample.target("Tram");
        try 
        {
            while(ros::ok())
            {
                sample.distance(distance);
                writer.write(sample);
                rti::util::sleep(dds::core::Duration(0.1));

            }        
        } 
        catch (const std::exception& ex) {
            // This will catch DDS exceptions
            std::cerr << "Exception in publisher_main(): " << ex.what() << std::endl;
            return;
        }
        
    }

};


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "example_pblisher_dds_rti");
    std::shared_ptr<ros::NodeHandle> nh(new ros::NodeHandle());
    ExamplePublisher pub(nh);
    pub.Initialize();
    ros::spin();
    return 0;
}