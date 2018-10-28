#ifndef PLUGIN_COLLISION_RTI_DDS_HPP
#define PLUGIN_COLLISION_RTI_DDS_HPP

#include <gazebo_ros_distance_plugin/plugin_collision.hpp>
// DDS
#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> 

#include "gen/GazeboObjectState.hpp"

namespace gazebo
{
    class DistanceGazeboPluginRtiDds: public DistanceGazeboPlugin
    {
    private:
        std::shared_ptr<dds::domain::DomainParticipant> participant;
        std::shared_ptr<dds::topic::Topic<GazeboState::DistanceObjectState> > topic;
        std::shared_ptr<dds::pub::DataWriter<GazeboState::DistanceObjectState> > writer;
        GazeboState::DistanceObjectState sample;
    public:
        ~DistanceGazeboPluginRtiDds()
        {
            participant.reset();
            topic.reset();
            writer.reset();
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

        void Init() override
        {
            participant = std::shared_ptr<dds::domain::DomainParticipant>(new dds::domain::DomainParticipant(0));
            topic = std::shared_ptr<dds::topic::Topic<GazeboState::DistanceObjectState> >(
                new dds::topic::Topic<GazeboState::DistanceObjectState>(*participant, "DistanceObjectGazebo")
            );
            writer = std::shared_ptr<dds::pub::DataWriter<GazeboState::DistanceObjectState> >(
                new dds::pub::DataWriter<GazeboState::DistanceObjectState>(dds::pub::Publisher(*participant), *topic)
            );
            sample.target("tram");

            update_event_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DistanceGazeboPlugin::Update, this));

        }

        virtual void DistanceUpdatePublish(double distance) override
        {
            sample.distance(distance);
            writer->write(sample);
        }

        virtual void DistanceUpdatePublish(const DistanceMeas& meas)
        {
            sample.distance(meas.distance);
            sample.target(meas.name);
            writer->write(sample);
        }

        void DistanceUpdateThread() override
        {
            auto t_last = world->SimTime();
            std::cout << "Starting distance calculation thread (broadphase)" << std::endl;
            while(true)
            {
                auto t = world->SimTime();
                if ((t - t_last).Double() > update_rate)
                {
                    //auto res = distance_calculation->getMinTargetDistance();
                    auto res = getMinTargetDistanceBoundingBox();
                    DistanceUpdatePublish(res);
                    t_last = world->SimTime();
                }
            }
        }
    };
}

#endif
