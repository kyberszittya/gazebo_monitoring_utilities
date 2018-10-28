#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
// DDS
#include <dds/pub/ddspub.hpp>
#include <rti/util/util.hpp> 

#include "gen/GazeboObjectState.hpp"

namespace gazebo
{
    class RtiStateGazeboPlugin: public WorldPlugin
    {
    private:
        std::shared_ptr<dds::domain::DomainParticipant> participant;
        std::shared_ptr<dds::topic::Topic<GazeboState::EnvironmentState> > topic;
        std::shared_ptr<dds::pub::DataWriter<GazeboState::EnvironmentState> > writer;
        
        event::ConnectionPtr update_event_;
        physics::WorldPtr world;
    public:
        virtual void Load(physics::WorldPtr _world, sdf::ElementPtr sdf) override
        {
            this->world = _world;
            update_event_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RtiStateGazeboPlugin::StateUpdate, this));
            participant = std::shared_ptr<dds::domain::DomainParticipant>(new dds::domain::DomainParticipant(0));
            topic = std::shared_ptr<dds::topic::Topic<GazeboState::EnvironmentState> >(
                new dds::topic::Topic<GazeboState::EnvironmentState>(*participant, "EnvironmentState")
            );
            writer = std::shared_ptr<dds::pub::DataWriter<GazeboState::EnvironmentState> >(
                new dds::pub::DataWriter<GazeboState::EnvironmentState>(dds::pub::Publisher(*participant), *topic)
            );
        }

        void StateUpdate()
        {
            GazeboState::EnvironmentState env_state;
            for(unsigned int i = 0; i < world->ModelCount(); i++)
            {
                GazeboState::ObjectState objectstate;
                auto model = world->ModelByIndex(i);
                auto pos = model->WorldPose().Pos();
                auto linvel = model->WorldLinearVel();
                auto bounding_box = model->CollisionBoundingBox().Size();
                GazeboState::Vector3 p(pos.X(), pos.Y(), pos.Z());
                GazeboState::Vector3 v(linvel.X(), linvel.Y(), linvel.Z());
                GazeboState::Vector3 b_box(bounding_box.X(), bounding_box.Y(), bounding_box.Z());
                objectstate.pos(p);
                objectstate.target(model->GetName());
                objectstate.lin_vel(v);
                //objectstate.bounding_box()

                env_state.objects().push_back(objectstate);
            }
            env_state.world_name(world->Name());
            writer->write(env_state);
        }

    };
    GZ_REGISTER_WORLD_PLUGIN(RtiStateGazeboPlugin)
}