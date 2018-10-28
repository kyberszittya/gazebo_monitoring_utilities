#include <gazebo_rti_state_plugin/plugin_collision_rti_dds.hpp>

namespace gazebo
{
    void DistanceGazeboPluginRtiDds::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        InitPlugin(_world, _sdf);
    }

    

    GZ_REGISTER_WORLD_PLUGIN(DistanceGazeboPluginRtiDds);
}