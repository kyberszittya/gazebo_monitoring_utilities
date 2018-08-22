#include "plugin_collision.hpp"

namespace gazebo
{
    
        

    void DistanceGazeboRosPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        distance_calculation = std::unique_ptr<DistanceCalculation>(new DistanceCalculation());
        sdf = _sdf;
        world = _world;
        last_update_time = world->SimTime();
        // Loading update rate
        if (_sdf->HasElement("update_rate"))
        {
            update_rate = 1.0/_sdf->Get<double>("update_rate");
            std::cout << "Using update rate from SDF: " << update_rate << std::endl;   
        }
        else
        {
            update_rate = 1.0/5.0;
            std::cout << "Using default update rate: " << update_rate << std::endl;
        }
        // Get target objects
        for (auto it = _sdf->GetElement("target_object"); 
            it != nullptr; it = it->GetNextElement("target_object"))
        {
            std::string name_target = it->Get<std::string>();
            std::cout << "Target: " << name_target << std::endl;
            distance_calculation->addTargetObject(name_target);
        }
        for(unsigned int i = 0; i < world->ModelCount(); i++)
        {
            auto model = world->ModelByIndex(i);
            std::string name = model->GetName();
            const Eigen::Vector3d center0(0.0, 0.0, 0.0);
            // Exclude some ignored objects                
            if (!distance_calculation->isIgnoredObject(name))
            {
                // Add bounding box: for monitoring this is sufficient enough
                auto size = model->CollisionBoundingBox().Size();
                auto center = model->CollisionBoundingBox().Center();
                Eigen::Vector3d s(size.X(), size.Y(), size.Z());
                Eigen::Vector3d c(center.X(), center.Y(), center.Z());
                Eigen::Quaterniond q(1,0,0,0);
                if (model->IsStatic())
                {
                    std::cout << "Static object: " << name << std::endl;
                    distance_calculation->addBoxCollisionMeshStatic(name, s, c, q, center0);
                }
                else
                {
                    if (distance_calculation->isTargetObject(name))
                    {
                        std::cout << "Target object: " << name << std::endl;
                        distance_calculation->addBoxCollisionMeshTarget(name, s, c, q, center0);
                    }
                    else
                    {
                        std::cout << "Dynamic object: " << name << std::endl;
                        distance_calculation->addBoxCollisionMeshDynamic(name, s, c, q, center0);
                    }
                }
                model_names.push_back(name);
                std::cout << "Bounding box center: " << center << std::endl;
                std::cout << "Bounding box size: " << size << std::endl;
            }
            
        }
        std::cout << "Distance: " << distance_calculation->getMinTargetDistance() << std::endl;
        update_event_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&DistanceGazeboRosPlugin::Update, this));

        distance_thread = std::thread(&DistanceGazeboRosPlugin::DistanceUpdateThread, this);
        distance_thread.detach();
    }

    

    void DistanceGazeboRosPlugin::DistanceUpdateThread()
    {
        auto t_last = world->SimTime();
        std::cout << "Starting distance calculation thread" << std::endl;
        while(true)
        {
            auto t = world->SimTime();
            if ((t - t_last).Double() > update_rate)
            {
                DistanceUpdatePublish(distance_calculation->getMinTargetDistanceNarrowphase());
                t_last = world->SimTime();
            }
        }
        
    }

    void DistanceGazeboRosPlugin::Update()
    {
        auto t = world->SimTime();
        if ((t - last_update_time).Double() > update_rate)
        {
            last_update_time = t;
            for(const auto &v: model_names)
            {
                auto obj = world->ModelByName(v);
                auto pose = obj->WorldPose();
                auto pos = pose.Pos();
                auto q = pose.Rot();
                Eigen::Vector3d t(pos.X(), pos.Y(), pos.Z());
                Eigen::Quaterniond q0(q.W(), q.X(), q.Y(), q.Z());
                distance_calculation->updateObjectPose(v, t, q0);
                //distance_calculation->getMinTargetDistance();
                //distance_calculation->getMinTargetDistanceNarrowphase();
            }
            
        }
    }
    GZ_REGISTER_WORLD_PLUGIN(DistanceGazeboRosPlugin)
}