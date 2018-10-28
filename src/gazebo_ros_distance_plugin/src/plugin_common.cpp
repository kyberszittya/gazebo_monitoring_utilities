#include <gazebo_ros_distance_plugin/plugin_collision.hpp>

namespace gazebo
{
    


    void DistanceGazeboPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        InitPlugin(_world, _sdf);
    }



    void DistanceGazeboPlugin::InitPlugin(physics::WorldPtr _world, sdf::ElementPtr _sdf)
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
            update_rate = 1.0/2.0;
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
        // Get ignored objects
        for (auto it = _sdf->GetElement("ignored_object");
            it != nullptr; it = it->GetNextElement("ignored_object"))
        {
            std::string name_ignored = it->Get<std::string>();
            std::cout << "Ignored object: " << name_ignored << std::endl;
            distance_calculation->addIgnoredObject(name_ignored);
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
                auto rot = model->WorldPose().Rot();                
                Eigen::Quaterniond q(rot.W(), rot.X(), rot.Y(), rot.Z());
                Eigen::Vector3d s(size.X(), size.Y(), size.Z());
                if ((rot.Yaw() > M_PI_2 && rot.Yaw() < M_PI) || (rot.Yaw() > M_PI && rot.Yaw() < 3.0*M_PI_2))
                {
                    s[0] = size.Y();
                    s[1] = size.X();
                }
                
                Eigen::Vector3d c(center.X(), center.Y(), center.Z());
                
                
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
                std::cout << "Bounding box size: " << s << std::endl;
            }

        }
        Init();
        distance_thread = std::thread(&DistanceGazeboPlugin::DistanceUpdateThread, this);
        distance_thread.detach();
    }

    void DistanceGazeboPlugin::DistanceUpdateThread()
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

    void DistanceGazeboPlugin::Update()
    {
        /*
        auto t = world->SimTime();
        if ((t - last_update_time).Double() > update_rate)
        {
            last_update_time = t;
            for(const auto &v: model_names)
            {
                auto obj = world->ModelByName(v);
                auto b_box = obj->CollisionBoundingBox();
                auto b_pos = b_box.Center();
                auto b_size = b_box.Size();
                auto pose = obj->WorldPose();
                auto pos = pose.Pos();
                auto q = pose.Rot();
                Eigen::Vector3d t(b_pos.X(), b_pos.Y(), b_pos.Z());
                //Eigen::Quaterniond q0(q.W(), q.X(), q.Y(), q.Z());
                Eigen::Vector3d s(b_size.X(), b_size.Y(), b_size.Z());
                //distance_calculation->updateObjectPose(v, t, q0);
                //distance_calculation->updateBoundingBox(v, t, s);
            }

        }
        */
    }

    void DistanceGazeboPluginRos::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
        InitPlugin(_world, _sdf);
    }

    
}
