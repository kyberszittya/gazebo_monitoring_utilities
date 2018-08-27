#include "plugin_collision.hpp"

namespace gazebo
{
    bool defaultDistanceFunction(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* cdata_, double& dist)
    {
        auto* cdata = static_cast<DistanceDataD*>(cdata_);
        const fcl::DistanceRequestd& request = cdata->request;
        fcl::DistanceResultd& result = cdata->result;

        if(cdata->done) { dist = result.min_distance; return true; }

        fcl::distance(o1, o2, request, result);

        dist = result.min_distance;

        if(dist <= 0) return true; // in collision or in touch

        return cdata->done;
    }
    void DistanceCalculation::initializeObjectSet()
    {
        ignored_objects.insert("ground_plane");

    }

    void DistanceCalculation::pushCollisionObjectDynamic(
        const std::string name,
        std::shared_ptr<fcl::CollisionObjectd> objd,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center
        )
    {
        objd->setTransform(orientation, position + center);
        monitored_map[name] = objd;
        objects[name] = object_cnt++;
        dynamic_object_set->registerObject(objd.get());
        monitored_object_set->registerObject(objd.get());
        objects_collision_mesh.push_back(objd);
        objects_collision_center.push_back(center);
    }

    void DistanceCalculation::pushCollisionObjectStatic(
        const std::string name,
        std::shared_ptr<fcl::CollisionObjectd> objd,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center
        )
    {
        objd->setTransform(orientation, position + center);
        monitored_map[name] = objd;
        objects[name] = object_cnt++;
        static_object_set->registerObject(objd.get());
        monitored_object_set->registerObject(objd.get());
        objects_collision_mesh.push_back(objd);
        objects_collision_center.push_back(center);
    }

    void DistanceCalculation::pushCollisionObjectTarget(
        const std::string name,
        std::shared_ptr<fcl::CollisionObjectd> objd,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center
        )
    {
        objd->setTransform(orientation, position + center);
        target_map[name] =  objd;
        objects[name] = object_cnt++;
        target_object_set->registerObject(objd.get());
        objects_collision_mesh.push_back(objd);
        objects_collision_center.push_back(center);
    }

    // @brief: is object ignored based on name
    bool DistanceCalculation::isIgnoredObject(std::string name)
    {
        return ignored_objects.find(name) != ignored_objects.end();
    }

    // @brief: is object target based on its name
    bool DistanceCalculation::isTargetObject(std::string name)
    {
        return target_objects_name.find(name) != target_objects_name.end();
    }

    // @brief: is object target based on its index
    bool DistanceCalculation::isTargetObject(unsigned int id)
    {
        return target_objects.find(id) != target_objects.end();
    }

    // @brief: A dynamic object with box mesh
    void DistanceCalculation::addBoxCollisionMeshDynamic(
        const std::string name,
        const Eigen::Vector3d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Boxd> s(new fcl::Boxd(size[0], size[1], size[2]));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectDynamic(name, objd, position, orientation, center);
    }

    // @brief: A dynamic object with sphere mesh
    void DistanceCalculation::addSphereCollisionMeshDynamic(std::string name,
        const double& radius,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Sphered> s(new fcl::Sphered(radius));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectDynamic(name, objd, position, orientation, center);
    }

    // @brief: A dynamic object with cylinder mesh
    void DistanceCalculation::addCylinderCollisionMeshDynamic(std::string name,
        const Eigen::Vector2d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Cylinderd> s(new fcl::Cylinderd(size[0], size[1]));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectDynamic(name, objd, position, orientation, center);
    }

    // @brief: A static object with box mesh
    void DistanceCalculation::addBoxCollisionMeshStatic(std::string name,
        const Eigen::Vector3d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Boxd> s(new fcl::Boxd(size[0], size[1], size[2]));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectStatic(name, objd, position, orientation, center);
    }

    // @brief: A static object with sphere mesh
    void DistanceCalculation::addSphereCollisionMeshStatic(std::string name,
        const double& radius,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {

        std::shared_ptr<fcl::Sphered> s(new fcl::Sphered(radius));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectStatic(name, objd, position, orientation, center);
    }

    // @brief: A static object with cylinder mesh
    void DistanceCalculation::addCylinderCollisionMeshStatic(std::string name,
        const Eigen::Vector2d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Cylinderd> s(new fcl::Cylinderd(size[0], size[1]));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectStatic(name, objd, position, orientation, center);
    }



    // @brief: A target with box collision mesh
    void DistanceCalculation::addBoxCollisionMeshTarget(std::string name,
        const Eigen::Vector3d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        if (objects.find(name) == objects.end())
        {
            std::shared_ptr<fcl::Boxd> s(new fcl::Boxd(size[0], size[1], size[2]));
            std::shared_ptr<fcl::CollisionObjectd> objd(
                new fcl::CollisionObjectd(s));
            pushCollisionObjectTarget(name, objd, position, orientation, center);
        }

    }

    // @brief: A target object with sphere mesh
    void DistanceCalculation::addSphereCollisionMeshTarget(std::string name,
        const double& radius,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Sphered> s(new fcl::Sphered(radius));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectTarget(name, objd, position, orientation, center);
    }

    // @brief: A static object with cylinder mesh
    void DistanceCalculation::addCylinderCollisionMeshTarget(std::string name,
        const Eigen::Vector2d& size,
        const Eigen::Vector3d& position,
        const Eigen::Quaterniond& orientation,
        const Eigen::Vector3d& center = Eigen::Vector3d(0.0, 0.0, 0.0))
    {
        std::shared_ptr<fcl::Cylinderd> s(new fcl::Cylinderd(size[0], size[1]));
        std::shared_ptr<fcl::CollisionObjectd> objd(
            new fcl::CollisionObjectd(s));
        pushCollisionObjectTarget(name, objd, position, orientation, center);
    }

    // @brief: Setup before use
    void DistanceCalculation::setupCollision()
    {
        monitored_object_set->setup();
        target_object_set->setup();
        static_object_set->setup();
        dynamic_object_set->setup();
    }

    // @brief: Update object pose identified by its name
    void DistanceCalculation::updateObjectPose(std::string name, Eigen::Vector3d t, Eigen::Quaterniond r)
    {
        unsigned int obj_ind = objects[name];
        objects_collision_mesh[obj_ind]->setTransform(r, t + objects_collision_center[obj_ind]);
    }

    // @brief: Update object pose identified by its ordinal
    void DistanceCalculation::updateObjectPose(unsigned int i, Eigen::Vector3d t, Eigen::Quaterniond r)
    {
        objects_collision_mesh[i]->setTransform(r, t + objects_collision_center[i]);
    }

    // @brief: Get minimal narrowphase distance
    double DistanceCalculation::getMinTargetDistanceNarrowphase()
    {
        DistanceDataD distance_data;
        target_object_set->distance(monitored_object_set.get(), &distance_data, defaultDistanceFunction);
        auto res = distance_data.result;
        return res.min_distance < 0.0 ? 0.0: res.min_distance;
    }

    // Get minimal distance between targets and monitored objects
    double DistanceCalculation::getMinTargetDistance()
    {
        fcl::DistanceResultd dist_result;

        double min_dist = std::numeric_limits<double>::max();
        for (auto it_target = target_map.begin(); it_target != target_map.end(); ++it_target)
        {
            for (auto it_monitored = monitored_map.begin(); it_monitored != monitored_map.end(); ++it_monitored)
            {
                double res = fcl::distance(it_target->second.get(),
                                        it_monitored->second.get(),
                        dist_request, dist_result);
                min_dist = min_dist > res ? res : min_dist;
                if (min_dist < 0.0)
                {
                    return 0.0;
                }

            }

        }
        return min_dist;
    }

    void DistanceCalculation::addTargetObject(std::string name)
    {
        target_objects_name.insert(name);
    }
}
