#include <string>
#include <memory>
// Include containers
#include <map>
#include <vector>
#include <list>
#include <set>
#include <thread>
#include <limits>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// Eigen
#include <Eigen/Dense>
// FCL
#include <fcl/narrowphase/detail/traversal/collision_node.h>
#include <fcl/narrowphase/distance.h>
#include <fcl/narrowphase/distance_request.h>
#include <fcl/narrowphase/distance_result.h>
#include <fcl/narrowphase/detail/traversal/collision_node.h>
#include <fcl/broadphase/broadphase_dynamic_AABB_tree.h>
// ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>


namespace gazebo
{

    struct DistanceDataD
    {
        DistanceDataD()
        {
            done = false;
        }

        /// @brief Distance request
        fcl::DistanceRequestd request;

        /// @brief Distance result
        fcl::DistanceResultd result;

        /// @brief Whether the distance iteration can stop
        bool done;

    };

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
    
    class DistanceCalculation
    {
    private:
        int object_cnt;
        std::map<std::string, unsigned int> objects;
        std::vector<std::shared_ptr<fcl::CollisionObjectd> > objects_collision_mesh;
        std::vector<Eigen::Vector3d> objects_collision_center;
        // Broadphase sets
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> monitored_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> dynamic_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> static_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> target_object_set;
        // Maps
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > monitored_map;
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > target_map;
        // Ignored objects
        std::set<std::string> ignored_objects;
        // Target objects
        std::set<unsigned int> target_objects;
        std::set<std::string> target_objects_name;
        // FCL
        fcl::DistanceRequestd dist_request;

        
        // Initialize ignored object set
        void initializeObjectSet()
        {
            ignored_objects.insert("ground_plane");
            
        }

        void pushCollisionObjectDynamic(
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

        void pushCollisionObjectStatic(
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

        void pushCollisionObjectTarget(
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
    protected:
        
    public:
        DistanceCalculation():
            object_cnt(0),
            target_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            dynamic_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            static_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            monitored_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd())
        {
            initializeObjectSet();
            std::cout << "Setup collision plugin" << std::endl;
            dist_request.enable_nearest_points = true;
            dist_request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        }

        // @brief: is object ignored based on name 
        bool isIgnoredObject(std::string name)
        {
            return ignored_objects.find(name) != ignored_objects.end();
        }

        // @brief: is object target based on its name
        bool isTargetObject(std::string name)
        {
            return target_objects_name.find(name) != target_objects_name.end();
        }

        // @brief: is object target based on its index
        bool isTargetObject(int id)
        {
            return target_objects.find(id) != target_objects.end();
        }

        // @brief: A dynamic object with box mesh
        void addBoxCollisionMeshDynamic(
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
        void addSphereCollisionMeshDynamic(std::string name, 
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
        void addCylinderCollisionMeshDynamic(std::string name, 
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
        void addBoxCollisionMeshStatic(std::string name, 
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
        void addSphereCollisionMeshStatic(std::string name, 
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
        void addCylinderCollisionMeshStatic(std::string name, 
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
        void addBoxCollisionMeshTarget(std::string name, 
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
        void addSphereCollisionMeshTarget(std::string name, 
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
        void addCylinderCollisionMeshTarget(std::string name, 
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
        void setupCollision()
        {
            monitored_object_set->setup();
            target_object_set->setup();
            static_object_set->setup();
            dynamic_object_set->setup();
        }

        // Update object pose identified by its name
        void updateObjectPose(std::string name, Eigen::Vector3d t, Eigen::Quaterniond r)
        {
            int obj_ind = objects[name];
            objects_collision_mesh[obj_ind]->setTransform(r, t + objects_collision_center[obj_ind]);
        }

        // Update object pose identified by its ordinal
        void updateObjectPose(int i, Eigen::Vector3d t, Eigen::Quaterniond r)
        {
            objects_collision_mesh[i]->setTransform(r, t + objects_collision_center[i]);
        }

        // Get minimal narrowphase distance
        double getMinTargetDistanceNarrowphase()
        {
            DistanceDataD distance_data;
            target_object_set->distance(monitored_object_set.get(), &distance_data, defaultDistanceFunction);
            auto res = distance_data.result;
            return res.min_distance < 0.0 ? 0.0: res.min_distance;
        }

        // Get minimal distance between targets and monitored objects
        double getMinTargetDistance()
        {
            fcl::DistanceResultd dist_result;
            
            double min_dist = std::numeric_limits<double>::max();
            for (auto it_target = target_map.begin(); it_target != target_map.end(); ++it_target)
            {
                for (auto it_monitored = monitored_map.begin(); it_monitored != monitored_map.end(); ++it_monitored)
                {
                    auto res = fcl::distance(it_target->second.get(), 
                        it_monitored->second.get(), 
                        dist_request, dist_result);
                    min_dist = min_dist > dist_result.min_distance ? dist_result.min_distance : min_dist;
                    if (min_dist < 0.0)
                    {
                        return 0.0;
                    }
                    
                }
                
            }
            return min_dist;
        }

        void addTargetObject(std::string name)
        {
            target_objects_name.insert(name);
        }

    };

    class DistanceGazeboRosPlugin: public WorldPlugin
    {
    private:
        double update_rate;
        common::Time last_update_time;
        std::unique_ptr<DistanceCalculation> distance_calculation;

        event::ConnectionPtr update_event_;
        std::vector<std::string> model_names;
        sdf::ElementPtr sdf;
        physics::WorldPtr world;

        std::thread distance_thread;
    public:
        DistanceGazeboRosPlugin(): WorldPlugin()
        {
            
        }
        
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
        void DistanceUpdateThread();
        void Update();

        virtual void DistanceUpdatePublish(double distance)
        {
            std::cout << distance<< std::endl;
        }

    };

    class DistanceGazeboRosPluginRos: public DistanceGazeboRosPlugin
    {
    private:
        ros::Publisher pub;
        std_msgs::Float64 msg;
    public:
        virtual void DistanceUpdatePublish(double distance) override
        {
            msg.data = distance;
            pub.publish(msg);
        }
    };
}