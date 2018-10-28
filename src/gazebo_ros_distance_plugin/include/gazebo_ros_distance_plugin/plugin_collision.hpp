#ifndef PLUGIN_COLLISION_HPP
#define PLUGIN_COLLISION_HPP

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
    struct DistanceMeas
    {
        double distance;
        std::string name;
    };

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

    bool defaultDistanceFunction(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* cdata_, double& dist);


    class DistanceCalculation
    {
    private:
        unsigned int object_cnt;
        std::map<std::string, unsigned int> objects;
        std::vector<std::shared_ptr<fcl::CollisionObjectd> > objects_collision_mesh;
        std::vector<Eigen::Vector3d> objects_collision_center;
        // Broadphase sets of different object types
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> dynamic_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> static_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> target_object_set;
        std::unique_ptr<fcl::BroadPhaseCollisionManagerd> monitored_object_set;
        
        // Set of ignored objects to ensure that they never will be selected
        std::set<std::string> ignored_objects;
        // Set of target object properties to ensure uniqueness
        std::set<unsigned int> target_objects;
        std::set<std::string> target_objects_name;
        // FCL related fields
        fcl::DistanceRequestd dist_request;

        
        // Initialize ignored object set
        void initializeObjectSet();
        void pushCollisionObjectDynamic(
            const std::string name,
            std::shared_ptr<fcl::CollisionObjectd> objd,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center
            );
        void pushCollisionObjectStatic(
            const std::string name,
            std::shared_ptr<fcl::CollisionObjectd> objd,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center
            );
        void pushCollisionObjectTarget(
            const std::string name,
            std::shared_ptr<fcl::CollisionObjectd> objd,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center
            );
    protected:
        
    public:
        // Maps to store
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > monitored_map;
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > target_map;
        DistanceCalculation():
            object_cnt(0),            
            dynamic_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            static_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            target_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd()),
            monitored_object_set(
                new fcl::DynamicAABBTreeCollisionManagerd())
        {
            initializeObjectSet();
            std::cout << "Setup collision plugin" << std::endl;
            dist_request.enable_nearest_points = true;
            dist_request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        }
        bool isIgnoredObject(std::string name);
        bool isTargetObject(std::string name);
        bool isTargetObject(unsigned int id);

        void setupCollision();
        void updateObjectPose(std::string name, Eigen::Vector3d t, Eigen::Quaterniond r);
        void updateObjectPose(unsigned int i, Eigen::Vector3d t, Eigen::Quaterniond r);
        void updateBoundingBox(std::string name, Eigen::Vector3d t, Eigen::Vector3d r);
        double getMinTargetDistanceNarrowphase();
        DistanceMeas getMinTargetDistance();
        
        void addTargetObject(std::string name);
        void addIgnoredObject(std::string name);
        void addCylinderCollisionMeshTarget(std::string name,
            const Eigen::Vector2d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addSphereCollisionMeshTarget(std::string name,
            const double& radius,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addBoxCollisionMeshTarget(std::string name,
            const Eigen::Vector3d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addCylinderCollisionMeshStatic(std::string name,
            const Eigen::Vector2d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addSphereCollisionMeshStatic(std::string name,
            const double& radius,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addBoxCollisionMeshStatic(std::string name,
            const Eigen::Vector3d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addCylinderCollisionMeshDynamic(std::string name,
            const Eigen::Vector2d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addSphereCollisionMeshDynamic(std::string name,
            const double& radius,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
        void addBoxCollisionMeshDynamic(
            const std::string name,
            const Eigen::Vector3d& size,
            const Eigen::Vector3d& position,
            const Eigen::Quaterniond& orientation,
            const Eigen::Vector3d& center);
    };

    class DistanceGazeboPlugin: public WorldPlugin
    {
    private:
        common::Time last_update_time;
        std::vector<std::string> model_names;
    protected:
        double update_rate;
        sdf::ElementPtr sdf;
        physics::WorldPtr world;

        std::thread distance_thread;
        event::ConnectionPtr update_event_;
        std::unique_ptr<DistanceCalculation> distance_calculation;
        
    public:
        DistanceGazeboPlugin(): WorldPlugin()
        {
            
            
        }
        
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
        virtual void DistanceUpdateThread();
        virtual void Update();

        void Init()
        {
            update_event_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DistanceGazeboPlugin::Update, this));
        }

        virtual void DistanceUpdatePublish(double distance)
        {
            std::cout << distance << std::endl;
        }

        virtual void InitPlugin(physics::WorldPtr _world, sdf::ElementPtr _sdf);

        // Get minimal distance between targets and monitored objects
        DistanceMeas getMinTargetDistanceBoundingBox()
        {
            
            gazebo::DistanceMeas res_m;
            res_m.distance = 0.0;
            res_m.name = "";
            fcl::DistanceRequestd dist_request;
            dist_request.enable_nearest_points = true;
            dist_request.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
            double min_dist = std::numeric_limits<double>::max();
            auto R = fcl::Matrix3d::Identity();
            for (auto it_target = distance_calculation->target_map.begin(); 
                it_target != distance_calculation->target_map.end(); ++it_target)
            {
                auto t_obj = world->ModelByName(it_target->first);
                auto target_bbox = t_obj->CollisionBoundingBox();
                std::shared_ptr<fcl::Boxd> targetbox(
                    new fcl::Boxd(target_bbox.Size().X(), target_bbox.Size().Y(), target_bbox.Size().Z())
                );
                fcl::Vector3d T_t(target_bbox.Center().X(), 
                    target_bbox.Center().Y(), 
                    target_bbox.Center().Z()
                );
                fcl::CollisionObjectd to(targetbox, R, T_t);
                for (auto it_monitored = distance_calculation->monitored_map.begin(); 
                    it_monitored != distance_calculation->monitored_map.end(); ++it_monitored)
                {
                    
                    auto m_obj = world->ModelByName(it_monitored->first);
                    auto monitor_bbox = m_obj->CollisionBoundingBox();
                    std::shared_ptr<fcl::Boxd> monitorbox(
                        new fcl::Boxd(monitor_bbox.Size().X(), monitor_bbox.Size().Y(), monitor_bbox.Size().Z())
                    );
                    fcl::Vector3d T_m(monitor_bbox.Center().X(), monitor_bbox.Center().Y(), monitor_bbox.Center().Z());
                    fcl::CollisionObjectd mo(monitorbox, R, T_m);
                    fcl::DistanceResultd dist_result;
                    double res = fcl::distance(&to, &mo, dist_request, dist_result);
                    /*
                    std::cout << it_monitored->first << std::endl;
                    
                    std::cout << res << std::endl;
                    
                    std::cout << it_target->first << '\t' 
                        << target_bbox.Center().X() << '\t'
                        << target_bbox.Center().Y() << '\t'
                        << target_bbox.Center().Z() 
                        << "Size" << '\t'
                        <<  target_bbox.Size().X() << '\t' 
                        <<  target_bbox.Size().Y() << '\t' 
                        <<  target_bbox.Size().Z() << '\t' 
                        << std::endl;
                    std::cout << it_monitored->first << '\t' 
                            << monitor_bbox.Center().X() << '\t'
                            << monitor_bbox.Center().Y() << '\t'
                            << monitor_bbox.Center().Z() << '\t' 
                            << "Size" << '\t'
                            <<  monitor_bbox.Size().X() << '\t' 
                            <<  monitor_bbox.Size().Y() << '\t' 
                            <<  monitor_bbox.Size().Z() << '\t' 
                            << std::endl << std::endl;
                    */
                    if (min_dist > res)
                    {
                        min_dist = res;
                        res_m.distance = min_dist;
                        res_m.name = it_monitored->first;
                    }
                    
                    if (min_dist < 0.0)
                    {
                        return res_m;
                    }
                    

                }

            }
            return res_m;
        }
    };

    class DistanceGazeboPluginRos: public DistanceGazeboPlugin
    {
    private:
        ros::Publisher pub;
        std_msgs::Float64 msg;
        std::shared_ptr<ros::NodeHandle> nh;
    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;
        void Init() override
        {
            int argc = 0;
            char* argv = nullptr;
            ros::init(argc, &argv, "GazeboDistancePluginRos");
            nh.reset(new ros::NodeHandle());
            pub = nh->advertise<std_msgs::Float64>("/gazebo/target_object/distance",100);
            update_event_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&DistanceGazeboPlugin::Update, this));
        }
        virtual void DistanceUpdatePublish(double distance) override
        {
            msg.data = distance;
            pub.publish(msg);
        }
        
        
    };

    
}

#endif
