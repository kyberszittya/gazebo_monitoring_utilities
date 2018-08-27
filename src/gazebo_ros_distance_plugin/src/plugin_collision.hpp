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
        // Maps to store
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > monitored_map;
        std::map<std::string, std::shared_ptr<fcl::CollisionObjectd> > target_map;
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
        double getMinTargetDistanceNarrowphase();
        double getMinTargetDistance();
        void addTargetObject(std::string name);
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
        double update_rate;
        common::Time last_update_time;
        std::unique_ptr<DistanceCalculation> distance_calculation;

        std::vector<std::string> model_names;

    protected:
        sdf::ElementPtr sdf;
        physics::WorldPtr world;

        std::thread distance_thread;
        event::ConnectionPtr update_event_;
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
