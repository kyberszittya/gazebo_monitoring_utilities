#include <ignition/math/Pose3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include <fcl/narrowphase/collision_object.h>
#include <fcl/narrowphase/distance.h>

#include <ros/ros.h>

#include <std_msgs/Float64.h>

namespace gazebo
{

struct ObjectNode
{
	physics::ModelPtr model;
	std::shared_ptr<fcl::CollisionObjectd> obj;

	ObjectNode(physics::ModelPtr _model, std::shared_ptr<fcl::CollisionObjectd> obj):
		model(_model), obj(obj) {}
};

class ReiDistancePlugin: public WorldPlugin
{
private:
	physics::WorldPtr parent;
	std::shared_ptr<ObjectNode> sut_obj;
	std::map<std::string, std::shared_ptr<ObjectNode>> dyn_objs;
	event::ConnectionPtr updateConnection;
	std::string sut_name;
	ros::Timer timer_update;
	ros::Publisher pub_distance;
	std_msgs::Float64 msg_distance;
	ros::NodeHandle nh;
public:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
    	if (!_sdf->HasElement("dyn_obj") || !_sdf->HasElement("sut_obj"))
    	{
    		std::cerr << "No dynamic objects defined\n";
    	}
    	else
    	{
    		parent = _parent;
    		sut_name = _sdf->Get<std::string>("sut_obj");
    		auto sdf_dyn_obj = _sdf->GetElement("dyn_obj");
    		std::cout << "SUT name: " << sut_name << '\n';
    		sut_obj = nullptr;
    		for (sdf::ElementPtr p0= sdf_dyn_obj->GetFirstElement(); p0!=nullptr; p0=sdf_dyn_obj->GetNextElement("name"))
    		{
    			std::string name = p0->GetValue()->GetAsString();
    			std::cout << "Adding dynamic object: " << name << '\n';
    			auto p = _parent->ModelByName(name);
    			if (p != nullptr)
    			{
    				auto b = p->BoundingBox();
    				dyn_objs.insert(std::pair<std::string, std::shared_ptr<ObjectNode>>(
    						name, std::make_shared<ObjectNode>(
    								p,
    								std::make_shared<fcl::CollisionObjectd>(
    										std::make_shared<fcl::Boxd>(b.XLength(), b.YLength(), b.ZLength())
							)))
					);
    			}
    			else
    			{
    				std::cerr << "Invalid dynamic object: " << name << '\n';
    			}
    		}
    		std::cout << "Loaded distance plugin\n";
    		updateConnection = event::Events::ConnectWorldUpdateBegin(
    				std::bind(&ReiDistancePlugin::Update, this)
    		);

    		int argc = 0;
			char **argv = NULL;
			ros::init(argc, argv, "gazebo_distance_plugin",
				ros::init_options::NoSigintHandler);
			nh = ros::NodeHandle();
			pub_distance = nh.advertise<std_msgs::Float64>("/gazebo_sut_distance/distance", 10);
			timer_update = nh.createTimer(ros::Duration(0.1), &ReiDistancePlugin::cbTimerDistanceUpdate, this);
			timer_update.start();
    	}
    }

    void cbTimerDistanceUpdate(const ros::TimerEvent& e)
    {
    	if (sut_obj==nullptr)
    	{
    		auto p = parent->ModelByName(sut_name);
			if (p != nullptr)
			{
				auto b = p->BoundingBox();
				sut_obj = std::make_shared<ObjectNode>(
					p, std::make_shared<fcl::CollisionObjectd>(
							std::make_shared<fcl::Boxd>(b.XLength(), b.YLength(), b.ZLength()
						)
					)
				);
				std::cout << "Added SUT object to collision detection\n";
			}
    	}
    	else
    	{
    		fcl::DistanceRequestd request;
    		fcl::DistanceResultd result;
    		double distance = std::numeric_limits<double>::max();
    		for (const auto kv: dyn_objs)
    		{
    			fcl::distance(sut_obj->obj.get(), kv.second->obj.get(), request, result);
    			distance = std::min(result.min_distance, distance);
    		}
    		msg_distance.data = distance;
    		pub_distance.publish(msg_distance);

    	}
    }

    static void setTransformation(std::shared_ptr<ObjectNode> obj)
    {
    	fcl::Transform3d tr;
		tr.translation() = fcl::Vector3d(
				obj->model->WorldPose().Pos().X(),
				obj->model->WorldPose().Pos().Y(),
				obj->model->WorldPose().Pos().Z()
		);
		tr.linear() = fcl::Quaterniond(
				obj->model->WorldPose().Rot().W(),
				obj->model->WorldPose().Rot().X(),
				obj->model->WorldPose().Rot().Y(),
				obj->model->WorldPose().Rot().Z()
		).toRotationMatrix();
		obj->obj->setTransform(std::move(tr));
    }


    void Update()
    {
    	if (sut_obj!=nullptr)
    	{
    		setTransformation(sut_obj);

    	}
    	for (const auto kv: dyn_objs)
    	{
    		setTransformation(kv.second);
    	}
    }
};
GZ_REGISTER_WORLD_PLUGIN(ReiDistancePlugin)

}
