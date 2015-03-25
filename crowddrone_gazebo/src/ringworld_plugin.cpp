#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Bool.h>

namespace gazebo {
	class RingVisualPlugin : public ModelPlugin {
	private:
		physics::ModelPtr parent;
transport::NodePtr node;
		event::ConnectionPtr updateConnection;
		ros::NodeHandle* node_handle_;
		ros::CallbackQueue callback_queue_;
		ros::Subscriber light_subscriber_;
	
	public:
		RingVisualPlugin() : ModelPlugin() {
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			//store the visual pointer, so the topic callbacks can access it
			parent = _parent;

			// Make sure the ROS node for Gazebo has already been initialized
			if (!ros::isInitialized()) {
				ROS_FATAL_STREAM("RingVisualPlugin: A ROS node for Gazebo has not been initialized, unable to load plugin. " 
					<< "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      				return;
			}

			//create ros topics
			node_handle_ = new ros::NodeHandle("/ringworld");

			std::string name = parent->GetName();
			name = name.substr(0, name.find(":"));

			ros::SubscribeOptions ops = ros::SubscribeOptions::create<std_msgs::Bool>(
				name+"Light", 1,
				boost::bind(&RingVisualPlugin::setRingLightCallback, this, _1),
				ros::VoidPtr(), &callback_queue_);
			light_subscriber_ = node_handle_->subscribe(ops);

			updateConnection = event::Events::ConnectWorldUpdateBegin(
				boost::bind(&RingVisualPlugin::update, this));

		}

		void update() {
			callback_queue_.callAvailable();
		}

		void setRingLightCallback(const std_msgs::BoolConstPtr& b) {
	transport::PublisherPtr visPub;
      msgs::Visual visualMsg;

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(parent->GetWorld()->GetName());
      visPub = this->node->Advertise<msgs::Visual>("~/visual", 10);

      // Set the visual's name. This should be unique.
      visualMsg.set_name("__RED_CYLINDER_VISUAL__");

      // Set the visual's parent. This visual will be attached to the parent
      visualMsg.set_parent_name(parent->GetScopedName());


      // Set the material to be bright red
      visualMsg.mutable_material()->mutable_diffuse()->set_r(0.0);
      visualMsg.mutable_material()->mutable_diffuse()->set_g(1.0);
      visualMsg.mutable_material()->mutable_diffuse()->set_b(0.0);
      visualMsg.mutable_material()->mutable_ambient()->set_r(0.0);
      visualMsg.mutable_material()->mutable_ambient()->set_g(1.0);
      visualMsg.mutable_material()->mutable_ambient()->set_b(0.0);

      // Don't cast shadows
      visualMsg.set_cast_shadows(false);

      visPub->Publish(visualMsg);

			common::Color c(0.0, 0.0, 0.0);
			if (b->data) {
				ROS_INFO("TRUE");
				c = common::Color(0.0, 1.0, 0.0);
			} else {
				ROS_INFO("FALSE");
				c = common::Color(1.0, 0.0, 0.0);
			}

		//	parent->SetAmbient(c);
		//	parent->SetDiffuse(c);
		}

	};

	GZ_REGISTER_MODEL_PLUGIN(RingVisualPlugin)
}
