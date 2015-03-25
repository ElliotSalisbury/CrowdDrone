#include "ring_visual_plugin.h"

namespace gazebo
{
  namespace rendering
  {

    ////////////////////////////////////////////////////////////////////////////////
    // Constructor
    RingVisualPlugin::RingVisualPlugin() {

    }

    ////////////////////////////////////////////////////////////////////////////////
    // Destructor
    RingVisualPlugin::~RingVisualPlugin()
    {
      // Finalize the visualizer
      this->rosnode_->shutdown();
      delete this->rosnode_;
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Load the plugin
    void RingVisualPlugin::Load( VisualPtr _parent, sdf::ElementPtr _sdf )
    {
      this->visual_ = _parent;

      this->model_name_ = this->visual_->GetName();
      this->model_name_ = this->model_name_.substr(0,this->model_name_.find(":"));

      this->visual_namespace_ = "/"+this->model_name_;

      // start ros node
      if (!ros::isInitialized())
      {
        int argc = 0;
        char** argv = NULL;
        ros::init(argc,argv,"gazebo_visual",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
      }

      this->rosnode_ = new ros::NodeHandle(this->visual_namespace_);
      this->light_sub_ = this->rosnode_->subscribe("light", 1000, &RingVisualPlugin::SetLight, this);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->update_connection_ = event::Events::ConnectRender(
          boost::bind(&RingVisualPlugin::UpdateChild, this));
    }

    //////////////////////////////////////////////////////////////////////////////////
    // Update the visualizer
    void RingVisualPlugin::UpdateChild()
    {
      ros::spinOnce();
    }

    //////////////////////////////////////////////////////////////////////////////////
    // VisualizeForceOnLink
    void RingVisualPlugin::SetLight(const std_msgs::BoolConstPtr &bool_msg)
    {
      std::string materialStr = "Gazebo/Purple";
      if (bool_msg->data) {
        materialStr = "Gazebo/Green";
      } else {
        materialStr = "Gazebo/Red";
      }
      // set the Material of the line, in this case to purple
      this->visual_->SetMaterial(materialStr);
      this->visual_->SetVisibilityFlags(GZ_VISIBILITY_ALL);
      this->visual_->SetVisible(true);
    }

    // Register this plugin within the simulator
    GZ_REGISTER_VISUAL_PLUGIN(RingVisualPlugin)
  }
}
