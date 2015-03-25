#ifndef RING_VISUAL_PLUGIN_H
#define RING_VISUAL_PLUGIN_H

#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/MessageTypes.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"

//#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <ros/ros.h>

#include <std_msgs/Bool.h>
// if you want some positions of the model use this....
#include <gazebo_msgs/ModelStates.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>


namespace gazebo
{
  namespace rendering
  {
    class RingVisualPlugin : public VisualPlugin
    {
      public:
        /// \brief Constructor
        RingVisualPlugin();

        /// \brief Destructor
        virtual ~RingVisualPlugin();

        /// \brief Load the visual force plugin tags
        /// \param node XML config node
        void Load( VisualPtr _parent, sdf::ElementPtr _sdf );


      protected: 
        /// \brief Update the visual plugin
        virtual void UpdateChild();


      private:
        /// \brief pointer to ros node
        ros::NodeHandle* rosnode_;

        /// \brief store model name
        std::string model_name_;

        // /// \brief The visual pointer used to visualize the force.
        VisualPtr visual_;

        /// \brief for setting ROS name space
        std::string visual_namespace_;

        /// \Subscribe to some force
        ros::Subscriber light_sub_;

        /// \brief Visualize the force
        void SetLight(const std_msgs::BoolConstPtr &bool_ms);

        // Pointer to the update event connection
        event::ConnectionPtr update_connection_;
    };
  }
}

#endif
