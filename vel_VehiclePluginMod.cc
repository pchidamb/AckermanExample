#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
	while(1)
	{
  // Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::setupClient(_argc, _argv);
#else
  gazebo::client::setup(_argc, _argv);
#endif

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to the  velodyne topic
  gazebo::transport::PublisherPtr pub1 =
    node->Advertise<gazebo::msgs::Vector3d>("~/Utility_cart_10kg/gas_joint");

  gazebo::transport::PublisherPtr pub2 =
    node->Advertise<gazebo::msgs::Vector3d>("~/Utility_cart_10kg/steering_joint");

  // Wait for a subscriber to connect to this publisher
  pub1->WaitForConnection();
  pub2->WaitForConnection();
 
  // Create a a vector3 message
  gazebo::msgs::Vector3d msg1;
  gazebo::msgs::Vector3d msg2;

  // Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::msgs::Set(&msg1, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
  gazebo::msgs::Set(&msg2, gazebo::math::Vector3(std::atof(_argv[2]), 0, 0));
#else
  gazebo::msgs::Set(&msg1, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
  gazebo::msgs::Set(&msg2, ignition::math::Vector3d(std::atof(_argv[2]), 0, 0));
#endif

  // Send the message
  pub1->Publish(msg1);
  pub2->Publish(msg2);

  // Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
  gazebo::shutdown();
#else
  gazebo::client::shutdown();
#endif
}
}
