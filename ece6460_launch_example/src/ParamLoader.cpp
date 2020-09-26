// Header file for the class
#include "ParamLoader.hpp"

// Namespace matches ROS package name
namespace ece6460_launch_example {

  // Constructor with global and private node handle arguments
  ParamLoader::ParamLoader(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Look up parameter and set to default value if not found
    double float_param1;
    pn.param("param1", float_param1, 5.0);
    ROS_INFO_STREAM("'param1' value: " << float_param1);

    // Look up parameter and return whether it was found or not
    double float_param2;
    bool found_param = pn.getParam("param2", float_param2);
    if (!found_param) {
      ROS_WARN("Did not find 'param2' parameter");
    } else {
      ROS_INFO_STREAM("'param2' value: " << float_param2);
    }

    // Example of a string parameter
    std::string string_param;
    pn.param("param3", string_param, std::string("default_string"));
    ROS_INFO_STREAM("'param3' value: " << string_param);
  }

}