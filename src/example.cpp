#include <rcl/rcl.h>
#include <rclcpp/rclcpp.hpp>
#include <message_conversion_json/message_parser.hpp>


int main(int argc, char* argv[]) {
  // Initialise the options for ROS
  rcl_init_options_t options = rcl_get_zero_initialized_init_options();
  rcl_ret_t ret = rcl_init_options_init(&options, rcl_get_default_allocator());
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("init options failed");
    return EXIT_FAILURE;
  }

  // Initialise ROS itself
  rcl_context_t context = rcl_get_zero_initialized_context();
  ret = rcl_init(argc, argv, &options, &context);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("init failed");
    return EXIT_FAILURE;
  }
  ret = rcl_init_options_fini(&options);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("init_options fini failed");
    return EXIT_FAILURE;
  }

  // Create a node to get access to the ROS graph, topics, etc.
  RCUTILS_LOG_DEBUG_NAMED("cli-tool", "Creating node");
  rcl_node_options_t node_options = rcl_node_get_default_options();
  rcl_node_t node = rcl_get_zero_initialized_node();
  ret = rcl_node_init(&node, "clitool", "", &context, &node_options);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("node init failed");
    return EXIT_FAILURE;
  }

  rcl_publisher_t pub = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t pub_options = rcl_publisher_get_default_options();

  const std::string topic_type = "geometry_msgs/Pose";
  const std::string topic = "/pose_topic";
  const auto poseMsgString =
      "{ \
    \"position\": { \
      \"x\": 1.0, \
      \"y\": 2.0, \
      \"z\": 3.0 \
    }, \"orientation\": { \
      \"x\": 1.0, \
      \"y\": 2.0, \
      \"z\": 3.0, \
      \"w\": 4.0 \
    } \
  }";

  auto ts_lib = rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_cpp");
  auto type_support = rclcpp::get_typesupport_handle(topic_type, "rosidl_typesupport_cpp", *ts_lib);
  auto type_support_introspection_lib =
      rclcpp::get_typesupport_library(topic_type, "rosidl_typesupport_introspection_cpp");

  if (!type_support || !type_support_introspection_lib) {
    RCUTILS_LOG_ERROR("Failed to load type support");
    return EXIT_FAILURE;
  }

  auto allocator = rcl_get_default_allocator();
  auto type_info = message_conversion_json::getTypeInfo(topic_type, type_support_introspection_lib);
  auto msg = message_conversion_json::toRosMsg(type_info, poseMsgString, &allocator);

  ret = rcl_publisher_init(&pub, &node, type_support, topic.c_str(), &pub_options);
  if (ret != RCL_RET_OK) {
    RCUTILS_LOG_ERROR("subscription init failed");
    return EXIT_FAILURE;
  }

  for (auto ii = 0; ii < 10; ++ii) {
    RCUTILS_LOG_INFO("Publishing");
    ret = rcl_publish(&pub, msg.data, nullptr);
    if (ret != RCL_RET_OK) {
      return EXIT_FAILURE;
    }
    sleep(1);
  }

  ret = rcl_publisher_fini(&pub, &node);
  if (ret != RCL_RET_OK) {
    return EXIT_FAILURE;
  }
}
