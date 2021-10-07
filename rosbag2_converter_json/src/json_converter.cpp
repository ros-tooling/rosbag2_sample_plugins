#include <memory>

#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_storage/ros_helper.hpp"

namespace rosbag2_converter_plugins
{

class JsonConverter
: public rosbag2_cpp::converter_interfaces::SerializationFormatConverter
{
public:
  JsonConverter() = default;

  virtual ~JsonConverter() = default;

  /** SerializationFormatSerializer **/
  void serialize(
    std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message) override;

  /** SerializationFormatDeserializer **/
  void deserialize(
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * type_support,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message) override;
};

void JsonConverter::serialize(
  std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message)
{
  // TODO
  const std::string fake_data{"fake_data"};
  serialized_message->serialized_data = rosbag2_storage::make_serialized_message(
    fake_data.c_str(), fake_data.size());
}

void JsonConverter::deserialize(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * type_support,
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message)
{
  // TODO
}

}  // namespace rosbag2_converter_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_converter_plugins::JsonConverter,
  rosbag2_cpp::converter_interfaces::SerializationFormatConverter)
