// Copyright 2021, Amazon.com Inc or its Affiliates. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <memory>
#include <string>

#include "dynmsg/typesupport.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/message_reading.hpp"
#include "dynmsg/yaml_utils.hpp"
#include "rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rcutils/logging_macros.h"

namespace rosbag2_converter_plugins
{

class YamlConverter
  : public rosbag2_cpp::converter_interfaces::SerializationFormatConverter
{
public:
  YamlConverter() = default;
  virtual ~YamlConverter() = default;

  /** SerializationFormatSerializer **/
  void serialize(
    std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
    const rosidl_message_type_support_t * introspection_type_support,
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message) override;

  /** SerializationFormatDeserializer **/
  void deserialize(
    std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
    const rosidl_message_type_support_t * introspection_type_support,
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message) override;
};

void YamlConverter::serialize(
  std::shared_ptr<const rosbag2_cpp::rosbag2_introspection_message_t> ros_message,
  const rosidl_message_type_support_t * introspection_type_support,
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message)
{
  // Reinterpret the opaque rosidl_message_type_support_t object into usable introspection type.
  RosMessage_Cpp ros_msg;
  const TypeInfo_Cpp * type_info = reinterpret_cast<const TypeInfo_Cpp *>(
    introspection_type_support->data);
  ros_msg.type_info = type_info;
  ros_msg.data = reinterpret_cast<uint8_t *>(ros_message->message);

  // Use dynmsg conversion to convert to YAML and dump as a string to the serialized message.
  YAML::Node yaml_msg = dynmsg::cpp::message_to_yaml(ros_msg);
  const std::string yaml_string = dynmsg::yaml_to_string(yaml_msg);
  serialized_message->serialized_data = rosbag2_storage::make_serialized_message(
    yaml_string.c_str(), yaml_string.size());
}

void YamlConverter::deserialize(
  std::shared_ptr<const rosbag2_storage::SerializedBagMessage> serialized_message,
  const rosidl_message_type_support_t * introspection_type_support,
  std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_message)
{
  // Reinterpret the opaque rosidl_message_type_support_t object into usable introspection type.
  const TypeInfo_Cpp * type_info = reinterpret_cast<const TypeInfo_Cpp *>(
    introspection_type_support->data);

  // Load serialized string value from the message
  const std::string yaml_string{
    reinterpret_cast<char *>(serialized_message->serialized_data->buffer),
    serialized_message->serialized_data->buffer_length};

  // Use dynmsg to load the serialized YAML as ROS message data
  RosMessage_Cpp ros_msg = dynmsg::cpp::yaml_to_rosmsg_typeinfo(
    type_info, yaml_string, &ros_message->allocator);
  ros_message->message = ros_msg.data;
}

}  // namespace rosbag2_converter_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_converter_plugins::YamlConverter,
  rosbag2_cpp::converter_interfaces::SerializationFormatConverter)
