#include "rosbag2_storage_simplefile/simple_file_storage.hpp"

#include "rcutils/logging_macros.h"

namespace rosbag2_storage_plugins
{

SimpleFileStorage::SimpleFileStorage()
{}

SimpleFileStorage::~SimpleFileStorage()
{}

/** BaseIOInterface **/
void SimpleFileStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  out_ = std::ofstream(storage_options.uri, std::ios::binary);
}

/** BaseIOInterface **/
rosbag2_storage::BagMetadata SimpleFileStorage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata;
  return metadata;
}

std::string SimpleFileStorage::get_relative_file_path() const
{
  return "";
}

uint64_t SimpleFileStorage::get_bagfile_size() const
{
  return 0u;
}

std::string SimpleFileStorage::get_storage_identifier() const
{
  return "simplefile";
}

/** BaseReadInterface **/
bool SimpleFileStorage::has_next()
{
  return false;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SimpleFileStorage::read_next()
{
  return nullptr;
}

std::vector<rosbag2_storage::TopicMetadata> SimpleFileStorage::get_all_topics_and_types()
{
  return {};
}

/** ReadOnlyInterface **/
void SimpleFileStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{}

void SimpleFileStorage::reset_filter()
{}

void SimpleFileStorage::seek(const rcutils_time_point_value_t & timestamp)
{}

/** ReadWriteInterface **/
uint64_t SimpleFileStorage::get_minimum_split_file_size() const
{
  return 0u;
}

/** BaseWriteInterface **/
void SimpleFileStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  static const size_t size_size = sizeof(uint64_t);
  static const uint64_t zero_pad = 0;

  // Write order: size, timestamp, topic (null-terminated), buffer, alignment padding

  // size
  uint64_t total_size = size_size;
  // timestamp
  total_size += size_size;
  // topic
  total_size += msg->topic_name.size();
  total_size += 1;
  // buffer
  // total_size += msg->serialized_data->buffer_length;

  // 64-bit alignment padding for the next message
  char padding = size_size - (total_size % size_size);
  RCUTILS_LOG_INFO("Writing %zu, pad %d", total_size, padding);
  total_size += padding;

  // size
  out_.write(reinterpret_cast<const char*>(&total_size), size_size);
  // timestamp
  out_.write(reinterpret_cast<const char*>(&msg->time_stamp), size_size);
  // topic
  out_.write(msg->topic_name.c_str(), msg->topic_name.size());
  out_.write(reinterpret_cast<const char*>(&zero_pad), 1);
  // buffer
  // out_.write(
  //   reinterpret_cast<char*>(msg->serialized_data->buffer),
  //   msg->serialized_data->buffer_length);
  // alignment padding
  out_.write(reinterpret_cast<const char*>(&zero_pad), padding);
  out_.flush();
}

void SimpleFileStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs)
{
  for (const auto & msg : msgs) {
    write(msg);
  }
}

void SimpleFileStorage::create_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // TODO
}

void SimpleFileStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  // TODO
}

}  // namespace rosbag2_storage_simplefile
