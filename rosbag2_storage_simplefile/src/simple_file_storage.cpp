
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>

#include "rcutils/logging_macros.h"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

namespace
{
// void debugPrintHexArray(const unsigned char* data, size_t len)
// {
//   std::ios_base::fmtflags f( std::cout.flags() );
//   for (size_t i = 0; i < len; ++i)
//       std::cout << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (((int)data[i]) & 0xFF) << " ";
//   std::cout << std::endl;
//   std::cout.flags( f );
// }

}  // namespace

namespace rosbag2_storage_plugins
{

constexpr const auto FILE_EXTENSION = ".simple";

class SimpleFileStorage
  : public rosbag2_storage::storage_interfaces::ReadWriteInterface
{
public:
  SimpleFileStorage();

  virtual ~SimpleFileStorage();

  // BaseIOInterface
  void open(
    const rosbag2_storage::StorageOptions & storage_options,
    rosbag2_storage::storage_interfaces::IOFlag io_flag =
    rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) override;

  // BaseInfoInterface
  rosbag2_storage::BagMetadata get_metadata() override;
  std::string get_relative_file_path() const override;
  uint64_t get_bagfile_size() const override;
  std::string get_storage_identifier() const override;

  // BaseReadInterface
  bool has_next() override;
  std::shared_ptr<rosbag2_storage::SerializedBagMessage> read_next() override;
  std::vector<rosbag2_storage::TopicMetadata> get_all_topics_and_types() override;

  // ReadOnlyInterface
  void set_filter(const rosbag2_storage::StorageFilter & storage_filter) override;
  void reset_filter() override;
  void seek(const rcutils_time_point_value_t & timestamp) override;

  // ReadWriteInterface
  uint64_t get_minimum_split_file_size() const override;

  // BaseWriteInterface
  void write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg) override;
  void write(
    const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msg)
  override;
  void create_topic(const rosbag2_storage::TopicMetadata & topic) override;
  void remove_topic(const rosbag2_storage::TopicMetadata & topic) override;

private:
  std::string relative_path_;
  std::ofstream out_;
  std::ifstream in_;
  rosbag2_storage::BagMetadata metadata_;
  rosbag2_storage::MetadataIo metadata_io_;
  rosbag2_storage::StorageFilter storage_filter_;
};

SimpleFileStorage::SimpleFileStorage()
{}

SimpleFileStorage::~SimpleFileStorage()
{
  // TODO shutdown cleanup ("index"?) - metadata stuff
}

/** BaseIOInterface **/
void SimpleFileStorage::open(
  const rosbag2_storage::StorageOptions & storage_options,
  rosbag2_storage::storage_interfaces::IOFlag io_flag)
{
  switch (io_flag) {
    case rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE:
      relative_path_ = storage_options.uri + FILE_EXTENSION;
      out_ = std::ofstream(relative_path_, std::ios::binary);
      in_ = std::ifstream(relative_path_, std::ios::binary);
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      relative_path_ = storage_options.uri;
      in_ = std::ifstream(relative_path_, std::ios::binary);
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      relative_path_ = storage_options.uri;
      out_ = std::ofstream(relative_path_, std::ios::binary | std::ios::ate);
      break;
  }
}

/** BaseInfoInterface **/
rosbag2_storage::BagMetadata SimpleFileStorage::get_metadata()
{
  rosbag2_storage::BagMetadata metadata;
  metadata.storage_identifier = get_storage_identifier();
  metadata.relative_file_paths = {get_relative_file_path()};
  metadata.message_count = 0;
  metadata.topics_with_message_count = {};
  // TODO how is metadata stored / fetched
  // metadata.message_count++;
  // metadata.topics_with_message_count.push_back();
  // metadata.starting_time = ??;
  // metadata.duration = ??;
  metadata.bag_size = get_bagfile_size();
  return metadata;
}

std::string SimpleFileStorage::get_relative_file_path() const
{
  return relative_path_;
}

uint64_t SimpleFileStorage::get_bagfile_size() const
{
  return std::filesystem::file_size(relative_path_);
}

std::string SimpleFileStorage::get_storage_identifier() const
{
  return "simplefile";
}

/** BaseReadInterface **/
bool SimpleFileStorage::has_next()
{
  // TODO this changes if I put metadata block at end
  return !in_.eof();
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SimpleFileStorage::read_next()
{
  // TODO filter
  uint64_t time_stamp = 0;
  uint32_t topic_size = 0;
  uint32_t data_size = 0;

  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  in_.read(reinterpret_cast<char *>(&time_stamp), sizeof(time_stamp));
  msg->time_stamp = time_stamp;

  in_.read(reinterpret_cast<char *>(&topic_size), sizeof(topic_size));
  msg->topic_name.resize(topic_size);
  in_.read(&msg->topic_name[0], topic_size);

  in_.read(reinterpret_cast<char *>(&data_size), sizeof(data_size));
  msg->serialized_data = rosbag2_storage::make_empty_serialized_message(data_size);
  in_.read(reinterpret_cast<char *>(msg->serialized_data->buffer), data_size);
  msg->serialized_data->buffer_length = data_size;

  return msg;
}

std::vector<rosbag2_storage::TopicMetadata> SimpleFileStorage::get_all_topics_and_types()
{
  // TODO
  return {};
}

/** ReadOnlyInterface **/
void SimpleFileStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
  // TODO is this enough?
  storage_filter_ = storage_filter;
}

void SimpleFileStorage::reset_filter()
{
  set_filter(rosbag2_storage::StorageFilter());
}

void SimpleFileStorage::seek(const rcutils_time_point_value_t &)
{
  // TODO
}

/** ReadWriteInterface **/
uint64_t SimpleFileStorage::get_minimum_split_file_size() const
{
  // NOTE: unlike SQLite3 there is no minimum size for this file type.
  // In practice, the minimum size is the size of an empty BagMetadata, which would be
  // written to the end of a file with no messages.
  // TODO - note about header
  return 0u;
}

/** BaseWriteInterface **/
void SimpleFileStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  // Write format:
  // - timestamp (8 byte)
  // - topic size (4 byte)
  // - topic name (variable)
  // - data size (4 byte)
  // - data (variable)
  uint64_t time_stamp = msg->time_stamp;
  uint32_t topic_size = msg->topic_name.size();
  uint32_t data_size = msg->serialized_data->buffer_length;

  out_.write(reinterpret_cast<const char *>(&time_stamp), sizeof(time_stamp));
  out_.write(reinterpret_cast<const char *>(&topic_size), sizeof(topic_size));
  out_.write(msg->topic_name.c_str(), topic_size);
  out_.write(reinterpret_cast<const char *>(&data_size), 4);
  out_.write(reinterpret_cast<const char *>(msg->serialized_data->buffer), data_size);

  // Write the whole message to disk
  out_.flush();
}

void SimpleFileStorage::write(
  const std::vector<std::shared_ptr<const rosbag2_storage::SerializedBagMessage>> & msgs)
{
  for (const auto & msg : msgs) {
    write(msg);
  }
}

void SimpleFileStorage::create_topic(const rosbag2_storage::TopicMetadata &)
{
  // TODO metadata stuff
}

void SimpleFileStorage::remove_topic(const rosbag2_storage::TopicMetadata &)
{
  // NOTE: ignoring this for this sample - there is no harm in the dangling metadata, even
  // if no messages are written for it. This API is not used in default rosbag2 behavior.
}

}  // namespace rosbag2_storage_simplefile

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::SimpleFileStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
