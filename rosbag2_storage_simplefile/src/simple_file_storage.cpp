
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <unordered_map>

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
constexpr const char MAGIC_SEQUENCE[9] = {'\x89', 'S', 'B', 'A', 'G', '\r', '\n', '\x1a', '\n'};
constexpr const uint8_t VERSION = 1;
constexpr const size_t FOOTER_SIZE = 8;

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
  void init_read();
  void init_write();

  bool open_ = false;
  uint8_t version_ = 0;
  std::string relative_path_;
  std::ofstream out_;
  std::ifstream in_;

  rosbag2_storage::BagMetadata metadata_;
  std::unordered_map<std::string, rosbag2_storage::TopicInformation> topics_;
  rosbag2_storage::MetadataIo metadata_io_;
  std::string tmp_metadata_path_;

  rosbag2_storage::StorageFilter storage_filter_;
};

SimpleFileStorage::SimpleFileStorage()
{
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.message_count = 0;
}

SimpleFileStorage::~SimpleFileStorage()
{
  // Write metadata
  // TODO read from meta.tmp into string, write that out

  // Now write footer

  // Finally, put the magic sequence at the end
  out_.write(MAGIC_SEQUENCE, 9);
  out_.write(reinterpret_cast<char *>(&version_), 1);
  out_.flush();

  // TODO Delete meta.tmp
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
      init_write();
      init_read();
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      relative_path_ = storage_options.uri;
      in_ = std::ifstream(relative_path_, std::ios::binary);
      init_read();
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      relative_path_ = storage_options.uri;
      out_ = std::ofstream(relative_path_, std::ios::binary | std::ios::ate);
      break;
  }

  open_ = true;
  tmp_metadata_path_ = std::filesystem::path{storage_options.uri}.parent_path() / "_meta.tmp";
  metadata_.relative_file_paths = {get_relative_file_path()};
}

void SimpleFileStorage::init_read()
{
  char version_check[10];
  in_.read(version_check, 10);
  if (memcmp(version_check, MAGIC_SEQUENCE, 9) != 0) {
    throw std::runtime_error("Specified file was not a SimpleFileStorage file");
  }
  version_ = version_check[9];
}

void SimpleFileStorage::init_write()
{
  version_ = VERSION;
  out_.write(MAGIC_SEQUENCE, 9);
  out_.write(reinterpret_cast<char *>(&version_), 1);
  out_.flush();

}

/** BaseInfoInterface **/
rosbag2_storage::BagMetadata SimpleFileStorage::get_metadata()
{
  metadata_.topics_with_message_count.clear();
  for (const auto & kv : topics_) {
    metadata_.topics_with_message_count.push_back(kv.second);
  }
  return metadata_;
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
  // TODO filter and meta
  (void)in_.peek();
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
  std::vector<rosbag2_storage::TopicMetadata> out;
  for (const auto & topic : topics_) {
    out.push_back(topic.second.topic_metadata);
  }
  return out;
}

/** ReadOnlyInterface **/
void SimpleFileStorage::set_filter(const rosbag2_storage::StorageFilter & storage_filter)
{
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
  // A minimally sized file will have:
  // - MAGIC_SEQUENCE + VERSION = 10B
  // - No messages = 0B
  // - Minimally sized metadata = 300B
  // (since it is serialized YAML, this is fairly variable, this is a strictly lower bound)
  // - FOOTER = 8B
  // - MAGIC_SEQUENCE + VERSION = 10B
  // 10 + 0 + 300 + 8 + 10 = 328B
  // We will just state 1KB as an arbitrary but small lower limit
  return 1024;
}

/** BaseWriteInterface **/
void SimpleFileStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  // MessageRecord format:
  // - timestamp (8 byte)
  // - topic name size (4 byte)
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

  // Write the pending message to disk
  out_.flush();

  /// Update metadata
  // Increment individual topic message count
  topics_.at(msg->topic_name).message_count++;
  // Increment global message count
  metadata_.message_count++;
  // Determine bag duration. Note: this assumes in-order writes.
  const auto chrono_ts = std::chrono::time_point<std::chrono::high_resolution_clock>(
    std::chrono::nanoseconds(msg->time_stamp));
  metadata_.duration = chrono_ts - metadata_.starting_time;
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
  if (topics_.find(topic.name) == topics_.end()) {
    topics_[topic.name] = rosbag2_storage::TopicInformation{topic, 0};
  }
}

void SimpleFileStorage::remove_topic(const rosbag2_storage::TopicMetadata & topic)
{
  topics_.erase(topic.name);
}

}  // namespace rosbag2_storage_simplefile

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::SimpleFileStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
