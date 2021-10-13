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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <memory>
#include <optional>  // NOLINT - cppcheck thinks this is C stdlib for some reason
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcutils/logging_macros.h"

#include "rosbag2_storage/metadata_io.hpp"
#include "rosbag2_storage/ros_helper.hpp"
#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

namespace rosbag2_storage_plugins
{

enum RecordType : uint8_t
{
  MESSAGE = 0x01,
  METADATA = 0x02,
  FOOTER = 0x03,
  MAGIC = 0x89,
};
constexpr const auto FILE_EXTENSION = ".simple";
constexpr const char MAGIC_SEQUENCE[9] = {
  static_cast<char>(RecordType::MAGIC), 'S', 'B', 'A', 'G', '\r', '\n', '\x1a', '\n'};
constexpr const uint8_t VERSION = 1;
constexpr const size_t FOOTER_SIZE = 8;

/**
 * A storage implementation for a very simple binary rosbag2 file format.
 *
 * File goes like:
 * MAGIC_SEQUENCE (10B - a 9 byte identifier sequence and one byte version number)
 * MESSAGE RECORDS (byte 0x01, followed by a message record)
 * METADATA (byte 0x02, followed by YAML serialized string of the bag metadata)
 * FOOTER (byte 0x03, followed by 8 byte size of the metadata - to be used as a relative offset)
 * MAGIC_SEQUENCE (10B)
 *
 * For the sake of simplicity for this demonstration,
 * this format is not crash-resistant - without a clean shutdown
 * the data will not be recoverable because metadata is written at the end.
 */
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
  bool read_and_enqueue_message();

  std::optional<rosbag2_storage::storage_interfaces::IOFlag> open_as_;
  uint8_t version_ = 0;
  std::string relative_path_;
  std::ofstream out_;
  std::ifstream in_;

  std::shared_ptr<rosbag2_storage::SerializedBagMessage> next_;

  rosbag2_storage::BagMetadata metadata_;
  std::unordered_map<std::string, rosbag2_storage::TopicInformation> topics_;
  rosbag2_storage::MetadataIo metadata_io_;

  rosbag2_storage::StorageFilter storage_filter_;
  std::unordered_set<std::string> filter_topics_;
};

SimpleFileStorage::SimpleFileStorage()
{
  metadata_.storage_identifier = get_storage_identifier();
  metadata_.message_count = 0;
}

SimpleFileStorage::~SimpleFileStorage()
{
  // A bag that has not been opened needs no finalizing.
  if (!open_as_) {
    return;
  }

  // A bag that is open for READ_ONLY needs no finalizing.
  // In the case of READ_WRITE (write) - we need to write the final information to the file.
  if (open_as_ == rosbag2_storage::storage_interfaces::IOFlag::READ_WRITE) {
    auto serialized = metadata_io_.serialize_metadata(get_metadata());
    uint64_t serialized_size = serialized.size();

    // Write metadata
    char record_type = static_cast<char>(RecordType::METADATA);
    out_.write(&record_type, 1);
    out_.write(serialized.c_str(), serialized_size);

    // Write footer
    record_type = static_cast<char>(RecordType::FOOTER);
    out_.write(&record_type, 1);
    out_.write(reinterpret_cast<const char *>(&serialized_size), sizeof(serialized_size));

    // Finally, put the magic sequence at the end
    out_.write(MAGIC_SEQUENCE, 9);
    out_.write(reinterpret_cast<char *>(&version_), 1);
    out_.flush();
  }
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
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY:
      relative_path_ = storage_options.uri;
      in_ = std::ifstream(relative_path_, std::ios::binary);
      init_read();
      break;
    case rosbag2_storage::storage_interfaces::IOFlag::APPEND:
      throw std::runtime_error("SimpleFileStorage does not support APPEND mode");
      break;
  }
  open_as_ = io_flag;
  metadata_.relative_file_paths = {get_relative_file_path()};
}

void SimpleFileStorage::init_read()
{
  char magic_check[10];
  size_t backwards_bytes = 0;

  // Jumping backwards from the end of the file to get the metadata
  // First read the magic sequence
  backwards_bytes += 10;
  in_.seekg(-backwards_bytes, std::ios::end);
  in_.read(magic_check, 10);
  if (memcmp(magic_check, MAGIC_SEQUENCE, 9) != 0) {
    throw std::runtime_error("Specified file was not a SimpleFileStorage file");
  }
  version_ = magic_check[9];

  // Now read the footer (1 identifier byte and 8 bytes of metadata size)
  backwards_bytes += 9;
  uint64_t metadata_size = 0;
  uint8_t record_type;
  in_.seekg(-backwards_bytes, std::ios::end);
  in_.read(reinterpret_cast<char *>(&record_type), 1);
  assert(record_type == RecordType::FOOTER);
  in_.read(reinterpret_cast<char *>(&metadata_size), 8);

  // Read the metadata based on the size we got
  backwards_bytes += metadata_size;
  std::string serialized_metadata;
  serialized_metadata.resize(metadata_size);
  in_.seekg(-backwards_bytes, std::ios::end);
  in_.read(&serialized_metadata[0], metadata_size);
  metadata_ = metadata_io_.deserialize_metadata(serialized_metadata);

  // Finally, seek to the beginning and read the magic sequence off the top.
  // Now ready to read messages.
  in_.seekg(0, std::ios::beg);
  in_.read(magic_check, 10);
  if (memcmp(magic_check, MAGIC_SEQUENCE, 9) != 0) {
    throw std::runtime_error("Specified file was not a SimpleFileStorage file");
  }
  version_ = magic_check[9];
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
bool SimpleFileStorage::read_and_enqueue_message()
{
  // The bag has not been opened.
  if (!open_as_) {
    return false;
  }
  // Already have popped and queued the next message.
  if (next_ != nullptr) {
    return true;
  }

  // Initiating any read, the next byte indicates the record type.
  uint8_t record_type = in_.peek();
  if (record_type != RecordType::MESSAGE) {
    next_.reset();
    return false;
  }
  (void)in_.get();  // Clear the peeked RecordType byte.

  // MessageRecord format:
  // - timestamp (8 byte)
  // - topic name size (4 byte)
  // - topic name (variable)
  // - data size (4 byte)
  // - data (variable)
  auto msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
  uint32_t topic_size = 0;
  uint32_t data_size = 0;

  // Timestamp
  in_.read(reinterpret_cast<char *>(&msg->time_stamp), sizeof(msg->time_stamp));

  // Topic name size
  in_.read(reinterpret_cast<char *>(&topic_size), sizeof(topic_size));

  // Topic name
  msg->topic_name.resize(topic_size);
  in_.read(&msg->topic_name[0], topic_size);

  // Data size
  in_.read(reinterpret_cast<char *>(&data_size), sizeof(data_size));
  msg->serialized_data = rosbag2_storage::make_empty_serialized_message(data_size);

  // Data
  in_.read(reinterpret_cast<char *>(msg->serialized_data->buffer), data_size);
  msg->serialized_data->buffer_length = data_size;

  // enqueue this message to be used
  next_ = msg;
  return true;
}

bool SimpleFileStorage::has_next()
{
  if (!open_as_) {
    return false;
  }
  // Have already verified next message and enqueued it for use.
  if (next_) {
    return true;
  }

  // Continue reading messages until one matches the filter, or there are none left
  while (true) {
    if (!read_and_enqueue_message()) {
      return false;
    }
    if (filter_topics_.empty() || filter_topics_.count(next_->topic_name)) {
      break;
    }
    // Next message did not pass filter - throw it away
    next_.reset();
  }
  return true;
}

std::shared_ptr<rosbag2_storage::SerializedBagMessage> SimpleFileStorage::read_next()
{
  if (!has_next()) {
    throw std::runtime_error{"No next message is available."};
  }
  // Importantly, clear next_ via move so that a next message can be read.
  return std::move(next_);
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
  filter_topics_.clear();
  filter_topics_.insert(storage_filter.topics.begin(), storage_filter.topics.end());
}

void SimpleFileStorage::reset_filter()
{
  set_filter(rosbag2_storage::StorageFilter());
}

void SimpleFileStorage::seek(const rcutils_time_point_value_t & time_stamp)
{
  if (open_as_ != rosbag2_storage::storage_interfaces::IOFlag::READ_ONLY) {
    throw std::runtime_error{"Seek is only allowed in READ_ONLY mode."};
  }

  // If at the end, or after the seek destination, reset to beginning
  if (!has_next() || next_->time_stamp > time_stamp) {
    in_.seekg(10, std::ios::beg);
  }

  // Read messages until we reach the destination or the end
  while (has_next() && next_->time_stamp < time_stamp) {
    next_.reset();
  }
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
  // We will just state 1KiB as an arbitrary but small lower limit
  return 1024;
}

/** BaseWriteInterface **/
void SimpleFileStorage::write(std::shared_ptr<const rosbag2_storage::SerializedBagMessage> msg)
{
  static const char record_type = static_cast<char>(RecordType::MESSAGE);
  out_.write(&record_type, 1);

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


}  // namespace rosbag2_storage_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::SimpleFileStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
