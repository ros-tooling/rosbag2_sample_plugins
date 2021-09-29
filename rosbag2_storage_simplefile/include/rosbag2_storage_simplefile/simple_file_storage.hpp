#ifndef ROSBAG2_STORAGE_SIMPLEFILE__ROSBAG2_STORAGE_SIMPLEFILE_HPP_
#define ROSBAG2_STORAGE_SIMPLEFILE__ROSBAG2_STORAGE_SIMPLEFILE_HPP_

#include <fstream>

#include "rosbag2_storage/storage_interfaces/read_write_interface.hpp"

#include "rosbag2_storage_simplefile/visibility_control.h"

// This is necessary because of using stl types here. It is completely safe, because
// a) the member is not accessible from the outside
// b) there are no inline functions.
#ifdef _WIN32
# pragma warning(push)
# pragma warning(disable:4251)
#endif

namespace rosbag2_storage_plugins
{

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
  std::ofstream out_;
};

}  // namespace rosbag2_storage_plugins

#endif  // ROSBAG2_STORAGE_SIMPLEFILE__ROSBAG2_STORAGE_SIMPLEFILE_HPP_

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_storage_plugins::SimpleFileStorage,
  rosbag2_storage::storage_interfaces::ReadWriteInterface)
