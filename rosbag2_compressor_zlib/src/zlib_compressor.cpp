#include <sstream>
#include <filesystem>
#include <vector>

#include "rosbag2_compression/base_compressor_interface.hpp"
#include "rosbag2_compression/base_decompressor_interface.hpp"

#define CHUNK 262144
#include "zlib_utils.c"

namespace
{

void throw_on_rcutils_resize_error(const rcutils_ret_t resize_result)
{
  if (resize_result == RCUTILS_RET_OK) {
    return;
  }

  std::stringstream error;
  error << "rcutils_uint8_array_resize error: ";
  switch (resize_result) {
    case RCUTILS_RET_INVALID_ARGUMENT:
      error << "Invalid Argument";
      break;
    case RCUTILS_RET_BAD_ALLOC:
      error << "Bad Alloc";
      break;
    case RCUTILS_RET_ERROR:
      error << "Ret Error";
      break;
    default:
      error << "Unexpected Result";
      break;
  }
  throw std::runtime_error(error.str());
}

}  // namespace

namespace rosbag2_compression_plugins
{

class ZlibCompressor :
  public rosbag2_compression::BaseCompressorInterface,
  public rosbag2_compression::BaseDecompressorInterface
{
public:
  ZlibCompressor() = default;
  virtual ~ZlibCompressor() = default;

  /** BaseCompressorInterface **/
  std::string compress_uri(const std::string & uri) override;

  void compress_serialized_bag_message(
    rosbag2_storage::SerializedBagMessage * bag_message) override;

  std::string get_compression_identifier() const override;

  /** BaseDecompressorInterface **/
  std::string decompress_uri(const std::string & uri) override;

  void decompress_serialized_bag_message(
    rosbag2_storage::SerializedBagMessage * bag_message) override;

  std::string get_decompression_identifier() const override;
};

std::string ZlibCompressor::compress_uri(const std::string & uri)
{
  const std::string compressed_uri = uri + "." + get_compression_identifier();

  auto source = std::shared_ptr<std::FILE>(fopen(uri.c_str(), "r"), std::fclose);
  auto dest = std::shared_ptr<std::FILE>(fopen(compressed_uri.c_str(), "w"), std::fclose);

  int ret = zlib_utils::compress(source.get(), dest.get(), Z_DEFAULT_COMPRESSION);
  if (ret != Z_OK) {
    std::stringstream errmsg;
    errmsg << "Failed to compress uri " << uri << ". Code " << ret;
    throw std::runtime_error{errmsg.str()};
  }

  return compressed_uri;
}

void ZlibCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * bag_message)
{
  std::vector<uint8_t> compressed_data;

  zlib_utils::compress(
    bag_message->serialized_data->buffer_length,
    bag_message->serialized_data->buffer,
    compressed_data);

  const auto resize_result = rcutils_uint8_array_resize(
    bag_message->serialized_data.get(), compressed_data.size());
  throw_on_rcutils_resize_error(resize_result);
  bag_message->serialized_data->buffer_length = compressed_data.size();
  std::copy(compressed_data.begin(), compressed_data.end(), bag_message->serialized_data->buffer);
}

std::string ZlibCompressor::get_compression_identifier() const
{
  return "zlib";
}

std::string ZlibCompressor::decompress_uri(const std::string & uri)
{
  const auto path_uri = std::filesystem::path{uri};
  if (path_uri.extension() != "." + get_decompression_identifier()) {
    std::stringstream errmsg;
    errmsg << "File " << uri << " was not a compressed file from "
      << get_decompression_identifier() << " plugin.";
    throw std::runtime_error{errmsg.str()};
  }

  const auto decompressed_uri = std::filesystem::path{uri}.replace_extension("");

  auto source = std::shared_ptr<std::FILE>(fopen(uri.c_str(), "r"), std::fclose);
  auto dest = std::shared_ptr<std::FILE>(fopen(decompressed_uri.c_str(), "w"), std::fclose);
  int ret = zlib_utils::decompress(source.get(), dest.get());
  if (ret != Z_OK) {
    std::stringstream errmsg;
    errmsg << "Failed to decompress uri " << uri << ". Code " << ret;
    throw std::runtime_error{errmsg.str()};
  }

  return decompressed_uri;
}

void ZlibCompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * bag_message)
{
  std::vector<uint8_t> decompressed_data;
  zlib_utils::decompress(
    bag_message->serialized_data->buffer_length,
    bag_message->serialized_data->buffer,
    decompressed_data);
  const auto resize_result = rcutils_uint8_array_resize(
    bag_message->serialized_data.get(), decompressed_data.size());
  throw_on_rcutils_resize_error(resize_result);
  bag_message->serialized_data->buffer_length = decompressed_data.size();
  std::copy(
    decompressed_data.begin(), decompressed_data.end(), bag_message->serialized_data->buffer);
}

std::string ZlibCompressor::get_decompression_identifier() const
{
  return "zlib";
}

}  // namespace rosbag2_compression_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression_plugins::ZlibCompressor,
  rosbag2_compression::BaseCompressorInterface)
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression_plugins::ZlibCompressor,
  rosbag2_compression::BaseDecompressorInterface)
