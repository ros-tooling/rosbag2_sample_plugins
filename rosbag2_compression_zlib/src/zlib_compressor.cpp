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
#include <algorithm>
#include <filesystem>  // NOLINT - cpplint thinks this is a c system header
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "zlib.h"  // NOLINT

#include "rcutils/logging_macros.h"
#include "rosbag2_compression/base_compressor_interface.hpp"
#include "rosbag2_compression/base_decompressor_interface.hpp"

#include "./zlib_utils.hpp"

namespace rosbag2_compression_plugins
{

class ZlibCompressor
  : public rosbag2_compression::BaseCompressorInterface,
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
  // Prepare files for read/write
  const std::string compressed_uri = uri + "." + get_compression_identifier();
  auto source = std::shared_ptr<std::FILE>(fopen(uri.c_str(), "r"), std::fclose);
  auto dest = std::shared_ptr<std::FILE>(fopen(compressed_uri.c_str(), "w"), std::fclose);

  // Compress source file to dest, check result
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
  // Compress the input into a temporary expanding buffer
  std::vector<uint8_t> compressed_data;
  zlib_utils::compress(
    bag_message->serialized_data->buffer_length,
    bag_message->serialized_data->buffer,
    compressed_data);

  // Copy the result to the fixed-sized array output
  zlib_utils::vector_to_uint8array(compressed_data, bag_message->serialized_data);
}

std::string ZlibCompressor::get_compression_identifier() const
{
  return "zlib";
}

std::string ZlibCompressor::decompress_uri(const std::string & uri)
{
  // Check that file extension matches expected value
  if (std::filesystem::path{uri}.extension() != "." + get_decompression_identifier()) {
    std::stringstream errmsg;
    errmsg << "File " << uri << " was not a compressed file from " <<
      get_decompression_identifier() << " plugin.";
    throw std::runtime_error{errmsg.str()};
  }

  // Prepare files for read/write
  const auto decompressed_uri = std::filesystem::path{uri}.replace_extension("");
  auto source = std::shared_ptr<std::FILE>(fopen(uri.c_str(), "r"), std::fclose);
  auto dest = std::shared_ptr<std::FILE>(fopen(decompressed_uri.c_str(), "w"), std::fclose);

  // Decompress source file to dest and check result
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
  // Decompress data into expanding vector buffer
  std::vector<uint8_t> decompressed_data;
  zlib_utils::decompress(
    bag_message->serialized_data->buffer_length,
    bag_message->serialized_data->buffer,
    decompressed_data);

  // Copy result to fixed-size array in serialized message
  zlib_utils::vector_to_uint8array(decompressed_data, bag_message->serialized_data);
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
