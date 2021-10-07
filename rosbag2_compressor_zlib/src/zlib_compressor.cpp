#include <sstream>
#include <filesystem>

#include "rosbag2_compression/base_compressor_interface.hpp"
#include "rosbag2_compression/base_decompressor_interface.hpp"

#include "zlib.h"

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

private:
  // Helper functions taken from https://zlib.net/zpipe.c
  int def(FILE * source, FILE * dest, int level) const;
  int inf(FILE * source, FILE * dest) const;

  static const size_t CHUNK = 262144;
};

std::string ZlibCompressor::compress_uri(const std::string & uri)
{
  const std::string compressed_uri = uri + "." + get_compression_identifier();

  FILE * source = fopen(uri.c_str(), "r");
  FILE * dest = fopen(compressed_uri.c_str(), "w");

  int ret = def(source, dest, Z_DEFAULT_COMPRESSION);
  if (ret != Z_OK) {
    std::stringstream errmsg;
    errmsg << "Failed to compress uri " << uri << ". Code " << ret;
    throw std::runtime_error{errmsg.str()};
  }

  fclose(source);
  fclose(dest);
  return compressed_uri;
}

void ZlibCompressor::compress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * bag_message)
{
  // TODO
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

  FILE * source = fopen(uri.c_str(), "r");
  FILE * dest = fopen(decompressed_uri.c_str(), "w");

  int ret = inf(source, dest);
  if (ret != Z_OK) {
    std::stringstream errmsg;
    errmsg << "Failed to decompress uri " << uri << ". Code " << ret;
    throw std::runtime_error{errmsg.str()};
  }

  fclose(source);
  fclose(dest);

  return decompressed_uri;
}

void ZlibCompressor::decompress_serialized_bag_message(
  rosbag2_storage::SerializedBagMessage * bag_message)
{
  // TODO
}

std::string ZlibCompressor::get_decompression_identifier() const
{
  return "zlib";
}

int ZlibCompressor::def(FILE * source, FILE * dest, int level) const
{
  int ret, flush;
  unsigned have;
  z_stream strm;
  unsigned char in[CHUNK];
  unsigned char out[CHUNK];

  /* allocate deflate state */
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  ret = deflateInit(&strm, level);
  if (ret != Z_OK)
      return ret;

  /* compress until end of file */
  do {
      strm.avail_in = fread(in, 1, CHUNK, source);
      if (ferror(source)) {
          (void)deflateEnd(&strm);
          return Z_ERRNO;
      }
      flush = feof(source) ? Z_FINISH : Z_NO_FLUSH;
      strm.next_in = in;

      /* run deflate() on input until output buffer not full, finish
         compression if all of source has been read in */
      do {
          strm.avail_out = CHUNK;
          strm.next_out = out;
          ret = deflate(&strm, flush);    /* no bad return value */
          assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
          have = CHUNK - strm.avail_out;
          if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
              (void)deflateEnd(&strm);
              return Z_ERRNO;
          }
      } while (strm.avail_out == 0);
      assert(strm.avail_in == 0);     /* all input will be used */

      /* done when last data in file processed */
  } while (flush != Z_FINISH);
  assert(ret == Z_STREAM_END);        /* stream will be complete */

  /* clean up and return */
  (void)deflateEnd(&strm);

  return Z_OK;
}

int ZlibCompressor::inf(FILE * source, FILE * dest) const
{
  int ret;
  unsigned have;
  z_stream strm;
  unsigned char in[CHUNK];
  unsigned char out[CHUNK];

  /* allocate inflate state */
  strm.zalloc = Z_NULL;
  strm.zfree = Z_NULL;
  strm.opaque = Z_NULL;
  strm.avail_in = 0;
  strm.next_in = Z_NULL;
  ret = inflateInit(&strm);
  if (ret != Z_OK)
      return ret;

  /* decompress until deflate stream ends or end of file */
  do {
      strm.avail_in = fread(in, 1, CHUNK, source);
      if (ferror(source)) {
          (void)inflateEnd(&strm);
          return Z_ERRNO;
      }
      if (strm.avail_in == 0)
          break;
      strm.next_in = in;

      /* run inflate() on input until output buffer not full */
      do {
          strm.avail_out = CHUNK;
          strm.next_out = out;
          ret = inflate(&strm, Z_NO_FLUSH);
          assert(ret != Z_STREAM_ERROR);  /* state not clobbered */
          switch (ret) {
          case Z_NEED_DICT:
              ret = Z_DATA_ERROR;     /* and fall through */
          case Z_DATA_ERROR:
          case Z_MEM_ERROR:
              (void)inflateEnd(&strm);
              return ret;
          }
          have = CHUNK - strm.avail_out;
          if (fwrite(out, 1, have, dest) != have || ferror(dest)) {
              (void)inflateEnd(&strm);
              return Z_ERRNO;
          }
      } while (strm.avail_out == 0);

      /* done when inflate() says it's done */
  } while (ret != Z_STREAM_END);

  /* clean up and return */
  (void)inflateEnd(&strm);
  return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

}  // namespace rosbag2_compression_plugins

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression_plugins::ZlibCompressor,
  rosbag2_compression::BaseCompressorInterface)
PLUGINLIB_EXPORT_CLASS(
  rosbag2_compression_plugins::ZlibCompressor,
  rosbag2_compression::BaseDecompressorInterface)
