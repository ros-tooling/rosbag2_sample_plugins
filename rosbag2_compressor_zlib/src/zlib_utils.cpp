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
#include <cassert>

#include <algorithm>
#include <vector>

#include "zlib.h"  // NOLINT

namespace zlib_utils
{

/// Utility functions implementing both file and buffer compression/decompression via Zlib.
/// Implementations copied with minor modifications from the Zlib samples:
/// https://zlib.net/zlib_how.html

static const size_t CHUNK = 262144;

int compress(FILE * source, FILE * dest, int level)
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
  if (ret != Z_OK) {
    return ret;
  }

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

int compress(size_t source_size, uint8_t * source, std::vector<uint8_t> & dest)
{
  uint8_t temp_buffer[CHUNK];

  z_stream strm;
  strm.zalloc = 0;
  strm.zfree = 0;
  strm.next_in = source;
  strm.avail_in = source_size;
  strm.next_out = temp_buffer;
  strm.avail_out = CHUNK;

  deflateInit(&strm, Z_DEFAULT_COMPRESSION);

  while (strm.avail_in != 0) {
    int res = deflate(&strm, Z_NO_FLUSH);
    assert(res == Z_OK);
    if (strm.avail_out == 0) {
      dest.insert(dest.end(), temp_buffer, temp_buffer + CHUNK);
      strm.next_out = temp_buffer;
      strm.avail_out = CHUNK;
    }
  }

  int deflate_res = Z_OK;
  while (deflate_res == Z_OK) {
    if (strm.avail_out == 0) {
      dest.insert(dest.end(), temp_buffer, temp_buffer + CHUNK);
      strm.next_out = temp_buffer;
      strm.avail_out = CHUNK;
    }
    deflate_res = deflate(&strm, Z_FINISH);
  }

  assert(deflate_res == Z_STREAM_END);
  dest.insert(dest.end(), temp_buffer, temp_buffer + CHUNK - strm.avail_out);
  deflateEnd(&strm);
  return Z_OK;
}

int decompress(FILE * source, FILE * dest)
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
  if (ret != Z_OK) {
    return ret;
  }

  /* decompress until deflate stream ends or end of file */
  do {
    strm.avail_in = fread(in, 1, CHUNK, source);
    if (ferror(source)) {
      (void)inflateEnd(&strm);
      return Z_ERRNO;
    }
    if (strm.avail_in == 0) {
      break;
    }
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

int decompress(size_t source_size, uint8_t * source, std::vector<uint8_t> & dest)
{
  int ret;
  unsigned have;
  z_stream strm;
  unsigned char * in;
  unsigned char out[CHUNK];

  size_t in_pos = 0;

  ret = inflateInit(&strm);
  if (ret != Z_OK) {
    return ret;
  }

  /* decompress until deflate stream ends or end of file */
  do {
    in = source + in_pos;
    strm.avail_in = std::min(CHUNK, source_size - in_pos);
    in_pos += strm.avail_in;

    if (strm.avail_in == 0) {
      break;
    }
    strm.next_in = in;

    /* run inflate() on input until output buffer not full */
    do {
      strm.avail_out = CHUNK;
      strm.next_out = out;
      ret = inflate(&strm, Z_NO_FLUSH);
      assert(ret != Z_STREAM_ERROR); /* state not clobbered */
      switch (ret) {
        case Z_NEED_DICT:
          ret = Z_DATA_ERROR; /* and fall through */
        case Z_DATA_ERROR:
        case Z_MEM_ERROR:
          (void)inflateEnd(&strm);
          return ret;
      }
      have = CHUNK - strm.avail_out;
      dest.insert(dest.end(), out, out + have);
    } while (strm.avail_out == 0);
    /* done when inflate() says it's done */
  } while (ret != Z_STREAM_END);

  /* clean up and return */
  (void)inflateEnd(&strm);
  return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

}  // namespace zlib_utils
