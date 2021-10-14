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

#ifndef ZLIB_UTILS_HPP_
#define ZLIB_UTILS_HPP_

#include <vector>

namespace zlib_utils
{

int compress(FILE * source, FILE * dest, int level);
int compress(size_t source_size, uint8_t * source, std::vector<uint8_t> & dest);
int decompress(FILE * source, FILE * dest);
int decompress(size_t source_size, uint8_t * source, std::vector<uint8_t> & dest);

}  // namespace zlib_utils

#endif  // ZLIB_UTILS_HPP_
