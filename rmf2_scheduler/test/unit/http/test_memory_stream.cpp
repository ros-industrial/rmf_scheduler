// Copyright 2024 ROS Industrial Consortium Asia Pacific
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

#include "gtest/gtest.h"

#include "rmf2_scheduler/http/memory_stream.hpp"

class TestMemoryStream : public ::testing::Test
{
};

TEST_F(TestMemoryStream, read_and_write) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  // Initial size
  MemoryStream::UPtr stream = std::make_unique<MemoryStream>(20);
  EXPECT_EQ(stream->size(), 0);
  EXPECT_EQ(stream->remaining_size(), 0);
  EXPECT_EQ(stream->position(), 0);
  EXPECT_EQ(stream->capacity(), 20);

  // Normal write
  size_t size_written;
  stream->write("Hello World!!!", 12, &size_written);
  EXPECT_EQ(size_written, 12);
  EXPECT_EQ(stream->size(), 12);
  EXPECT_EQ(stream->remaining_size(), 0);
  EXPECT_EQ(stream->position(), 12);
  EXPECT_EQ(stream->capacity(), 20);

  // Update position
  stream->set_position(0);
  EXPECT_EQ(stream->remaining_size(), 12);
  EXPECT_EQ(stream->position(), 0);

  // Normal read
  size_t size_read;
  char data_read[20];

  EXPECT_TRUE(stream->read(data_read, 12, &size_read));
  EXPECT_EQ(size_read, 12);
  EXPECT_EQ(std::string(data_read, size_read), "Hello World!");
  EXPECT_EQ(stream->size(), 12);
  EXPECT_EQ(stream->remaining_size(), 0);
  EXPECT_EQ(stream->position(), 12);
  EXPECT_EQ(stream->capacity(), 20);

  // Reserve (do nothing)
  stream->reserve(10);
  EXPECT_EQ(stream->capacity(), 20);

  // Automatic resize
  stream->write("Hello World!!!!", 15, &size_written);
  EXPECT_EQ(size_written, 15);
  EXPECT_EQ(stream->size(), 27);
  EXPECT_EQ(stream->remaining_size(), 0);
  EXPECT_EQ(stream->position(), 27);
  EXPECT_EQ(stream->capacity(), 27);
}

TEST_F(TestMemoryStream, open_copy_of) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  {
    // void *
    int data_array[5];
    size_t data_array_size = sizeof(int) * 5;
    for (int i = 0; i < 5; i++) {
      data_array[i] = i + 4;
    }
    Stream::UPtr stream = MemoryStream::open_copy_of(&data_array[0], data_array_size);
    EXPECT_EQ(stream->size(), data_array_size);
    EXPECT_EQ(stream->remaining_size(), data_array_size);
    EXPECT_EQ(stream->position(), 0);
    EXPECT_EQ(stream->capacity(), data_array_size);

    // Check data
    int data_array_read[5];
    EXPECT_TRUE(stream->read(data_array_read, data_array_size, nullptr));
    for (int i = 0; i < 5; i++) {
      EXPECT_EQ(data_array[i], data_array_read[i]);
    }
  }

  {
    // const char *
    const char * data_c_string = "Hello World!!!!";
    size_t data_size_c_string = 15;
    Stream::UPtr stream = MemoryStream::open_copy_of(data_c_string);
    EXPECT_EQ(stream->size(), data_size_c_string);
    EXPECT_EQ(stream->remaining_size(), data_size_c_string);
    EXPECT_EQ(stream->position(), 0);
    EXPECT_EQ(stream->capacity(), data_size_c_string);

    // Check data
    char data_c_string_read[15];
    EXPECT_TRUE(stream->read(data_c_string_read, data_size_c_string, nullptr));
    EXPECT_EQ(strncmp(data_c_string_read, data_c_string, 15), 0);
  }

  {
    // std::string
    std::string data_string = "Hello World!!!!";
    size_t data_size_string = 15;
    Stream::UPtr stream = MemoryStream::open_copy_of(data_string);
    EXPECT_EQ(stream->size(), data_size_string);
    EXPECT_EQ(stream->remaining_size(), data_size_string);
    EXPECT_EQ(stream->position(), 0);
    EXPECT_EQ(stream->capacity(), data_size_string);

    // Check data
    char data_c_string_read[15];
    EXPECT_TRUE(stream->read(data_c_string_read, data_size_string, nullptr));
    EXPECT_EQ(std::string(data_c_string_read, 15), data_string);
  }
}

TEST_F(TestMemoryStream, seek) {
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  {
    // FROM_BEGIN
    std::string data_string = "Hello World!!!!";
    size_t data_size_string = 15;
    Stream::UPtr stream = MemoryStream::open_copy_of(data_string);
    EXPECT_EQ(stream->size(), data_size_string);
    EXPECT_EQ(stream->position(), 0);

    // Success
    EXPECT_TRUE(stream->seek(5, Stream::Whence::FROM_BEGIN, nullptr));
    EXPECT_EQ(stream->position(), 5);
    EXPECT_EQ(stream->remaining_size(), data_size_string - 5);

    EXPECT_FALSE(stream->seek(data_size_string + 1, Stream::Whence::FROM_BEGIN, nullptr));
  }

  {
    // FROM_CURRENT
    std::string data_string = "Hello World!!!!";
    size_t data_size_string = 15;
    Stream::UPtr stream = MemoryStream::open_copy_of(data_string);
    EXPECT_EQ(stream->size(), data_size_string);
    EXPECT_EQ(stream->position(), 0);

    stream->set_position(5);

    // Success
    EXPECT_TRUE(stream->seek(2, Stream::Whence::FROM_CURRENT, nullptr));
    EXPECT_EQ(stream->position(), 7);
    EXPECT_EQ(stream->remaining_size(), data_size_string - 7);

    EXPECT_FALSE(stream->seek(data_size_string, Stream::Whence::FROM_CURRENT, nullptr));
  }

  {
    // FROM_END
    std::string data_string = "Hello World!!!!";
    size_t data_size_string = 15;
    Stream::UPtr stream = MemoryStream::open_copy_of(data_string);
    EXPECT_EQ(stream->size(), data_size_string);
    EXPECT_EQ(stream->position(), 0);

    // Success
    EXPECT_TRUE(stream->seek(8, Stream::Whence::FROM_END, nullptr));
    EXPECT_EQ(stream->position(), data_size_string - 8);
    EXPECT_EQ(stream->remaining_size(), 8);

    EXPECT_FALSE(stream->seek(data_size_string + 1, Stream::Whence::FROM_END, nullptr));
  }
}
