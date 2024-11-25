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

#include "rmf2_scheduler/data/json_macros.hpp"

class TestJSONMacro : public ::testing::Test
{
};

namespace test_rs_json_macro_all_required
{

struct TestStruct
{
  std::string id;
  int index;

  inline bool operator==(const TestStruct & rhs) const
  {
    if (this->id != rhs.id) {return false;}
    if (this->index != rhs.index) {return false;}

    return true;
  }

  inline bool operator!=(const TestStruct & rhs) const
  {
    return !(*this == rhs);
  }
};

RS_JSON_DEFINE(
  TestStruct,
  id,
  index
)

}  // namespace test_rs_json_macro_all_required

TEST_F(TestJSONMacro, all_required) {
  using namespace test_rs_json_macro_all_required;  // NOLINT(build/namespaces)
  using json = nlohmann::json;
  {
    // Serializer
    TestStruct obj = TestStruct{"test_id", 3};
    json expected_obj_json = R"({
      "id": "test_id",
      "index": 3
    })"_json;

    EXPECT_EQ(
      expected_obj_json,
      json(obj)
    );
  }

  {
    // Deserializer
    json obj_json = R"({
      "id": "test_id",
      "index": 3
    })"_json;
    TestStruct expected_obj = TestStruct{"test_id", 3};

    EXPECT_EQ(
      expected_obj,
      obj_json.template get<TestStruct>()
    );
  }
}


namespace test_rs_json_macro_all_required_separate
{

struct TestStruct
{
  std::string id;
  int index;

  inline bool operator==(const TestStruct & rhs) const
  {
    if (this->id != rhs.id) {return false;}
    if (this->index != rhs.index) {return false;}

    return true;
  }

  inline bool operator!=(const TestStruct & rhs) const
  {
    return !(*this == rhs);
  }
};

RS_JSON_SERIALIZER_DEFINE_TYPE(
  TestStruct,
  id,
  index
)

RS_JSON_DESERIALIZER_DEFINE_TYPE(
  TestStruct,
  RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(
    id,
    index
  )
)

}  // namespace test_rs_json_macro_all_required_separate

TEST_F(TestJSONMacro, all_required_separate_serialize_deserialize) {
  using namespace test_rs_json_macro_all_required_separate;  // NOLINT(build/namespaces)
  using json = nlohmann::json;
  {
    // Serializer
    TestStruct obj = TestStruct{"test_id", 3};
    json expected_obj_json = R"({
      "id": "test_id",
      "index": 3
    })"_json;

    EXPECT_EQ(
      expected_obj_json,
      json(obj)
    );
  }

  {
    // Deserializer
    json obj_json = R"({
      "id": "test_id",
      "index": 3
    })"_json;
    TestStruct expected_obj = TestStruct{"test_id", 3};

    EXPECT_EQ(
      expected_obj,
      obj_json.template get<TestStruct>()
    );
  }
}


namespace test_rs_json_macro_optional
{

struct TestStruct
{
  std::string id;
  int index;

  std::string optional_id;
  int optional_index;

  inline bool operator==(const TestStruct & rhs) const
  {
    if (this->id != rhs.id) {return false;}
    if (this->index != rhs.index) {return false;}
    if (this->optional_id != rhs.optional_id) {return false;}
    if (this->optional_index != rhs.optional_index) {return false;}

    return true;
  }

  inline bool operator!=(const TestStruct & rhs) const
  {
    return !(*this == rhs);
  }
};

RS_JSON_SERIALIZER_DEFINE_TYPE(
  TestStruct,
  id,
  index,
  optional_id,
  optional_index
)

RS_JSON_DESERIALIZER_DEFINE_TYPE(
  TestStruct,
  RS_JSON_DESERIALIZER_REQUIRED_MEMBERS(
    id,
    index
  ),
  RS_JSON_DESERIALIZER_OPTIONAL_MEMBERS(
    optional_id,
    optional_index
  )
)

}  // namespace test_rs_json_macro_optional

TEST_F(TestJSONMacro, optional) {
  using namespace test_rs_json_macro_optional;  // NOLINT(build/namespaces)
  using json = nlohmann::json;
  {
    // Serializer
    TestStruct obj = TestStruct{"test_id", 3, "optional_test_id", 4};
    json expected_obj_json =
      R"({
      "id": "test_id",
      "index": 3,
      "optional_id": "optional_test_id",
      "optional_index": 4
    })"_json;

    EXPECT_EQ(
      expected_obj_json,
      json(obj)
    );
  }

  {
    // Deserializer required only
    json obj_json = R"({
      "id": "test_id",
      "index": 3
    })"_json;
    TestStruct expected_obj;
    expected_obj.id = "test_id";
    expected_obj.index = 3;

    EXPECT_EQ(
      expected_obj.id,
      obj_json.template get<TestStruct>().id
    );
    EXPECT_EQ(
      expected_obj.index,
      obj_json.template get<TestStruct>().index
    );
  }

  {
    // Deserializer full
    json obj_json =
      R"({
      "id": "test_id",
      "index": 3,
      "optional_id": "optional_test_id",
      "optional_index": 4
    })"_json;
    TestStruct expected_obj;
    expected_obj.id = "test_id";
    expected_obj.index = 3;
    expected_obj.optional_id = "optional_test_id";
    expected_obj.optional_index = 4;

    EXPECT_EQ(
      expected_obj,
      obj_json.template get<TestStruct>()
    );
  }
}
