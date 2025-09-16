// Copyright 2025 ROS Industrial Consortium Asia Pacific
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

#include "rmf2_scheduler/storage/schedule_stream_ld_broker.hpp"
#include "rmf2_scheduler/cache/action.hpp"
#include "rmf2_scheduler/http/transport_fake.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"


class TestStorageScheduleStreamLDBroker : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
    using namespace rmf2_scheduler::storage;  // NOLINT(build/namespaces)
    using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)
    transport_ = std::make_shared<fake::Transport>();

    // Context broker URL
    std::string ld_broker_domain = "http://localhost:9090";
    std::string ld_broker_path = "ngsi-ld";

    // Create URL
    ld_broker_url_ = url::combine(
      ld_broker_domain,
      ld_broker_path
    );

    // Create schedule stream
    schedule_stream_ = std::make_shared<ld_broker::ScheduleStream>(
      ld_broker_url_,
      transport_
    );
  }

  void TearDown() override
  {
    schedule_stream_.reset();
    transport_.reset();
  }

protected:
  std::shared_ptr<rmf2_scheduler::storage::ld_broker::ScheduleStream> schedule_stream_;
  std::shared_ptr<rmf2_scheduler::http::fake::Transport> transport_;
  std::string ld_broker_url_;
};

TEST_F(TestStorageScheduleStreamLDBroker, create_connection_error) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  std::string connection_error = "Cannot connect to " + ld_broker_url_;

  transport_->set_create_connection_error(connection_error);
  auto schedule_cache = ScheduleCache::make_shared();

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  std::string error;
  EXPECT_FALSE(
    schedule_stream_->read_schedule(
      schedule_cache,
      time_window,
      error
  ));

  EXPECT_EQ(connection_error, error);
}

TEST_F(TestStorageScheduleStreamLDBroker, not_found_error) {
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)

  auto schedule_cache = ScheduleCache::make_shared();

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  std::string error;
  EXPECT_FALSE(
    schedule_stream_->read_schedule(
      schedule_cache,
      time_window,
      error
  ));

  EXPECT_EQ(
    "Invalid response from GET "
    "http://localhost:9090/ngsi-ld/v1/entities?"
    "type=urn%3Armf2%3Aevent&limit=1000&offset=0&"
    "q=startTime%3E%3D2025-01-21T10%3A00%3A00Z%3BstartTime%3C2025-01-22T10%3A00%3A00Z\n"
    "404 Not Found\n"
    "<html><body>Not found</body><html>",
    error
  );
}

TEST_F(TestStorageScheduleStreamLDBroker, read_schedule_event_only) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  // Create callback handler for /v1/entities
  fake::ServerRequestHandler read_entities_handler;

  // Add first callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ {
          "id": "urn:event:477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
          "type": "urn:rmf2:event",
          "payload": {
            "type": "Property",
            "value": {
              "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
              "type": "task_type",
              "description": "This is a task!",
              "start_time": "2025-01-22T02:15:00Z",
              "end_time": "2025-01-22T03:15:00Z",
              "resource_id": "09d5323d-43e0-4668-b53a-3cf33f9b9a96",
              "deadline": "2025-01-22T04:15:00Z",
              "status": "ongoing",
              "planned_start_time": "2025-01-22T02:15:00Z",
              "planned_end_time": "2025-01-22T03:15:00Z",
              "estimated_duration": 3700.0,
              "actual_start_time": "2025-01-22T02:16:00Z",
              "actual_end_time": "2025-01-22T03:18:40Z",
              "task_details": {
                "some_field": "some_value"
              }
            }
          },
          "startTime": {
            "type": "Property",
            "value": {
              "@type": "Datetime",
              "@value": "2025-01-22T02:15:00Z"
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        } ])"_json,
        "application/json+ld"
      );
    }
  );

  // Add second callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );


  // Read event handler
  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entities"}),
    request_type::kGet,
    std::bind(
      &fake::ServerRequestHandler::perform,
      &read_entities_handler,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  auto schedule_cache = ScheduleCache::make_shared();

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  std::string error;
  EXPECT_TRUE(
    schedule_stream_->read_schedule(
      schedule_cache,
      time_window,
      error
  ));

  EXPECT_EQ(2, transport_->get_request_count());
  EXPECT_EQ(2, read_entities_handler.call_count());

  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  EXPECT_EQ(task->type, "task_type");
  EXPECT_EQ(task->duration, Duration(3600, 0));
}


TEST_F(TestStorageScheduleStreamLDBroker, read_schedule_event_and_process) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  // Create callback handler for /v1/entities
  fake::ServerRequestHandler read_entities_handler;

  // Add first callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      ASSERT_NE(nullptr, response);

      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      ASSERT_NE(nullptr, response);
      // Provide stub event reply
      response->reply_json(
        status_code::Ok,
        R"([ {
          "id": "urn:event:477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
          "type": "urn:rmf2:event",
          "payload": {
            "type": "Property",
            "value": {
              "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
              "type": "task_type",
              "start_time": "2025-01-22T02:15:00Z",
              "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "status": "ongoing"
            }
          },
          "startTime": {
            "type": "Property",
            "value": {
              "@type": "Datetime",
              "@value": "2025-01-22T02:15:00Z"
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        }, {
          "id": "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973",
          "type": "urn:rmf2:event",
          "payload": {
            "type": "Property",
            "value": {
              "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
              "type": "task_type",
              "start_time": "2025-01-22T04:15:00Z",
              "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "status": "ongoing"
            }
          },
          "startTime": {
            "type": "Property",
            "value": {
              "@type": "Datetime",
              "@value": "2025-01-22T04:15:00Z"
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        } ])"_json,
        "application/json+ld"
      );
    }
  );

  // Add second callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );

  // Add third callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:process");
      EXPECT_EQ(request.get_form_field("id"), "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ {
          "id": "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "type": "urn:rmf2:process",
          "payload": {
            "type": "Property",
            "value": {
              "id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "graph": [
                { "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6", "needs": [] },
                {
                  "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
                  "needs": {
                    "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
                    "type": "hard"
                  }
                }
              ],
              "status": "",
              "current_events" : [],
              "process_details" : null
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        }])"_json,
        "application/json+ld"
      );
    }
  );

  // Add 4th callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:process");
      EXPECT_EQ(request.get_form_field("id"), "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );

  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entities"}),
    request_type::kGet,
    std::bind(
      &fake::ServerRequestHandler::perform,
      &read_entities_handler,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  auto schedule_cache = ScheduleCache::make_shared();

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  std::string error;
  EXPECT_TRUE(
    schedule_stream_->read_schedule(
      schedule_cache,
      time_window,
      error
  ));

  EXPECT_EQ(4, transport_->get_request_count());
  EXPECT_EQ(4, read_entities_handler.call_count());

  // Check task
  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task1 = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  ASSERT_TRUE(task1->process_id.has_value());
  EXPECT_EQ(task1->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  ASSERT_TRUE(schedule_cache->has_task("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto task2 = schedule_cache->get_task("8a9f4819-bbd4-4550-acce-e061d3289973");
  ASSERT_TRUE(task2->process_id.has_value());
  EXPECT_EQ(task2->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  // Check process
  ASSERT_TRUE(schedule_cache->has_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a"));
  auto process = schedule_cache->get_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a");
  EXPECT_TRUE(process->graph.has_node("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto node = process->graph.get_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  EXPECT_EQ(Edge("hard"), node->inbound_edges().at("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
}

TEST_F(
  TestStorageScheduleStreamLDBroker,
  read_schedule_event_and_process_with_extra
) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  // Create callback handler for /v1/entities
  fake::ServerRequestHandler read_entities_handler;

  // Add first callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      ASSERT_NE(nullptr, response);

      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      ASSERT_NE(nullptr, response);
      // Provide stub event reply
      response->reply_json(
        status_code::Ok,
        R"([ {
          "id": "urn:event:477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
          "type": "urn:rmf2:event",
          "payload": {
            "type": "Property",
            "value": {
              "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
              "type": "task_type",
              "start_time": "2025-01-22T02:15:00Z",
              "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "status": "ongoing"
            }
          },
          "startTime": {
            "type": "Property",
            "value": {
              "@type": "Datetime",
              "@value": "2025-01-22T02:15:00Z"
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        }])"_json,
        "application/json+ld"
      );
    }
  );

  // Add second callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");
      EXPECT_EQ(
        request.get_form_field("q"),
        "startTime>=2025-01-21T10:00:00Z;"
        "startTime<2025-01-22T10:00:00Z"
      );

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );

  // Add third callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:process");
      EXPECT_EQ(request.get_form_field("id"), "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ {
          "id": "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "type": "urn:rmf2:process",
          "payload": {
            "type": "Property",
            "value": {
              "id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "graph": [
                { "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6", "needs": [] },
                {
                  "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
                  "needs": {
                    "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
                    "type": "hard"
                  }
                }
              ],
              "status": "",
              "current_events" : [],
              "process_details" : null
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        }])"_json,
        "application/json+ld"
      );
    }
  );

  // Add 4th callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:process");
      EXPECT_EQ(request.get_form_field("id"), "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );

  // Add 5th callback
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      ASSERT_NE(nullptr, response);

      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("id"), "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973");
      EXPECT_EQ(request.get_form_field("offset"), "0");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      ASSERT_NE(nullptr, response);
      // Provide stub event reply
      response->reply_json(
        status_code::Ok,
        R"([{
          "id": "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973",
          "type": "urn:rmf2:event",
          "payload": {
            "type": "Property",
            "value": {
              "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
              "type": "task_type",
              "start_time": "2025-01-23T04:15:00Z",
              "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
              "status": "ongoing"
            }
          },
          "startTime": {
            "type": "Property",
            "value": {
              "@type": "Datetime",
              "@value": "2025-01-23T04:15:00Z"
            }
          },
          "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld"
        }])"_json,
        "application/json+ld"
      );
    }
  );

  // Add 6th callback, empty for end of pagination
  read_entities_handler.add_callback(
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      // Check query parameters
      EXPECT_EQ(request.get_form_field("type"), "urn:rmf2:event");
      EXPECT_EQ(request.get_form_field("id"), "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973");
      EXPECT_EQ(request.get_form_field("offset"), "1000");
      EXPECT_EQ(request.get_form_field("limit"), "1000");

      // Provide stub reply
      ASSERT_NE(nullptr, response);
      response->reply_json(
        status_code::Ok,
        R"([ ])"_json,
        "application/json+ld"
      );
    }
  );

  // Read event handler
  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entities"}),
    request_type::kGet,
    std::bind(
      &fake::ServerRequestHandler::perform,
      &read_entities_handler,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  auto schedule_cache = ScheduleCache::make_shared();

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  std::string error;
  EXPECT_TRUE(
    schedule_stream_->read_schedule(
      schedule_cache,
      time_window,
      error
  ));

  EXPECT_EQ(6, transport_->get_request_count());
  EXPECT_EQ(6, read_entities_handler.call_count());

  // Check task
  ASSERT_TRUE(schedule_cache->has_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
  auto task1 = schedule_cache->get_task("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  ASSERT_TRUE(task1->process_id.has_value());
  EXPECT_EQ(task1->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  ASSERT_TRUE(schedule_cache->has_task("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto task2 = schedule_cache->get_task("8a9f4819-bbd4-4550-acce-e061d3289973");
  ASSERT_TRUE(task2->process_id.has_value());
  EXPECT_EQ(task2->process_id.value(), "13aa1c62-64ca-495d-a4b7-84de6a00f56a");

  // Check process
  ASSERT_TRUE(schedule_cache->has_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a"));
  auto process = schedule_cache->get_process("13aa1c62-64ca-495d-a4b7-84de6a00f56a");
  EXPECT_TRUE(process->graph.has_node("8a9f4819-bbd4-4550-acce-e061d3289973"));
  auto node = process->graph.get_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  EXPECT_EQ(Edge("hard"), node->inbound_edges().at("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6"));
}

TEST_F(
  TestStorageScheduleStreamLDBroker,
  write_schedule_time_window_empty
) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  // Write event handler
  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entityOperations", "upsert"}),
    request_type::kPost,
    [](const fake::ServerRequest & request, fake::ServerResponse * response) {
      ASSERT_NE(nullptr, response);
      EXPECT_FALSE(request.get_data_as_json().has_value());
      response->reply(status_code::Created, nullptr, 0, "text");
    }
  );

  auto schedule_cache = ScheduleCache::make_shared();

  std::string error;
  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  EXPECT_TRUE(
    schedule_stream_->write_schedule(
      schedule_cache,
      time_window,
      error
  ));

  // No call
  EXPECT_EQ(0, transport_->get_request_count());
}

TEST_F(
  TestStorageScheduleStreamLDBroker,
  write_schedule_time_window
) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  std::vector<nlohmann::json> request_bodies;

  // Write event handler
  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entityOperations", "upsert"}),
    request_type::kPost,
    [&request_bodies](const fake::ServerRequest & request, fake::ServerResponse * response) {
      auto request_body = request.get_data_as_json();
      ASSERT_TRUE(request_body.has_value());
      request_bodies.push_back(request_body.value());

      // Check content type
      EXPECT_EQ("application/ld+json", request.get_header(response_header::kContentType));

      ASSERT_NE(nullptr, response);
      response->reply(status_code::Created, nullptr, 0, "text");
    }
  );

  auto schedule_cache = ScheduleCache::make_shared();

  // Add tasks and process
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task1->type = "task_type";
  task1->start_time = Time::from_ISOtime("2025-01-22T02:15:00Z");
  task1->status = "ongoing";

  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = "8a9f4819-bbd4-4550-acce-e061d3289973";
  task2->type = "task_type";
  task2->start_time = Time::from_ISOtime("2025-01-22T04:15:00Z");
  task2->status = "ongoing";

  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = "ad0b5ee3-fd53-489b-a712-99d8b24161e8";
  task3->type = "task_type";
  task3->start_time = Time::from_ISOtime("2025-01-24T04:15:00Z");  // not in range
  task3->status = "ongoing";

  std::vector<Task::Ptr> tasks {task1, task2, task3};

  Process::Ptr process = std::make_shared<Process>();
  process->id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process->graph.add_node("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  process->graph.add_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  process->graph.add_edge(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "8a9f4819-bbd4-4550-acce-e061d3289973"
  );

  std::vector<Process::Ptr> processes{process};
  std::string error;

  for (auto & task : tasks) {
    auto cache_action = Action::create(
      action_type::TASK_ADD,
      ActionPayload().task(task)
    );

    ASSERT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
  }

  for (auto & process : processes) {
    auto cache_action = Action::create(
      action_type::PROCESS_ADD,
      ActionPayload().process(process)
    );

    EXPECT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
  }

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  EXPECT_TRUE(
    schedule_stream_->write_schedule(
      schedule_cache,
      time_window,
      error
  ));

  // Call 2 times
  EXPECT_EQ(2, transport_->get_request_count());

  // Check the request data
  std::vector<nlohmann::json> expected_data {
    // First request
    R"([{
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:event:477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "payload": {
        "type": "Property",
        "value": {
          "actual_end_time": null,
          "actual_start_time": null,
          "deadline": null,
          "description": null,
          "end_time": null,
          "estimated_duration": null,
          "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
          "planned_end_time": null,
          "planned_start_time": null,
          "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "resource_id": null,
          "series_id": null,
          "start_time": "2025-01-22T02:15:00Z",
          "status": "ongoing",
          "task_details": null,
          "type": "task_type"
        }
      },
      "startTime": {
        "type": "Property",
        "value": {
          "@type": "Datetime",
          "@value": "2025-01-22T02:15:00Z"
        }
      },
      "type": "urn:rmf2:event"
    },
    {
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973",
      "payload": {
        "type": "Property",
        "value": {
          "actual_end_time": null,
          "actual_start_time": null,
          "deadline": null,
          "description": null,
          "end_time": null,
          "estimated_duration": null,
          "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
          "planned_end_time": null,
          "planned_start_time": null,
          "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "resource_id": null,
          "series_id": null,
          "start_time": "2025-01-22T04:15:00Z",
          "status": "ongoing",
          "task_details": null,
          "type": "task_type"
        }
      },
      "startTime": {
        "type": "Property",
        "value": {
          "@type": "Datetime",
          "@value": "2025-01-22T04:15:00Z"
        }
      },
      "type": "urn:rmf2:event"
    }])"_json,
    R"([{
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "payload": {
        "type": "Property",
        "value": {
          "graph": [
            {
              "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
              "needs": []
            },
            {
              "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
              "needs": [
                {
                  "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
                  "type": "hard"
                }
              ]
            }
          ],
          "id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "status": "",
          "current_events" : [],
          "process_details" : null
        }
      },
      "type": "urn:rmf2:process"
    }])"_json
  };
  EXPECT_EQ(
    request_bodies,
    expected_data
  );
}

TEST_F(
  TestStorageScheduleStreamLDBroker,
  write_schedule_record
) {
  using namespace rmf2_scheduler;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::cache;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::data;  // NOLINT(build/namespaces)
  using namespace rmf2_scheduler::http;  // NOLINT(build/namespaces)

  std::vector<nlohmann::json> request_bodies;

  // Write event handler
  transport_->add_handler(
    url::combine_multiple(ld_broker_url_, {"v1", "entityOperations", "upsert"}),
    request_type::kPost,
    [&request_bodies](const fake::ServerRequest & request, fake::ServerResponse * response) {
      auto request_body = request.get_data_as_json();
      ASSERT_TRUE(request_body.has_value());
      request_bodies.push_back(request_body.value());

      // Check content type
      EXPECT_EQ("application/ld+json", request.get_header(response_header::kContentType));

      ASSERT_NE(nullptr, response);
      response->reply(status_code::Ok, nullptr, 0, "text");
    }
  );

  auto schedule_cache = ScheduleCache::make_shared();

  // First test ADD

  // Add tasks and process
  Task::Ptr task1 = std::make_shared<Task>();
  task1->id = "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6";
  task1->type = "task_type";
  task1->start_time = Time::from_ISOtime("2025-01-22T02:15:00Z");
  task1->status = "ongoing";

  Task::Ptr task2 = std::make_shared<Task>();
  task2->id = "8a9f4819-bbd4-4550-acce-e061d3289973";
  task2->type = "task_type";
  task2->start_time = Time::from_ISOtime("2025-01-22T04:15:00Z");
  task2->status = "ongoing";

  Task::Ptr task3 = std::make_shared<Task>();
  task3->id = "ad0b5ee3-fd53-489b-a712-99d8b24161e8";
  task3->type = "task_type";
  task3->start_time = Time::from_ISOtime("2025-01-24T04:15:00Z");  // not in range
  task3->status = "ongoing";

  std::vector<Task::Ptr> tasks {task1, task2, task3};

  Process::Ptr process = std::make_shared<Process>();
  process->id = "13aa1c62-64ca-495d-a4b7-84de6a00f56a";
  process->graph.add_node("477425e8-a8a2-44bf-9a0d-a433ce4a5fe6");
  process->graph.add_node("8a9f4819-bbd4-4550-acce-e061d3289973");
  process->graph.add_edge(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "8a9f4819-bbd4-4550-acce-e061d3289973"
  );

  std::vector<Process::Ptr> processes{process};
  std::vector<ScheduleChangeRecord> records;
  std::string error;

  for (auto & task : tasks) {
    auto cache_action = Action::create(
      action_type::TASK_ADD,
      ActionPayload().task(task)
    );

    ASSERT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
    records.push_back(cache_action->record());
  }

  for (auto & process : processes) {
    auto cache_action = Action::create(
      action_type::PROCESS_ADD,
      ActionPayload().process(process)
    );

    EXPECT_TRUE(cache_action->validate(schedule_cache, error));
    cache_action->apply();
    records.push_back(cache_action->record());
  }

  TimeWindow time_window;
  time_window.start = Time::from_ISOtime("2025-01-21T10:00:00Z");
  time_window.end = Time::from_ISOtime("2025-01-22T10:00:00Z");

  EXPECT_TRUE(
    schedule_stream_->write_schedule(
      schedule_cache,
      records,
      error
  ));

  // Call 2 times
  EXPECT_EQ(2, transport_->get_request_count());

  // Check the request data
  std::vector<nlohmann::json> expected_data {
    // First request
    R"([{
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:event:477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
      "payload": {
        "type": "Property",
        "value": {
          "actual_end_time": null,
          "actual_start_time": null,
          "deadline": null,
          "description": null,
          "end_time": null,
          "estimated_duration": null,
          "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
          "planned_end_time": null,
          "planned_start_time": null,
          "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "resource_id": null,
          "series_id": null,
          "start_time": "2025-01-22T02:15:00Z",
          "status": "ongoing",
          "task_details": null,
          "type": "task_type"
        }
      },
      "startTime": {
        "type": "Property",
        "value": {
          "@type": "Datetime",
          "@value": "2025-01-22T02:15:00Z"
        }
      },
      "type": "urn:rmf2:event"
    },
    {
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:event:8a9f4819-bbd4-4550-acce-e061d3289973",
      "payload": {
        "type": "Property",
        "value": {
          "actual_end_time": null,
          "actual_start_time": null,
          "deadline": null,
          "description": null,
          "end_time": null,
          "estimated_duration": null,
          "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
          "planned_end_time": null,
          "planned_start_time": null,
          "process_id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "resource_id": null,
          "series_id": null,
          "start_time": "2025-01-22T04:15:00Z",
          "status": "ongoing",
          "task_details": null,
          "type": "task_type"
        }
      },
      "startTime": {
        "type": "Property",
        "value": {
          "@type": "Datetime",
          "@value": "2025-01-22T04:15:00Z"
        }
      },
      "type": "urn:rmf2:event"
    },
    {
        "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
        "id": "urn:event:ad0b5ee3-fd53-489b-a712-99d8b24161e8",
        "payload": {
            "type": "Property",
            "value": {
                "actual_end_time": null,
                "actual_start_time": null,
                "deadline": null,
                "description": null,
                "end_time": null,
                "estimated_duration": null,
                "id": "ad0b5ee3-fd53-489b-a712-99d8b24161e8",
                "planned_end_time": null,
                "planned_start_time": null,
                "process_id": null,
                "resource_id": null,
                "series_id": null,
                "start_time": "2025-01-24T04:15:00Z",
                "status": "ongoing",
                "task_details": null,
                "type": "task_type"
            }
        },
        "startTime": {
            "type": "Property",
            "value": {
                "@type": "Datetime",
                "@value": "2025-01-24T04:15:00Z"
            }
        },
        "type": "urn:rmf2:event"
    }])"_json,
    R"([{
      "@context": "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld",
      "id": "urn:process:13aa1c62-64ca-495d-a4b7-84de6a00f56a",
      "payload": {
        "type": "Property",
        "value": {
          "graph": [
            {
              "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
              "needs": []
            },
            {
              "id": "8a9f4819-bbd4-4550-acce-e061d3289973",
              "needs": [
                {
                  "id": "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
                  "type": "hard"
                }
              ]
            }
          ],
          "id": "13aa1c62-64ca-495d-a4b7-84de6a00f56a",
          "status": "",
          "current_events" : [],
          "process_details" : null
        }
      },
      "type": "urn:rmf2:process"
    }])"_json
  };
  EXPECT_EQ(
    request_bodies,
    expected_data
  );

  // Test Update
  // clear records
  request_bodies.clear();
  records.clear();

  // Update task in cache
  task1->duration = Duration::from_seconds(600);

  // Update process in cache
  task1->duration = Duration::from_seconds(600);
}
