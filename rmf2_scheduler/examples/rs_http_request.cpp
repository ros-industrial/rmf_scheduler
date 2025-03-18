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

#include <cstdio>
#include <iostream>

#include "rmf2_scheduler/data/event.hpp"
#include "rmf2_scheduler/data/json_serializer.hpp"
#include "rmf2_scheduler/http/transport.hpp"
#include "rmf2_scheduler/http/request.hpp"
#include "rmf2_scheduler/log.hpp"
#include "rmf2_scheduler/utils/url_utils.hpp"

std::string to_context_broker_body(
  rmf2_scheduler::data::Event::Ptr event
)
{
  // Create Start time lookup
  nlohmann::json start_time;
  start_time["@type"] = "Datetime";
  start_time["@value"] = nlohmann::json(event->start_time);
  nlohmann::json start_time_lookup;
  start_time_lookup["type"] = "Property";
  start_time_lookup["value"] = start_time;

  // Payload
  nlohmann::json payload;
  payload["type"] = "Property";
  payload["value"] = nlohmann::json(*event);

  // Body
  nlohmann::json json_body;
  json_body["id"] = "urn:event:" + event->id;
  json_body["type"] = "urn:rmf2:event";
  json_body["startTime"] = start_time_lookup;
  json_body["payload"] = payload;
  json_body["@context"] = "https://uri.etsi.org/ngsi-ld/v1/ngsi-ld-core-context-v1.7.jsonld";

  nlohmann::json body = nlohmann::json(std::vector<nlohmann::json>{json_body});

  return body.dump();
}

int main()
{
  rmf2_scheduler::log::setLogLevel(rmf2_scheduler::LogLevel::INFO);

  auto transport = rmf2_scheduler::http::Transport::create_default();

  // Context broker URL
  std::string context_broker_url = "http://localhost:9090";
  std::string context_broker_path = "ngsi-ld";

  // Create URL
  std::string endpoint_url = rmf2_scheduler::url::combine_multiple(
    context_broker_url,
    {context_broker_path, "v1", "entities"}
  );

  std::string error;

  auto event = std::make_shared<rmf2_scheduler::data::Event>(
    "477425e8-a8a2-44bf-9a0d-a433ce4a5fe6",
    "event_type",
    rmf2_scheduler::data::Time(1672625700, 0)
  );
  event->duration = std::chrono::minutes(4);
  event->process_id = "process_1";

  auto add_request = std::make_shared<rmf2_scheduler::http::Request>(
    rmf2_scheduler::url::combine_multiple(
      context_broker_url,
      {context_broker_path, "v1", "entityOperations", "update"}
    ),
    rmf2_scheduler::http::request_type::kPost,
    transport
  );
  add_request->set_content_type("application/ld+json");
  add_request->add_request_body(to_context_broker_body(event), error);

  auto add_response = add_request->get_response_and_block(error);

  auto query_request = std::make_shared<rmf2_scheduler::http::Request>(
    rmf2_scheduler::url::append_query_params(
      endpoint_url,
  {      // Query parameters
    {"type", "urn:rmf2:event"},
    {
      "q",
      "startTime>=2021-01-11T14:45:00Z;"
      "startTime<=2021-01-18T14:45:00Z"
    }
  }
    ),
    rmf2_scheduler::http::request_type::kGet,
    transport
  );

  auto query_response = query_request->get_response_and_block(error);
  std::cout << (query_response->is_successful() ? "Success" : "Failed") << std::endl;
  std::cout << "Status code: " << query_response->get_status_code() << std::endl;
  std::cout << "Status text: " << query_response->get_status_text() << std::endl;
  std::cout << "Content type: " << query_response->get_content_type() << std::endl;
  std::cout << "Data Received: " << query_response->extract_data_stream() << std::endl;
  std::cout << "Header - Link: " << query_response->get_header("Link") << std::endl;
}
