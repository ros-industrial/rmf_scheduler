# Copyright 2025 ROS Industrial Consortium Asia Pacific
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rmf2_scheduler.data import Edge, Graph, Process


def test_process():
    process = Process()
    process.id = '4321e2c1-71f7-42ef-a10d-9a3b3f4241ff'
    process.graph.add_node('task_1')
    process.graph.add_node('task_2')
    process.graph.add_node('task_3')
    process.graph.add_node('task_4')
    process.graph.add_node('task_5')

    process.graph.add_edge('task_1', 'task_3')
    process.graph.add_edge('task_2', 'task_3')
    process.graph.add_edge('task_3', 'task_4', Edge('soft'))
    process.graph.add_edge('task_3', 'task_5')

    assert (
        process.graph.dump()
        == """digraph DAG {
  "task_1" -> "task_3" [label=hard];
  "task_2" -> "task_3" [label=hard];
  "task_3" -> "task_4" [label=soft];
  "task_3" -> "task_5" [label=hard];
}
"""
    )
    assert process.id == '4321e2c1-71f7-42ef-a10d-9a3b3f4241ff'
    assert process.graph.get_node('task_3').inbound_edges() == {
        'task_1': Edge('hard'),
        'task_2': Edge('hard'),
    }
    assert process.graph.get_node('task_3').outbound_edges() == {
        'task_4': Edge('soft'),
        'task_5': Edge('hard'),
    }

    expected_graph_json = [
        {'id': 'task_1', 'needs': []},
        {'id': 'task_2', 'needs': []},
        {
            'id': 'task_3',
            'needs': [
                {'id': 'task_1', 'type': 'hard'},
                {'id': 'task_2', 'type': 'hard'},
            ],
        },
        {'id': 'task_4', 'needs': [{'id': 'task_3', 'type': 'soft'}]},
        {'id': 'task_5', 'needs': [{'id': 'task_3', 'type': 'hard'}]},
    ]
    assert process.graph.json() == expected_graph_json
    expected_process_json = {'id': process.id,
                             'graph': expected_graph_json,
                             'process_details': None,
                             'status': '',
                             'current_events': []}
    assert process.json() == expected_process_json

    duplicate_process = Process.from_json(expected_process_json)

    assert duplicate_process == process
