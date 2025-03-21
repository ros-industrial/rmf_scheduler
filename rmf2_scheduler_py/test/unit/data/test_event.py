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

from datetime import datetime, timedelta

from rmf2_scheduler.data import Event, Time


def test_empty_init():
    event = Event()
    assert event.id == ''
    assert event.type == ''
    assert event.description is None
    assert event.start_time == Time(0)
    assert event.duration is None
    assert event.series_id is None
    assert event.process_id is None


def test_minimal():
    event = Event(
        id='4321e2c1-71f7-42ef-a10d-9a3b3f4241ff',
        type='go_to_place',
        start_time=Time.from_ISOtime('2024-06-03T23:02:30Z'),
    )
    assert event.id == '4321e2c1-71f7-42ef-a10d-9a3b3f4241ff'
    assert event.type == 'go_to_place'
    assert event.description is None
    assert event.start_time.to_ISOtime() == '2024-06-03T23:02:30Z'
    assert event.duration is None
    assert event.series_id is None
    assert event.process_id is None
