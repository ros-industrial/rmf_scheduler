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

from rmf2_scheduler.data import Time
from datetime import datetime, timedelta


def test_empty_init():
    t = Time()
    assert t.seconds() == 0
    assert t.nanoseconds() == 0


def test_init():
    # Seconds and nanoseconds
    t = Time(1, 2 * 1000 * 1000)
    assert t.seconds() == 1.002
    assert t.nanoseconds() == 1002 * 1000 * 1000

    # Nanoseconds only
    t = Time(5 * 1000 * 1000)
    assert t.seconds() == 0.005
    assert t.nanoseconds() == 5 * 1000 * 1000


def test_init_datetime():
    # Seconds and nanoseconds
    t_datetime = datetime(year=2024, month=6, day=3, hour=23, minute=2, second=30)
    t = Time(t_datetime)
    assert t.seconds() == t_datetime.timestamp()


def test_localtime():
    # Seconds and nanoseconds
    t_datetime = datetime(year=2024, month=6, day=3, hour=23, minute=2, second=30)
    t = Time(t_datetime)
    assert t.to_localtime() == "Jun 03 23:02:30 2024"


def test_isotime():
    # Seconds and nanoseconds
    t_datetime = datetime(year=2024, month=6, day=3, hour=23, minute=2, second=30)
    t = Time(t_datetime)
    assert t.to_ISOtime() == "2024-06-03T23:02:30Z"
