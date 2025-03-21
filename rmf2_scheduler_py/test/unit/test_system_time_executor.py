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

from datetime import datetime
from threading import Thread
from time import sleep

from rmf2_scheduler import SystemTimeAction, SystemTimeExecutor
from rmf2_scheduler.data import Duration, Time


def dummy_action():
    print('action done at %s' % Time(datetime.now()).to_ISOtime())


def test_system_time_executor():
    ste = SystemTimeExecutor()

    # Start spinning right away
    thr = Thread(target=ste.spin)
    thr.start()

    # Create an action
    action = SystemTimeAction()
    action.time = Time(datetime.now()) + Duration.from_seconds(2)
    action.work = dummy_action
    print('action scheduled at %s' % action.time.to_ISOtime())

    ste.add_action(action)

    # sleep 3 seconds
    print('Running...')
    sleep(3)

    ste.stop()
    thr.join()
