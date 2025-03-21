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

import matplotlib.pyplot as plt

from rmf2_scheduler.cache import ScheduleCache
from rmf2_scheduler.data import Time, TimeWindow
from rmf2_scheduler.storage import ScheduleStream


def main():
    stream = ScheduleStream.create_default('http://localhost:9090/ngsi-ld')

    cache = ScheduleCache()
    time_window = TimeWindow()
    time_window.start = Time.from_ISOtime('2025-01-01T10:15:00Z')
    time_window.end = Time.from_ISOtime('2025-01-25T18:10:00Z')

    result, error = stream.read_schedule(cache, time_window)

    if not result:
        print(error)

    all_tasks = cache.get_all_tasks()

    print('Total tasks: %d' % len(all_tasks))

    for task in all_tasks:
        print('Task json %s' % task.json())

    # All Tasks
    task_duration = []
    for task in all_tasks:
        task_duration.append(task.duration.seconds())

    plt.hist(task_duration, bins=20, color='skyblue', edgecolor='black')
    print(task_duration)

    plt.title('All Task Duration')
    plt.xlabel('Task Duration (s)')
    plt.ylabel('Frequency')
    plt.show()

    # AMR1 tasks
    plt.clf()
    task_duration = []
    for task in all_tasks:
        if task.resource_id == 'amr_1':
            task_duration.append(task.duration.seconds())

    plt.hist(task_duration, bins=20, color='blue', edgecolor='black')
    plt.title('AMR 1 Task Duration')
    plt.xlabel('Task Duration (s)')
    plt.ylabel('Frequency')
    plt.show()

    # AMR2 tasks
    plt.clf()
    task_duration = []
    for task in all_tasks:
        if task.resource_id == 'amr_2':
            task_duration.append(task.duration.seconds())

    plt.hist(task_duration, bins=20, color='wheat', edgecolor='black')
    plt.title('AMR 2 Task Duration')
    plt.xlabel('Task Duration (s)')
    plt.ylabel('Frequency')
    plt.show()

    # Forklift tasks
    plt.clf()
    task_duration = []
    for task in all_tasks:
        if task.resource_id == 'forklift_1':
            task_duration.append(task.duration.seconds())

    plt.hist(task_duration, bins=20, color='violet', edgecolor='black')
    plt.title('Forklift 1 Task Duration')
    plt.xlabel('Task Duration (s)')
    plt.ylabel('Frequency')
    plt.show()

    # Palletization tasks
    plt.clf()
    task_duration = []
    for task in all_tasks:
        if task.resource_id == 'palletizer_1':
            task_duration.append(task.duration.seconds())

    plt.hist(task_duration, bins=10, color='springgreen', edgecolor='black')
    plt.title('Palletization Task Duration')
    plt.xlabel('Task Duration (s)')
    plt.ylabel('Frequency')
    plt.show()


if __name__ == '__main__':
    main()
