^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_scheduler_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.4
-----
* Update schemas for estimate robot task request. (`#12 <https://github.com/ros-industrial/rmf_scheduler/pull/12>`_)
* Fix wrong task state update topic (`#15 <https://github.com/ros-industrial/rmf_scheduler/pull/15>`_)
* Contributors: Lum Kai Wen

0.2.3
-----
* Add task state health watchdog capability to exeuction client. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!31 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/31>`_)
* Contributors: Lum Kai Wen

0.2.2
-----
* Fix race conditions when accessing unordered map. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!36 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/36>`_)
* Contributors: Lum Kai Wen

0.2.1
-----
* Add feature to pause, resume and cancel ongoing tasks. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!34 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/34>`_)
* Contributors: Lum Kai Wen

0.2.0
-----
* Add in battery allocation prototype and fixes for observer (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!32 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/32>`_)
* Get task id from task state json instead of ros2 message's request id. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!29 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/29>`_)
* Contributors: Chen Bainian, Lum Kai Wen

0.1.0
-----
* Add in functional estimation and executor plug-ins (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!23 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/23>`_)
* Add runtime and estimation interface for scheduler and rework static tasking api (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!15 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/15>`_)
* Implement Dispatching Capabilities to Client (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!10 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/10>`_)
* Wrap JSON response with future (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!8 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/8>`_)
* Create CPP task estimate client (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!4 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/4>`_)
* Contributors: Chen Bainian, Lum Kai Wen, Santosh Balaji Selvaraj
