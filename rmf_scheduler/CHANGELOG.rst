^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_scheduler
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.2
-----

0.2.1
-----
* Fix cache filename buffer overflow (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!35 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/35>`_)
* Add feature to pause, resume and cancel ongoing tasks. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!34 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/34>`_)
* Contributors: Chen Bainian, Lum Kai Wen

0.2.0
-----
* Refactor to get charger info from config file (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!33 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/33>`_)
* Add in battery allocation prototype and fixes for observer (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!32 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/32>`_)
* Use ros parameters for plugin loading instead (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!27 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/27>`_)
* Add new schemas for updating entire series. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!25 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/25>`_)
* Contributors: Chen Bainian, Lum Kai Wen

0.1.0
-----
* Add in functional estimation and executor plug-ins (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!23 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/23>`_)
* Add series id to event in dag series if not populated. (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!22 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/22>`_)
* Prevent change of event start time in observer callback (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!20 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/20>`_)
* Add feature for updating via event id and updated timing (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!18 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/18>`_)
* Rework identifier and cp solver to use event detail (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!19 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/19>`_)
* Add CpSolver deconfliction capability (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!16 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/16>`_)
* Add runtime and estimation interface for scheduler and rework static tasking api (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!15 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/15>`_)
* Create rmf scheduler api msgs interface class (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!14 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/14>`_)
* Add scheduler static tasking, add, get and update (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!13 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/13>`_)
*  Preliminary version for conflict resolver (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!12 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/12>`_)
* Add series data structure (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!11 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/11>`_)
* Add dependency graph and execution support (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!6 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/6>`_)
* Export as shared library
* Add system time executor (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!2 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/2>`_)
* Additional changes for first pass event and event handler (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!3 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/3>`_)
* Add first event and event handler (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!1 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/1>`_)
* Contributors: Chen Bainian, Lum Kai Wen, Santosh Balaji Selvaraj
