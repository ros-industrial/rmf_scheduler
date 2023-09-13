^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ortools_vendor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

9.6.2534
--------
* Rework identifier and cp solver to use event detail (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!19 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/19>`_)

  * Update source build path and add in git ignore
* Add CpSolver deconfliction capability (`ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler!16 <https://gitlab.com/ROSI-AP/rosi-ap_commercial/cag/rmf_scheduler/-/merge_requests/16>`_)

  * Apply patch for absl to solve sanitizer build error
  * Use prebuilt binary for ortools vendor by default
  * initial implementation for scheduler deconfliction
* Contributors: Chen Bainian, Lum Kai Wen
