# Guide for managing changes in rmf2_scheduler for this python package

This guide contains instructions on generating stubs and running `isort` when making changes on the `rmf2_scheduler` pacakge which requires updating the `rmf2_scheduler_py` package.


## Updating openapi.json for the client
When changes are made to the `rmf2_scheduler`'s `router` module or structs from `rmf2_scheduler::data` are changed, the following changes would be required.
1. Add the changes in the python bindings
2. Rebuild the `rmf2_scheduler` and `rmf2_scheduler_py` packages
3. Remove the existing `openapi.json` file
4. Regenerate the `openapi.json` file by calling `python3 ./script/dump_openapi.py` from this current directory i.e(rmf2_scheduler/rmf2_scheduler_py)
5. Generate the stubs by calling `./script.generate_stubs.sh` from this directory.
6. Call isort to reorder the imports of the regenerated stubs by calling `isort -dt --multi-line 3 --ext pyi ./rmf2_scheduler/` from this directory

### Changes to tests
Changes to the structs in `scheduler::data` would require changes to the tests in both `rmf2_scheduler` and `rmfs_scheduler_py` packages. Additionally, resources would need to be regenerated, for the test resources in `rmf2_scheduler/test/resources`.
1. Update all the test variables that involve the udpated structs from `rmf2_scheduler::data` in `rmf2_scheduler/test` and `rmf2_scheduler_py/test`.
2. Regenerate the sample backups by running the command `ros2 rmf2_scheduler create_sample_backups`, which would generate files in `~/.cache/`.
3. copy the generated files in `~/.cache/sample_r2ts_backups/event_only` and `~/.cache/sample_r2ts_backups/event_and_process` into `rmf2_scheduler/test/resources/simple_storage`
4. Remove the git-ignored folders with the `tmp-` prefixes in the folder to avoid confusion and to distinguish the generated files and folders in the next step.
5. Run `colcon test --packages-select rmf2_scheduler` to generate the resources.
6. Run the command `colcon-result --verbose` to the corresponding generated files for the `sample-write-schedule-time-window` and `sample-write-schedule-record` folder and replace the files accordingly.