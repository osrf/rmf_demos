^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-24)
------------------
* Merge pull request `#75 <https://github.com/osrf/rmf_demos/issues/75>`_ from osrf/develop
  Develop
* RMF Linter
* Feature/non intrusive dispenser (`#69 <https://github.com/osrf/rmf_demos/issues/69>`_)
  * finished first iteration of TeleportDispenser, crashing at world start
  * ingestor working, need to figure out how to teleport it back to dispenser
  * TeleportIngestor working with office and airport demos, removed old teleport plugin
  * removed old teleport plugin, removed odd vscode artifact
  * minor cleanup, const correctness
  * removed delay before dispensing, made respawn by ingestor 10 seconds
  * made function names more verbose and const qualify variables
* Merge remote-tracking branch 'origin/master' into new_interface
* Merge pull request `#68 <https://github.com/osrf/rmf_demos/issues/68>`_ from osrf/fix/airport_scenarios
  Fix/airport scenarios
* Merge remote-tracking branch 'origin/master' into develop_compability
* Merge pull request `#63 <https://github.com/osrf/rmf_demos/issues/63>`_ from osrf/feature/airport-dispenser
  added new respawn seconds sdf element, added delivery scenario
* Added delay before loading and unloading
* added new respawn seconds sdf element, added delivery scenario
* Feature/teleport plugin v2 (`#62 <https://github.com/osrf/rmf_demos/issues/62>`_)
  * experimenting with placeholder links on models, refactoring teleport to have dispenser stuff
  * no need for mutexes
  * fixed RobotPlaceholder model, teleport plugin works, now v3 to remove target name
  * extended respawn to 5 seconds, caching modelptrs instead of names
  * handle duplicate dispenser request guids
  * cleaned up dispenser script and usage
  * removed launch call for dispensers
* Minor enhancement to readonly plugin (`#52 <https://github.com/osrf/rmf_demos/issues/52>`_)
  * POC merge lane
  * POC merge lane
  * Added lane_threshold to control merging of robot into its lane
  * Spawning caddy without launching airport_terminal world
  * Format fix
* Merge branch 'master' of https://github.com/osrf/rmf_demos
* Merge pull request `#43 <https://github.com/osrf/rmf_demos/issues/43>`_ from cnboonhan94/reset_dispense_payload_visuals
  implement visual reset of coke can in 2s via fixed initial pose
* Update teleport.cpp
* Update rmf_gazebo_plugins/src/teleport.cpp
  Co-Authored-By: Yadu <yadunund@gmail.com>
* implement visual reset of coke can in 2s via fixed initial pose
* Add gazebo plugins to rosdep dependencies (`#35 <https://github.com/osrf/rmf_demos/issues/35>`_)
  * Add gazebo plugins to rosdep dependencies
  * Remove unneeded dependencies
* Feature/office single doors (`#31 <https://github.com/osrf/rmf_demos/issues/31>`_)
  * using building_gazebo_plugins instead, cleaned up, WIP testing hinged doors
  * removed doors in airport first
* Feature/caddy track (`#27 <https://github.com/osrf/rmf_demos/issues/27>`_)
  * reduced shop size, reduced number of floor models
  * made the stall islands smaller, WIP adding caddy lanes
  * tidied up the floor tiles, chose texture, added caddy track
  * cleaned up world, removed doors, WIP figure out why doors are not rendering
  * allow plugin to handle single doors, testing .building.yaml
  * testing minor door stuff
  * removed door testing stuff, ready to cleanup and merge
  * cleanup, removed door testing stuff
  * changing the screenshots
  * renewed docs
  * change build test to use rosdep
  * corrected build test
  * rosdep update
  * skipping some keys
  * skipping libgazebo9 in rosdep
  * adding sudo to rosdep command, removing skip keys
  * added skip keys
  * continue installing despite errors
  * removed gazebo installation step, updted readme as debug notes
  * adding ignore keys to readme
* Merge pull request `#26 <https://github.com/osrf/rmf_demos/issues/26>`_ from osrf/feature/caddy
  Feature/caddy
* Fixes
* Fixes
* Cleanup
* Plugin now predicts look_ahead waypoints
* Modified caddy drive parameters and introduced rmf_traffic dependency
* Plugin adds nearest waypoint in direction of caddy to its path
* Fixed graph initialization
* Initialize start waypoint
* Added caddy model with readonly plugin
* Merge pull request `#17 <https://github.com/osrf/rmf_demos/issues/17>`_ from osrf/rviz_plugin
  Rviz plugin
* Created layout
* Merge branch 'master' into feature/fix-readme
* Fixed merge conflict
* Load and unload of can
* Added teleport plugin to coke model
* Added coke model
* Fixed merge conflict
* Fixed linker error with door plugin
* Fixed CMake
* Added rmf_gazebo_plugins
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Luca Della Vedova, Yadu, Yadunund
