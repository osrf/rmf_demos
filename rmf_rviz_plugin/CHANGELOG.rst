^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_rviz_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

1.0.0 (2020-06-24)
------------------
* Develop (`#85 <https://github.com/osrf/rmf_demos/issues/85>`_)
  * Updated request_delivery with pickup and dropoff dispener names
  * Fixed dangling waypoint
  * Added newly migrated Door panels
  * Reverted some automatic changes when updating configs
  * Adding marker subscriber for /building_systems_markers
  * Fixes for first release (`#84 <https://github.com/osrf/rmf_demos/issues/84>`_)
  * Removed workcell names from maps and added delivery launch files
  * Updated RMF panel with new Delivery message structure. Updated rviz configs
  * Linted with pycodestyle
  * Updated build.yaml
  * RMF linter
  * Fixed EOL error
  * Updated README.md
  Co-authored-by: Yadunund <yadunund@openrobotics.org>
* Merge pull request `#75 <https://github.com/osrf/rmf_demos/issues/75>`_ from osrf/develop
  Develop
* Fixed CMake warnings
* RMF Linter
* Updated API call usage in RMFPanel
* Merge pull request `#45 <https://github.com/osrf/rmf_demos/issues/45>`_ from cnboonhan94/new_interface
  Upgrade to RMF Panel
* Merge remote-tracking branch 'origin/master' into new_interface
* Merge pull request `#68 <https://github.com/osrf/rmf_demos/issues/68>`_ from osrf/fix/airport_scenarios
  Fix/airport scenarios
* refactor header namespace pollution
* refactor header namespace pollution
* fix mutex deadlock issue and header signature bug
* locking everything with a vengeance
* revert to work with master branch
* refactor to namespace using definitions more obsecurely
* add mutex lock
* removing unnecessary non-const
* remove unnecessary un-constness
* remove unnecessary parameter
* stricter constness
* resolve header guard misnaming
* fix building error from profile api change
* handle empty waypoint inputs by ignoring them
* handle read-only fleet adapters
* add helpful message when fleet_adapter loader service cannot be found
* add fleet name to loop messages
* remove dangling connections
* remove robot selector
* move pause plan checkbox
* move workcell only and update time checkboxes
* move emergency button
* remove pause and resume buttons and corresponding plumbing
* set default loop count to 1
* uncheck pause_plan checkbox by default
* Merge branch 'new_interface' of github.com:cnboonhan94/rmf_demos into new_interface
* removing unused topic names
* Update rmf_rviz_plugin/package.xml
  removing whitespace
  Co-Authored-By: Yadu <yadunund@gmail.com>
* Update rmf_rviz_plugin/package.xml
  removing whitespace
  Co-Authored-By: Yadu <yadunund@gmail.com>
* Added emergency stop option in RmfPanel
* Update ParseActionPlan.cpp
* brave attempt to salvage formatting :(
* merge yaml-loading
* rename schedule to plan to avoid terminology conflict; implement loading of action plan from yaml
* add loading of yaml file from gui
* add gui boilerplate for loading yaml; messed up formatting
* add sample yaml
* add template header for action plan
* add emergency status
* add inline autocomplete for comboboxes
* implement checkbox options
* update package.xml dependencies
* implemented everything in one go, wow
* implement delivery
* implement callback for fleet state
* finish fleshing out gui components
* add header definitions for GUI components
* clean slate
* Merge pull request `#33 <https://github.com/osrf/rmf_demos/issues/33>`_ from osrf/fix/rviz_plugin
  Fixed warnings and extended load/save function
* Fixed warnings and extended load/save function
* Merge pull request `#17 <https://github.com/osrf/rmf_demos/issues/17>`_ from osrf/rviz_plugin
  Rviz plugin
* Fixed format
* Updated office.rviz
* Implemented grid layout
* Modified layout
* Edited layouts
* Added loop request
* Fixed update of text fields
* Renamed files and completed internal function definitions
* Delivery request working
* Added ros node
* Created layout
* Updated dependency
* Added source and header files
* Added skeleton
* Contributors: Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Yadu, Yadunund
