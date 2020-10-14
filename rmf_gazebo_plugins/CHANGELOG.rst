^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_gazebo_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2020-09-23)
------------------
* Support cross-compiling with ROS 2 Foxy and Eloquent. [#103](https://github.com/osrf/rmf_demos/pull/103)
* TeleportIngestorPlugin uses rmf_ingestor_msgs. [#117](https://github.com/osrf/rmf_demos/pull/117)
* Modularized Readonly TeleportIngestorPlugin and TeleportDispenserPlugin to support both Gazebo and Ignition simulations. [#124, #134](https://github.com/osrf/rmf_demos/pull/124) 
* Contributors: Aaron Chong, Boon Han, Michael X. Grey, Yadu, mrushyendra

1.0.0 (2020-06-24)
------------------
* Provides the `readonly` Gazebo plugin that emulates the behavior of a `read_only` fleet type. When attached to a steerable vehicle that operates on a known traffic graph, the plugin publishes a prediction of the vehicle's intended path to the `rmf traffic database` via its configured `read_only_fleet_adapter`. Other fleets operating in the space can plan routes to avoid the path of this vehicle.
* Provides `TeleportDispenser` Gazebo plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Gazebo along side a "payload" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestor` Gazebo plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Contributors: Aaron, Aaron Chong, Boon Han, Charayaphan Nakorn Boon Han, Luca Della Vedova, Yadu, Yadunund
