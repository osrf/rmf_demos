^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_ignition_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2021-01-06)
------------------
* Provides `TeleportDispenserPlugin` Ignition plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Ignition Gazebo along side a "payload" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestorPlugin` Ignition plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Ignition Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_ingestor_msgs::IngestorRequest` message with `target_guid` matching its model name.
* Provides `ReadonlyPlugin` Readonly plugin that emulates the behavior of a `read_only` fleet type. When attached to a steerable vehicle that operates on a known traffic graph, the plugin publishes a prediction of the vehicle's intended path to the `rmf traffic database` via its configured `read_only_fleet_adapter`. Other fleets operating in the space can plan routes to avoid the path of this vehicle.
* Contributors: Aaron Chong, Luca Della Vedova, Yadunund, Rushyendra Maganty
