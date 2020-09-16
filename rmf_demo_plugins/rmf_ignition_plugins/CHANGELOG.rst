^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rmf_ignition_plugins
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Provides `TeleportDispenserPlugin` Ingestor plugin that is attached to the `TeleportDispenser` model in `rmf_demo_assets`. When loaded into Ignition Gazebo along side a "payload" model, the plugin will teleport the payload onto the nearest robot when it registers a `rmf_dispenser_msgs::DispenserRequest` message with `target_guid` matching its model name.
* Provides `TeleportIngestorPlugin` Ingestor plugin that is attached to the `TeleportIngestor` model in `rmf_demo_assets`. When loaded into Ignition Gazebo, the plugin will teleport the payload off the nearest robot and onto its location, when it registers a `rmf_ingestor_msgs::IngestorRequest` message with `target_guid` matching its model name.
* Contributors: Aaron Chong, Luca Della Vedova, Yadunund, Rushyendra Maganty
