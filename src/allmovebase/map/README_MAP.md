# 地图使用说明

The navigation stack now uses two maps at the same time:

- `map.yaml`
  - Publishes to `/map`.
  - Used by `move_base` global costmap/static layer.
  - Current image: `arena_nav_manual_3.pgm`.
  - Contains physical obstacles plus 2D no-crossing boundaries/competition lines.

- `map_amcl.yaml`
  - Publishes to `/amcl_map`.
  - Used only by AMCL.
  - Current image: `arena_amcl_manual_1.pgm`.
  - Contains only physical obstacles that can be observed by depth/scan.

Legacy/special map:

- `map_avoidance.yaml`
  - Current image: `arena_avoidance_manual_1.pgm`.
  - Kept for small obstacle-zone experiments; it is no longer the default map
    used by `map_server.launch`.

PGM notes:

- `arena_nav_manual_1.pgm`, `arena_nav_manual_2.pgm`, and
  `arena_nav_manual_3.pgm` are navigation/rule maps.
- `arena_amcl_manual_1.pgm` is the localization map for AMCL.
- All default competition maps use `resolution: 0.05` and `origin: [0, 0, 0]`.
