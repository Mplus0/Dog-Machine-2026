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
  - Current image: `arena_amcl_fence_v2.pgm`.
  - Default for the current 6 m x 6 m fenced test field after Gate6 validation.
  - Contains the three observable boxes and the physical fence with the
    lower-right L-shaped opening.

- `map_amcl_fence_v2.yaml`
  - Explicit alias for the same Gate6 map currently selected by `map_amcl.yaml`.
  - Preserves the three measured boxes from `arena_amcl_manual_1.pgm` and
    replaces its legacy thick partial-top block with the measured physical fence.
  - The lower boundary ends at x=4.70 m and the right boundary starts at
    y=1.65 m, leaving the measured lower-right L-shaped opening.
  - Generated reproducibly by `../tools/generate_amcl_fence_map.py`.

Legacy/special map:

- `map_avoidance.yaml`
  - Current image: `arena_avoidance_manual_1.pgm`.
  - Kept for small obstacle-zone experiments; it is no longer the default map
    used by `map_server.launch`.

PGM notes:

- `arena_nav_manual_1.pgm`, `arena_nav_manual_2.pgm`, and
  `arena_nav_manual_3.pgm` are navigation/rule maps.
- `arena_amcl_manual_1.pgm` is the retained pre-fence AMCL rollback map.
- `arena_amcl_fence_v2.pgm` is the current-field default AMCL map.
- All default competition maps use `resolution: 0.05` and `origin: [0, 0, 0]`.
