# relnav requirements and verification

Each requirement from the [design spec](specs/2026-10-02-relnav-design.md) maps to the test
or procedure that verifies it.

| ID | Requirement | Verified by |
|---|---|---|
| REQ-ACC-01 | Pin-to-pin distance error < 3 cm for pins 1–3 m from the camera | Field test ([field-test.md](field-test.md)) with `relnav verify`; logic in `tests/unit/test_verify.py::test_pairs_pass_and_fail` |
| REQ-ACC-02 | Re-observed pin moves < 5 cm after a ~10 m closed loop | Field test step 5 ([field-test.md](field-test.md)) |
| REQ-ACC-03 | Synthetic trajectory RMSE < 2 cm | `tests/synthetic/test_pipeline.py::test_trajectory_error_under_2_cm` |
| REQ-PERF-01 | Live processing mean ≥ 25 fps on the M1 Pro at 848×480 | `fps` in `results/<run>/timing.json` from a replay of a real recording (field test step 4) |
| REQ-REL-01 | Frame timeouts and failed starts never crash the app | `tests/unit/test_runner.py::test_timeouts_are_retried`, `tests/unit/test_runner.py::test_source_error_stops_cleanly`, `tests/unit/test_realsense.py::test_start_gives_up_with_a_clear_error` |
| REQ-REL-02 | LOST is reported, and tracking recovers on a mapped area | `tests/synthetic/test_pipeline.py::test_lost_then_recovers` |
| REQ-REP-01 | Same recording replays to identical trajectory and pins | `tests/integration/test_cli.py::test_replay_is_deterministic`, `tests/synthetic/test_pipeline.py::test_same_input_gives_same_output` |
| REQ-ARCH-01 | Core and domain never import adapters, app, pyrealsense2, rerun, mcap or typer | `tests/test_architecture.py::test_import_contracts_hold` (import-linter) |
