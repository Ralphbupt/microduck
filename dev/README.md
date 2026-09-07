# Laptop params files

`robotd --params` files for running the daemon on a laptop against the MuJoCo plant
(`docs/design/sim-backend-design.md`). They point the `[policy]` paths at a `microduck_rl`
checkout's `policies/` directory — edit the absolute paths for your machine. Nothing here is
shipped; a robot never reads these.

- `robotd-mac.toml` — walk mode, the alpha policies.
- `robotd-mac-roller.toml` — `mode = "roller"`, `roller.onnx` as the gait.
