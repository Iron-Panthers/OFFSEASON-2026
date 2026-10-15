# Coprocessor

Code that runs **off the robot**, on the vision coprocessor.

Nothing here imports WPILib or knows it is in a simulation. The same files run on the Rubik Pi
against a real camera; only `--source` changes. That is the whole point of the arrangement — what
you debug in simulation is the program that ships, not a stand-in for it.

See [`docs/coprocessors.md`](../docs/coprocessors.md) for running it against the simulator.

## Standalone

```bash
python -m coproc objdetect \
  --source http://localhost:1193/stream.mjpg \
  --camera 2 --nt localhost --output-port 1293
```

Omit `--nt` to skip NetworkTables. Omit `--output-port` to skip the annotated stream.

## Install

```bash
python -m venv .venv
.venv/Scripts/pip install -r requirements.txt     # Windows
.venv/bin/pip install -r requirements.txt         # Linux/macOS
```

The simulation launcher finds `.venv` on its own.

## Layout

| Path | What it is |
| --- | --- |
| `coproc/runtime/` | The framework. Stream in, stream out, NetworkTables, camera geometry. |
| `coproc/modules/` | One directory per module. |
| `coproc/__main__.py` | Entry point and the module registry. |
| `tests/` | Runs without a model, a camera or a network. |

## Adding a module

1. Add a directory under `coproc/modules/` with a class extending `CoprocessorModule`.
   Implement `add_arguments`, `setup` and `process`; optionally `annotate`.
2. Register it in `MODULES` in `coproc/__main__.py`.
3. Add an entry to `CoprocessorModule` in the Java launcher so the simulation can start it.

Frames, reconnects, NetworkTables, the annotated stream and timing all come from the runtime.

**Import heavy dependencies inside `setup`, not at module scope.** Listing modules and running the
tests must not require a Torch install.

## Tests

```bash
python -m pytest tests/ -q
```

The geometry tests check the estimator against an exact projection of the sphere's silhouette
rather than against a second approximation — testing an approximation against a matching one
agrees with itself and proves nothing.
