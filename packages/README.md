# Shared pure-Python packages

`packages/jcheng_core` is the single source for the primitives that were
previously copied between Studio, `armpi_analysis` and the phase-2/3 tools:

- `jcheng_stats` — paired-trial mean/percentile/bootstrap/binomial helpers
  (stdlib only);
- `jcheng_vision` — voxel components, top-slab fit and cluster selection
  (NumPy only).

Both packages are intentionally free of ROS, FastAPI, adapters or filesystem
side effects. They are versioned together as `jcheng-core`.

## Consumers

| Consumer | How it loads the package |
|---|---|
| Studio backend | `pip install -e ../../packages/jcheng_core --no-deps` in the backend venv (after `pip install -e '.[dev]'`); CI does the same |
| `tools/phase3` | `tools/phase3/_bootstrap.py` adds `packages/jcheng_core` to `sys.path` |
| `tools/tests` / pure CI | `PYTHONPATH=packages/jcheng_core:...` (see `python-ci.yml`) |
| `armpi_analysis` | same `PYTHONPATH` / bootstrap entry; the ROS package itself is pure Python |
| `tools/phase2/object_segment_rgbd.py` | adds the checkout path when run from the repo; for a copied container deployment, copy `packages/jcheng_core/jcheng_vision` beside the tool or set `PYTHONPATH` |

The backend `pyproject.toml` does **not** declare a relative `file:` URL
(setuptools rejects it); the explicit editable install is part of the backend
setup and CI.

## Rules

1. Keep `jcheng_stats` stdlib-only; keep `jcheng_vision` NumPy-only.
2. Do not move ROS message types, gateway HTTP or FastAPI code here.
3. Add a test in `tools/tests/test_shared.py` for every new primitive.
4. Bump `jcheng-core` when changing a public result shape; Studio and phase 3
   reports must stay comparable.
