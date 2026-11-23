# Backend shared-token auth — 2026-11-23

Scope: close the P1 item "the Studio backend has no authentication".

## Contract

- Default remains `BIND_HOST=127.0.0.1` with **no token**; local behavior is
  unchanged.
- A non-loopback `BIND_HOST` is refused at startup unless
  `ARMPI_BACKEND_TOKEN` is set. There is no unauthenticated remote mode.
- With a token, every `/api` path except `/api/health` requires
  `X-Armpi-Token` (header) or `?token=...` (used by WebSocket and CSV
  download). Comparisons use `hmac.compare_digest`.
- The frontend accepts `VITE_API_TOKEN` at build time and sends the header,
  appends the query token to WebSocket URLs, and appends it to the CSV export
  URL. The token is therefore embedded in the built JS; remote use needs TLS
  at a reverse proxy.
- `/api/health` stays unauthenticated for liveness/monitoring.

## Verification

- Backend tests: **148 passed**, including four new auth tests
  (`tests/test_auth.py`).
- A temporary mock backend on `127.0.0.1:8001` with
  `ARMPI_BACKEND_TOKEN=test-secret`:
  - `/api/health` no token → `200`;
  - `/api/state` no token → `401 UNAUTHORIZED`;
  - header or query token → `200`;
  - wrong token → `401`.
- `BIND_HOST=0.0.0.0` without a token raises a startup `ValueError`; with a
  token it parses and reports `0.0.0.0 remote-secret`.
- The live `127.0.0.1:8000` backend was restarted with the new code and no
  token; `/api/health` and `/api/state` both return `200`. No hardware motion
  was sent.
- Frontend `npm run build` and the 10 mock Playwright tests pass.
- A token-mode build (`VITE_API_TOKEN=test-secret`) plus
  `ARMPI_BACKEND_TOKEN=test-secret` passed the three dashboard Playwright
  tests; this caught and fixed the camera `<img>` URLs, which now also carry
  the query token.

## Residual / deployment note

The current live backend does **not** set `ARMPI_BACKEND_TOKEN` because it is
localhost-only. If it is moved behind a remote bind, set the token and rebuild
the frontend with the same `VITE_API_TOKEN`; the startup guard makes a
forgotten token fail closed.

## Files

- `auth_runtime.txt` — live temp-server status codes and the config guard.
