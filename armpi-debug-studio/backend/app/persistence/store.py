"""Append-only SQLite event store.

Events are never updated or deleted through the application. The `commands`
table is a current-state projection used for preview/execute continuity and
idempotency diagnostics.
"""

from __future__ import annotations

import csv
import io
import json
import sqlite3
import threading
from pathlib import Path
from typing import Any

from ..domain.models import CommandRecord, Event, jsonable
from ..errors import StorageError


class EventStore:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._conn = sqlite3.connect(str(self.path), check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        try:
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA synchronous=NORMAL")
            self._conn.execute("PRAGMA busy_timeout=3000")
        except sqlite3.Error:
            # A read-only or unusual filesystem may reject WAL; keep the
            # connection usable and let writes surface as StorageError.
            pass
        self._init_schema()

    def _init_schema(self) -> None:
        with self._lock:
            try:
                self._conn.executescript(
                    """
                    CREATE TABLE IF NOT EXISTS events (
                        seq INTEGER PRIMARY KEY AUTOINCREMENT,
                        event_id TEXT NOT NULL UNIQUE,
                        command_id TEXT,
                        stage TEXT NOT NULL,
                        wall_time TEXT NOT NULL,
                        monotonic_ms REAL NOT NULL,
                        source TEXT NOT NULL,
                        result TEXT NOT NULL,
                        error_code TEXT,
                        details_json TEXT NOT NULL,
                        schema_version INTEGER NOT NULL
                    );
                    CREATE INDEX IF NOT EXISTS idx_events_command
                        ON events(command_id);
                    CREATE INDEX IF NOT EXISTS idx_events_wall_time
                        ON events(wall_time);
                    CREATE TABLE IF NOT EXISTS commands (
                        command_id TEXT PRIMARY KEY,
                        session_id TEXT NOT NULL,
                        status TEXT NOT NULL,
                        targets_json TEXT NOT NULL,
                        created_at TEXT NOT NULL,
                        updated_at TEXT NOT NULL,
                        error_code TEXT,
                        details_json TEXT NOT NULL,
                        stage_history_json TEXT NOT NULL,
                        schema_version INTEGER NOT NULL
                    );
                    """
                )
                self._conn.commit()
            except sqlite3.Error as exc:
                raise StorageError(f"failed to initialize event store: {exc}") from exc

    # ------------------------------------------------------------------ writes
    def append_event(self, event: Event) -> int:
        with self._lock:
            try:
                cur = self._conn.execute(
                    """
                    INSERT INTO events(event_id, command_id, stage, wall_time,
                                       monotonic_ms, source, result, error_code,
                                       details_json, schema_version)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        event.event_id,
                        event.command_id,
                        event.stage,
                        event.wall_time,
                        event.monotonic_ms,
                        event.source,
                        event.result,
                        event.error_code,
                        json.dumps(jsonable(event.details), sort_keys=True),
                        event.schema_version,
                    ),
                )
                self._conn.commit()
                return int(cur.lastrowid)
            except sqlite3.Error as exc:
                self._conn.rollback()
                raise StorageError(f"failed to append event: {exc}") from exc

    def upsert_command(self, record: CommandRecord) -> None:
        with self._lock:
            try:
                self._conn.execute(
                    """
                    INSERT INTO commands(command_id, session_id, status,
                                         targets_json, created_at, updated_at,
                                         error_code, details_json,
                                         stage_history_json, schema_version)
                    VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, 1)
                    ON CONFLICT(command_id) DO UPDATE SET
                        status=excluded.status,
                        updated_at=excluded.updated_at,
                        error_code=excluded.error_code,
                        details_json=excluded.details_json,
                        stage_history_json=excluded.stage_history_json
                    """,
                    (
                        record.command_id,
                        record.session_id,
                        record.status.value,
                        json.dumps(jsonable(record.targets), sort_keys=True),
                        record.created_at,
                        record.updated_at,
                        record.error_code,
                        json.dumps(jsonable(record.details), sort_keys=True),
                        json.dumps(jsonable(record.stage_history), sort_keys=True),
                    ),
                )
                self._conn.commit()
            except sqlite3.Error as exc:
                self._conn.rollback()
                raise StorageError(f"failed to save command: {exc}") from exc

    # ------------------------------------------------------------------- reads
    def list_events(
        self,
        *,
        limit: int = 200,
        stage: str | None = None,
        source: str | None = None,
        command_id: str | None = None,
        after_seq: int = 0,
    ) -> list[dict[str, Any]]:
        clauses = ["seq > ?"]
        params: list[Any] = [after_seq]
        if stage:
            clauses.append("stage = ?")
            params.append(stage)
        if source:
            clauses.append("source = ?")
            params.append(source)
        if command_id:
            clauses.append("command_id = ?")
            params.append(command_id)
        query = (
            "SELECT seq, event_id, command_id, stage, wall_time, monotonic_ms, "
            "source, result, error_code, details_json, schema_version "
            "FROM events WHERE " + " AND ".join(clauses) + " ORDER BY seq DESC LIMIT ?"
        )
        params.append(max(1, min(limit, 2000)))
        with self._lock:
            try:
                rows = self._conn.execute(query, params).fetchall()
            except sqlite3.Error as exc:
                raise StorageError(f"failed to read events: {exc}") from exc
        result = []
        for row in rows:
            item = dict(row)
            item["details"] = json.loads(item.pop("details_json"))
            result.append(item)
        return result

    def get_command(self, command_id: str) -> dict[str, Any] | None:
        with self._lock:
            try:
                row = self._conn.execute(
                    "SELECT * FROM commands WHERE command_id = ?", (command_id,)
                ).fetchone()
            except sqlite3.Error as exc:
                raise StorageError(f"failed to read command: {exc}") from exc
        if row is None:
            return None
        item = dict(row)
        item["targets"] = json.loads(item.pop("targets_json"))
        item["details"] = json.loads(item.pop("details_json"))
        item["stage_history"] = json.loads(item.pop("stage_history_json"))
        return item

    def export_csv(self) -> str:
        with self._lock:
            try:
                rows = self._conn.execute(
                    "SELECT seq, event_id, command_id, stage, wall_time, monotonic_ms, "
                    "source, result, error_code, details_json, schema_version "
                    "FROM events ORDER BY seq"
                ).fetchall()
            except sqlite3.Error as exc:
                raise StorageError(f"failed to export events: {exc}") from exc
        buffer = io.StringIO()
        writer = csv.writer(buffer)
        writer.writerow(
            [
                "seq",
                "event_id",
                "command_id",
                "stage",
                "wall_time",
                "monotonic_ms",
                "source",
                "result",
                "error_code",
                "details_json",
                "schema_version",
            ]
        )
        for row in rows:
            writer.writerow([row[key] for key in row.keys()])
        return buffer.getvalue()

    def last_seq(self) -> int:
        with self._lock:
            try:
                row = self._conn.execute("SELECT COALESCE(MAX(seq), 0) AS seq FROM events").fetchone()
            except sqlite3.Error as exc:
                raise StorageError(f"failed to read event sequence: {exc}") from exc
        return int(row["seq"])

    def close(self) -> None:
        with self._lock:
            self._conn.close()
