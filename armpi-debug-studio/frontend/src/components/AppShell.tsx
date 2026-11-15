import { useEffect, useState } from "react";
import { NavLink, Outlet } from "react-router-dom";
import { api, ApiError } from "../api/client";
import type { StopResult } from "../api/types";
import { formatAge, formatClock, highestFaultLevel, maxFeedbackAge } from "../format";
import { usePreferences, useRobotStore } from "../store/robotStore";
import { getSessionId } from "../session";
import { Button, SourceBadge, StatusChip } from "./ui";

const NAV_ITEMS = [
  { to: "/", label: "Dashboard", end: true },
  { to: "/joints", label: "Joint Control" },
  { to: "/diagnostics", label: "Diagnostics" },
  { to: "/events", label: "Event Timeline" },
  { to: "/settings", label: "Settings" },
];

function connectionTone(state: string | undefined): "ok" | "warning" | "fault" | "neutral" {
  if (!state) return "neutral";
  if (state === "Ready" || state === "Simulation" || state === "ReadOnly") return "ok";
  if (state === "Executing" || state === "Connecting" || state === "Degraded") return "warning";
  return "fault";
}

export function AppShell() {
  const robot = useRobotStore((store) => store.state);
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const events = useRobotStore((store) => store.events);
  const lease = useRobotStore((store) => store.lease);
  const wsConnected = useRobotStore((store) => store.wsConnected);
  const error = useRobotStore((store) => store.error);
  const setLease = useRobotStore((store) => store.setLease);
  const preferences = usePreferences();
  const [stopResult, setStopResult] = useState<StopResult | null>(null);
  const [stopBusy, setStopBusy] = useState(false);
  const [stopError, setStopError] = useState<string | null>(null);
  const sessionId = getSessionId();

  useEffect(() => {
    if (!lease) {
      return;
    }
    const timer = window.setInterval(() => {
      void api
        .heartbeat(lease.lease_id, sessionId)
        .catch(() => setLease(null));
    }, 20000);
    return () => window.clearInterval(timer);
  }, [lease, sessionId, setLease]);

  const handleStop = async () => {
    setStopBusy(true);
    setStopError(null);
    try {
      const result = await api.stop({
        sessionId,
        leaseId: lease?.lease_id,
      });
      setStopResult(result);
    } catch (caught) {
      setStopError(
        caught instanceof ApiError ? `${caught.code}: ${caught.message}` : String(caught),
      );
    } finally {
      setStopBusy(false);
    }
  };

  const feedbackAge = robot ? maxFeedbackAge(robot.joints) : null;
  const faultLevel = robot ? highestFaultLevel(robot.faults) : null;
  const lastEvent = events[0] ?? null;
  const mode = robot?.mode ?? diagnostics?.mode ?? "mock";
  const sourceLabel = robot?.source_label ?? (mode === "mock" ? "SIMULATION" : "LIVE");
  const ready = diagnostics?.safety.ready ?? false;

  return (
    <div className={`app-shell density-${preferences.density}`}>
      <header className="topbar">
        <div className="brand">
          <span className="brand-mark">ARM</span>
          <div>
            <strong>ArmPi Debug Studio</strong>
            <span className="brand-sub">G1 safe console</span>
          </div>
        </div>

        <div className="topbar-status">
          <div className="topbar-field">
            <span className="topbar-label">Robot</span>
            <strong>{robot?.robot_id ?? "—"}</strong>
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Source</span>
            <SourceBadge source={sourceLabel} />
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Connection</span>
            <StatusChip
              tone={connectionTone(robot?.connection_state)}
              label={robot?.connection_state ?? "Disconnected"}
            />
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Control owner</span>
            <strong className="mono">
              {lease ? lease.session_id.slice(0, 8) : robot?.control_owner ? robot.control_owner.slice(0, 8) : "none"}
            </strong>
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Feedback age</span>
            <StatusChip
              tone={feedbackAge === null ? "neutral" : feedbackAge > 500 ? "fault" : "ok"}
              label={formatAge(feedbackAge)}
            />
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Faults</span>
            <StatusChip
              tone={
                faultLevel === "critical"
                  ? "critical"
                  : faultLevel === "fault"
                    ? "fault"
                    : faultLevel === "warning"
                      ? "warning"
                      : "ok"
              }
              label={faultLevel ? `${faultLevel} (${robot?.faults.length ?? 0})` : "none"}
            />
          </div>
          <div className="topbar-field">
            <span className="topbar-label">Safety gate</span>
            <StatusChip
              tone={ready ? "ok" : "warning"}
              label={ready ? "ready" : "blocked"}
              detail={diagnostics?.safety.blockers.join(", ")}
            />
          </div>
          <Button
            variant="danger"
            onClick={() => void handleStop()}
            disabled={stopBusy}
            title="软件停止请求，非急停；物理断电是最终防护"
          >
            {stopBusy ? "停止请求中…" : "停止请求"}
          </Button>
        </div>
      </header>

      {!wsConnected && (
        <div className="ws-banner">
          状态 WebSocket 未连接；页面会使用 HTTP 轮询回退。请检查后端是否运行。
        </div>
      )}
      {error && <div className="global-error">{error}</div>}
      {stopError && (
        <div className="error-banner">
          <span>停止请求失败：{stopError}</span>
          <button type="button" className="link-button" onClick={() => setStopError(null)}>
            关闭
          </button>
        </div>
      )}
      {stopResult && (
        <div className="stop-banner">
          <StatusChip
            tone={
              stopResult.state === "StopObserved"
                ? "ok"
                : stopResult.state === "StopUnknown"
                  ? "fault"
                  : "warning"
            }
            label={stopResult.state}
          />
          <span>{stopResult.detail}</span>
          <span className="muted">{stopResult.evidence}</span>
          <button type="button" className="link-button" onClick={() => setStopResult(null)}>
            关闭
          </button>
        </div>
      )}

      <div className="layout">
        <nav className="left-nav" aria-label="主导航">
          {NAV_ITEMS.map((item) => (
            <NavLink
              key={item.to}
              to={item.to}
              end={item.end}
              className={({ isActive }) => `nav-link${isActive ? " nav-link-active" : ""}`}
            >
              {item.label}
            </NavLink>
          ))}
          <div className="nav-note">
            <strong>G1 范围</strong>
            <p>Mock 完整演示；Hardware 只读发现，运动未实现。</p>
          </div>
        </nav>

        <main className="page-content">
          <Outlet />
        </main>
      </div>

      <footer className="statusbar">
        <span>mode: {mode}</span>
        <span>mapping: {robot?.mapping_version ?? "—"}</span>
        <span>limits: {robot?.limit_version ?? "—"}</span>
        <span>last event: {lastEvent ? formatClock(lastEvent.wall_time, preferences.timeMode) : "—"}</span>
        <span>backend: {String(diagnostics?.environment.backend_version ?? "—")}</span>
        <span>clock: {preferences.timeMode.toUpperCase()}</span>
      </footer>
    </div>
  );
}
