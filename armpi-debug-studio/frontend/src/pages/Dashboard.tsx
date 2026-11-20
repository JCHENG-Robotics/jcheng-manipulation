import { useEffect, useState } from "react";
import { api, ApiError } from "../api/client";
import type { CameraStatus, Peripherals } from "../api/types";
import {
  Button,
  Card,
  DataAge,
  EmptyState,
  ErrorBanner,
  KeyValue,
  MetricValue,
  SourceBadge,
  StatusChip,
  TruthBadge,
  ValidityBadge,
} from "../components/ui";
import { formatAge, formatAngle, formatClock, formatNumber, maxFeedbackAge } from "../format";
import { getSessionId } from "../session";
import { usePreferences, useRobotStore } from "../store/robotStore";

function CameraPanel({
  title,
  subtitle,
  camera,
  snapshotUrl,
  alt,
  overlay,
  loading,
  failed,
  onLoad,
  onError,
}: {
  title: string;
  subtitle: string;
  camera: CameraStatus | null;
  snapshotUrl: string;
  alt: string;
  overlay: string;
  loading: boolean;
  failed: boolean;
  onLoad: () => void;
  onError: () => void;
}) {
  return (
    <Card
      title={title}
      subtitle={subtitle}
      actions={
        <>
          {camera && <TruthBadge truth={camera.truth} />}
          <StatusChip
            tone={camera && camera.available ? (camera.validity === "valid" ? "ok" : "warning") : "neutral"}
            label={camera && camera.available ? camera.validity : "unavailable"}
          />
        </>
      }
    >
      {loading ? (
        <EmptyState title="正在读取相机…">等待 /api/peripherals 返回相机状态。</EmptyState>
      ) : camera && camera.available ? (
        <>
          <div className="camera-frame">
            {failed ? (
              <div className="camera-fallback">Snapshot unavailable</div>
            ) : (
              <img src={snapshotUrl} alt={alt} onLoad={onLoad} onError={onError} />
            )}
            <span className="camera-overlay">{overlay}</span>
          </div>
          <div className="camera-meta">
            <span>
              {camera.width ?? "—"}×{camera.height ?? "—"}
            </span>
            <span>{camera.fps !== null ? `${camera.fps.toFixed(1)} fps` : "—"}</span>
            <span>{camera.encoding ?? "—"}</span>
            <span>age {formatAge(camera.sample_age_ms)}</span>
            <span>frames {camera.frame_count}</span>
          </div>
          {camera.notes.length > 0 && <p className="footnote">{camera.notes.join(" ")}</p>}
        </>
      ) : (
        <EmptyState title="Camera Unavailable">
          {camera?.notes?.length
            ? camera.notes.join(" ")
            : "尚未收到相机帧，或网关未配置。"}
        </EmptyState>
      )}
    </Card>
  );
}

export function Dashboard() {
  const robot = useRobotStore((store) => store.state);
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const events = useRobotStore((store) => store.events);
  const lease = useRobotStore((store) => store.lease);
  const setLease = useRobotStore((store) => store.setLease);
  const preferences = usePreferences();
  const sessionId = getSessionId();
  const [leaseBusy, setLeaseBusy] = useState(false);
  const [leaseError, setLeaseError] = useState<string | null>(null);
  const [peripherals, setPeripherals] = useState<Peripherals | null>(null);
  const [peripheralError, setPeripheralError] = useState<string | null>(null);
  const [cameraTick, setCameraTick] = useState(0);
  const [cameraFailed, setCameraFailed] = useState(false);
  const [auxCameraFailed, setAuxCameraFailed] = useState(false);

  useEffect(() => {
    let cancelled = false;
    const load = async () => {
      try {
        const data = await api.peripherals();
        if (!cancelled) {
          setPeripherals(data);
          setPeripheralError(null);
        }
      } catch (error) {
        if (!cancelled) {
          setPeripheralError(
            error instanceof ApiError ? `${error.code}: ${error.message}` : String(error),
          );
        }
      }
    };
    void load();
    const timer = window.setInterval(load, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, []);

  useEffect(() => {
    const timer = window.setInterval(() => {
      setCameraTick((tick) => tick + 1);
      setCameraFailed(false);
      setAuxCameraFailed(false);
    }, 300);
    return () => window.clearInterval(timer);
  }, []);

  if (!robot || !diagnostics) {
    return (
      <EmptyState title="正在连接后端…">
        页面会通过 HTTP/WebSocket 读取状态。请确认 backend 已启动，并打开浏览器控制台查看错误。
      </EmptyState>
    );
  }

  const safety = diagnostics.safety;
  const feedbackAge = maxFeedbackAge(robot.joints);
  const jointTruth = robot.joints[0]?.truth;
  const feedbackHint =
    robot.mode === "mock"
      ? "simulated in Mock"
      : jointTruth === "commanded"
        ? "command echo, not encoder feedback"
        : "measured feedback";
  const activeFaults = robot.faults;
  const latestEvents = events.slice(0, 8);
  const battery = peripherals?.battery ?? null;
  const camera = peripherals?.camera ?? null;
  const auxCamera = peripherals?.aux_camera ?? null;
  const batteryTone =
    battery?.status === "ok"
      ? "ok"
      : battery?.status === "low"
        ? "warning"
        : battery?.status === "critical"
          ? "fault"
          : "neutral";
  const batteryPercent = Math.max(0, Math.min(100, battery?.percent_estimate ?? 0));

  const acquireLease = async () => {
    setLeaseBusy(true);
    setLeaseError(null);
    try {
      setLease(await api.acquire(sessionId));
    } catch (error) {
      setLeaseError(error instanceof ApiError ? `${error.code}: ${error.message}` : String(error));
    } finally {
      setLeaseBusy(false);
    }
  };

  const releaseLease = async () => {
    if (!lease) {
      return;
    }
    setLeaseBusy(true);
    setLeaseError(null);
    try {
      await api.release(lease.lease_id, sessionId);
      setLease(null);
    } catch (error) {
      setLeaseError(error instanceof ApiError ? `${error.code}: ${error.message}` : String(error));
    } finally {
      setLeaseBusy(false);
    }
  };

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">Dashboard</h1>
          <p className="page-subtitle">
            连接链路、安全门禁、关节快照与最近事件。默认只读；运动需要显式控制权与预览确认。
          </p>
        </div>
        <div className="page-actions">
          <SourceBadge source={robot.source_label} />
          <StatusChip
            tone={safety.ready ? "ok" : "warning"}
            label={safety.ready ? "safety ready" : "safety blocked"}
            detail={safety.blockers.join(", ")}
          />
        </div>
      </header>

      {robot.source_label === "SIMULATION" && (
        <div className="banner banner-simulation">
          <strong>SIMULATION</strong>
          <span>当前数据来自 Mock 适配器，不是硬件反馈。没有打开串口，也没有启动 ROS 节点。</span>
        </div>
      )}
      {robot.mode === "hardware" && (
        <div className="banner banner-warning">
          <strong>Hardware Mode</strong>
          <span>G1 只读发现阶段：运动未实现，Execute 会被后端拒绝。</span>
        </div>
      )}

      <div className="grid grid-4">
        <MetricValue
          label="Connection"
          value={robot.connection_state}
          tone={robot.connection_state === "Simulation" ? "simulation" : "neutral"}
          hint={robot.mode === "mock" ? "in-process mock" : "read-only gate"}
        />
        <MetricValue
          label="Safety blockers"
          value={safety.blockers.length}
          tone={safety.blockers.length === 0 ? "ok" : "warning"}
          hint={safety.blockers.join(", ") || "none"}
        />
        <MetricValue
          label="Joints"
          value={robot.joints.length}
          unit="G1"
          hint="gripper excluded (Q-08)"
        />
        <MetricValue
          label="Feedback age"
          value={feedbackAge === null ? "—" : `${feedbackAge} ms`}
          tone={feedbackAge === null ? "neutral" : feedbackAge > 500 ? "fault" : "ok"}
          hint={feedbackHint}
        />
      </div>

      {peripheralError && (
        <ErrorBanner message={peripheralError} onDismiss={() => setPeripheralError(null)} />
      )}

      <div className="grid grid-camera">
        <CameraPanel
          title="Camera · RGB"
          subtitle={camera?.topic ?? "/depth_cam/rgb/image_raw"}
          camera={camera}
          snapshotUrl={api.cameraSnapshotUrl(cameraTick)}
          alt="ArmPi camera"
          overlay={robot.source_label}
          loading={!peripherals}
          failed={cameraFailed}
          onLoad={() => setCameraFailed(false)}
          onError={() => setCameraFailed(true)}
        />

        <CameraPanel
          title="Camera · Aux"
          subtitle={auxCamera?.topic ?? "uvc:/dev/video4"}
          camera={auxCamera}
          snapshotUrl={api.auxCameraSnapshotUrl(cameraTick)}
          alt="ArmPi aux camera"
          overlay={auxCamera?.truth === "simulated" ? "AUX · SIMULATION" : "D435i · UVC"}
          loading={!peripherals}
          failed={auxCameraFailed}
          onLoad={() => setAuxCameraFailed(false)}
          onError={() => setAuxCameraFailed(true)}
        />

        <Card
          className="battery-card"
          title="Battery · 2S"
          subtitle={battery?.topic ?? "/ros_robot_controller/battery"}
          actions={
            <>
              {battery && <TruthBadge truth={battery.truth} />}
              <StatusChip tone={batteryTone} label={battery?.status ?? "unavailable"} />
            </>
          }
        >
          {!peripherals ? (
            <EmptyState title="正在读取电池…">等待 /api/peripherals 返回电池状态。</EmptyState>
          ) : battery && battery.available && battery.voltage_v !== null ? (
            <>
              <div className="battery-readout">
                <span className="battery-voltage mono">
                  {battery.voltage_v.toFixed(2)}
                  <span className="battery-unit">V</span>
                </span>
                <div className="battery-gauge">
                  <div
                    className={`battery-fill battery-${battery.status}`}
                    style={{ width: `${batteryPercent}%` }}
                  />
                </div>
                <span className="battery-percent mono">
                  {battery.percent_estimate !== null
                    ? `${battery.percent_estimate.toFixed(0)}%`
                    : "—"}
                </span>
              </div>
              <div className="kv-list">
                <KeyValue label="Cells">{battery.cell_count}S</KeyValue>
                <KeyValue label="Age">{formatAge(battery.sample_age_ms)}</KeyValue>
                <KeyValue label="Source">{battery.source}</KeyValue>
                <KeyValue label="Validity">
                  <ValidityBadge validity={battery.validity} />
                </KeyValue>
              </div>
              {battery.status !== "ok" && (
                <p className="battery-warning">
                  {battery.status === "critical"
                    ? "电压严重偏低，停止运动并充电。"
                    : "电压偏低，长时运动前请充电。"}
                </p>
              )}
              {battery.notes.length > 0 && <p className="footnote">{battery.notes.join(" ")}</p>}
            </>
          ) : (
            <EmptyState title="Battery Unavailable">
              {battery?.notes?.length
                ? battery.notes.join(" ")
                : "尚未收到电池数据，或网关未配置。"}
            </EmptyState>
          )}
        </Card>
      </div>

      <div className="grid grid-2">
        <Card title="Connection chain" subtitle="serial → driver → gateway → feedback">
          <div className="chain">
            {diagnostics.connection.segments.map((segment) => (
              <div className="chain-segment" key={segment.name}>
                <div className="chain-head">
                  <strong>{segment.name}</strong>
                  <StatusChip
                    tone={
                      segment.state === "simulated"
                        ? "simulation"
                        : segment.state === "online"
                          ? "ok"
                          : segment.state === "offline"
                            ? "fault"
                            : segment.state === "unavailable"
                              ? "warning"
                              : "neutral"
                    }
                    label={segment.state}
                  />
                </div>
                <p>{segment.detail}</p>
                <span className="evidence">evidence: {segment.evidence}</span>
              </div>
            ))}
          </div>
          {diagnostics.connection.notes.length > 0 && (
            <ul className="note-list">
              {diagnostics.connection.notes.map((note) => (
                <li key={note}>{note}</li>
              ))}
            </ul>
          )}
        </Card>

        <Card
          title="Control lease"
          subtitle="唯一控制权；预览不需要租约，执行需要"
          actions={
            lease ? (
              <Button variant="secondary" onClick={() => void releaseLease()} disabled={leaseBusy}>
                释放控制权
              </Button>
            ) : (
              <Button variant="primary" onClick={() => void acquireLease()} disabled={leaseBusy}>
                {leaseBusy ? "申请中…" : "申请控制权"}
              </Button>
            )
          }
        >
          {leaseError && <ErrorBanner message={leaseError} onDismiss={() => setLeaseError(null)} />}
          <div className="lease-panel">
            <KeyValue label="Session">{sessionId.slice(0, 12)}…</KeyValue>
            {lease ? (
              <>
                <KeyValue label="Lease">{lease.lease_id.slice(0, 12)}…</KeyValue>
                <KeyValue label="Expires">{formatClock(lease.expires_at, preferences.timeMode)}</KeyValue>
                <KeyValue label="TTL">{lease.ttl_s} s</KeyValue>
                <StatusChip tone="ok" label="lease held by this browser" />
              </>
            ) : (
              <EmptyState title="当前没有控制权">
                申请租约后才能执行运动命令；页面默认只读。
              </EmptyState>
            )}
          </div>
        </Card>
      </div>

      <Card
        title="Safety gate"
        subtitle={`${safety.blockers.length} blockers · ${safety.warnings.length} warnings`}
        actions={<StatusChip tone={safety.ready ? "ok" : "warning"} label={safety.ready ? "READY" : "BLOCKED"} />}
      >
        <div className="check-grid">
          {safety.checks.map((check) => (
            <div
              className={`check-item ${check.passed ? "check-pass" : check.blocking ? "check-block" : "check-warn"}`}
              key={check.check_id}
            >
              <div className="check-head">
                <span className="check-id">{check.check_id}</span>
                <StatusChip
                  tone={check.passed ? "ok" : check.blocking ? "fault" : "warning"}
                  label={check.passed ? "pass" : check.blocking ? "block" : "warn"}
                />
              </div>
              <strong>{check.label}</strong>
              <p>{check.detail}</p>
            </div>
          ))}
        </div>
      </Card>

      <Card title="Joint snapshot" subtitle="目标 / 读回 / 来源 / 年龄；不把模拟值当作硬件反馈">
        <div className="table-wrap">
          <table className="data-table">
            <thead>
              <tr>
                <th>Joint</th>
                <th>Servo</th>
                <th>Target</th>
                <th>Readback</th>
                <th>Truth</th>
                <th>Validity</th>
                <th>Age</th>
                <th>Limit</th>
              </tr>
            </thead>
            <tbody>
              {robot.joints.map((joint) => (
                <tr key={joint.name}>
                  <td className="mono">{joint.name}</td>
                  <td className="mono">{joint.servo_id}</td>
                  <td className="mono">{formatAngle(joint.target_rad, preferences.unit, preferences.decimals)}</td>
                  <td className="mono">{formatAngle(joint.position_rad, preferences.unit, preferences.decimals)}</td>
                  <td><TruthBadge truth={joint.truth} /></td>
                  <td><ValidityBadge validity={joint.validity} /></td>
                  <td><DataAge ageMs={joint.sample_age_ms} maxAgeMs={500} /></td>
                  <td className="mono muted">
                    {formatNumber(joint.limit_min_rad, 2)} … {formatNumber(joint.limit_max_rad, 2)} rad
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
        <p className="footnote">
          这些限位来自厂商软件配置，状态为 <strong>provisional</strong>，不是机械限位实测值。
        </p>
      </Card>

      <div className="grid grid-2">
        <Card title="Active faults" subtitle="只有有证据时才显示 Fault/Critical">
          {activeFaults.length === 0 ? (
            <EmptyState title="无活动故障">
              注意：没有故障不等于硬件已验证；Mock 数据始终是模拟值。
            </EmptyState>
          ) : (
            <div className="fault-list">
              {activeFaults.map((fault) => (
                <div className="fault-item" key={fault.fault_id}>
                  <div className="fault-head">
                    <StatusChip
                      tone={fault.level === "critical" ? "critical" : fault.level === "fault" ? "fault" : "warning"}
                      label={fault.level}
                    />
                    <strong>{fault.code}</strong>
                  </div>
                  <p>{fault.message}</p>
                  <span className="evidence">{fault.remediation}</span>
                </div>
              ))}
            </div>
          )}
        </Card>

        <Card title="Latest events" subtitle="最近 8 条命令/状态事件">
          {latestEvents.length === 0 ? (
            <EmptyState title="还没有事件">执行一次预览或故障注入后，这里会出现审计记录。</EmptyState>
          ) : (
            <div className="table-wrap">
              <table className="data-table">
                <thead>
                  <tr>
                    <th>Time</th>
                    <th>Stage</th>
                    <th>Source</th>
                    <th>Result</th>
                  </tr>
                </thead>
                <tbody>
                  {latestEvents.map((event) => (
                    <tr key={event.event_id}>
                      <td className="mono">{formatClock(event.wall_time, preferences.timeMode)}</td>
                      <td><StatusChip tone={event.error_code ? "fault" : "info"} label={event.stage} /></td>
                      <td>{event.source}</td>
                      <td className="ellipsis" title={event.result}>{event.result}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </Card>
      </div>
    </div>
  );
}
