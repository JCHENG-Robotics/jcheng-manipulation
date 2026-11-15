import { useState } from "react";
import { api, ApiError } from "../api/client";
import type { ProbeResult } from "../api/types";
import {
  Button,
  Card,
  EmptyState,
  ErrorBanner,
  KeyValue,
  StatusChip,
  TruthBadge,
} from "../components/ui";
import { formatAngle, formatNumber } from "../format";
import { usePreferences, useRobotStore } from "../store/robotStore";

const FAULT_OPTIONS = [
  { kind: "timeout", label: "命令超时" },
  { kind: "disconnect", label: "连接断开" },
  { kind: "empty_feedback", label: "空反馈" },
  { kind: "nan_feedback", label: "NaN 反馈" },
  { kind: "missing_servo", label: "舵机缺失" },
  { kind: "driver_exit", label: "Driver 退出" },
];

function errorText(error: unknown): string {
  return error instanceof ApiError ? `${error.code}: ${error.message}` : String(error);
}

export function DiagnosticsPage() {
  const robot = useRobotStore((store) => store.state);
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const setDiagnostics = useRobotStore((store) => store.setDiagnostics);
  const preferences = usePreferences();
  const [probe, setProbe] = useState<ProbeResult | null>(null);
  const [faultKind, setFaultKind] = useState("timeout");
  const [faultJoint, setFaultJoint] = useState("joint1");
  const [error, setError] = useState<string | null>(null);
  const [busy, setBusy] = useState(false);

  if (!robot || !diagnostics) {
    return <EmptyState title="正在读取诊断信息…">需要后端 /api/diagnostics。</EmptyState>;
  }

  const refresh = async () => {
    try {
      setDiagnostics(await api.diagnostics());
    } catch (caught) {
      setError(errorText(caught));
    }
  };

  const runProbe = async () => {
    setBusy(true);
    setError(null);
    try {
      setProbe(await api.probe());
      await refresh();
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const injectFault = async () => {
    setBusy(true);
    setError(null);
    try {
      await api.injectFault(faultKind, faultKind === "missing_servo" ? faultJoint : undefined);
      await refresh();
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const resetMock = async () => {
    setBusy(true);
    setError(null);
    try {
      await api.resetMock();
      setProbe(null);
      await refresh();
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const isMock = robot.mode === "mock";

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">Diagnostics</h1>
          <p className="page-subtitle">
            ROS 图、串口、已知 Driver 缺陷、单次探针与 Mock 故障注入。缺失数据必须显示 Unavailable。
          </p>
        </div>
        <div className="page-actions">
          <StatusChip tone={diagnostics.safety.ready ? "ok" : "warning"} label={diagnostics.safety.ready ? "ready" : "blocked"} />
          <StatusChip tone={isMock ? "simulation" : "warning"} label={isMock ? "SIMULATION" : "LIVE / read-only"} />
        </div>
      </header>

      {error && <ErrorBanner message={error} onDismiss={() => setError(null)} />}
      {!isMock && (
        <div className="banner banner-warning">
          <strong>Hardware gateway not implemented</strong>
          <span>E7 只读网关尚未实现；不会打开串口，也不会启动 ROS 节点。</span>
        </div>
      )}

      <div className="grid grid-2">
        <Card title="Connection / ROS graph" subtitle="期望链路 vs 实际证据">
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
                            : "warning"
                    }
                    label={segment.state}
                  />
                </div>
                <p>{segment.detail}</p>
                <span className="evidence">evidence: {segment.evidence}</span>
              </div>
            ))}
          </div>
        </Card>

        <Card title="Environment" subtitle="版本与映射来源">
          <div className="kv-list">
            <KeyValue label="Robot">{robot.robot_id}</KeyValue>
            <KeyValue label="Mode">{diagnostics.mode}</KeyValue>
            <KeyValue label="Backend">{String(diagnostics.environment.backend_version ?? "—")}</KeyValue>
            <KeyValue label="Python">{String(diagnostics.environment.python ?? "—")}</KeyValue>
            <KeyValue label="Bind host">{String(diagnostics.environment.bind_host ?? "—")}</KeyValue>
            <KeyValue label="Mapping version">{diagnostics.mapping.mapping_version}</KeyValue>
            <KeyValue label="Mapping hash">
              <span className="mono">{diagnostics.mapping.mapping_hash.slice(0, 16)}…</span>
            </KeyValue>
            <KeyValue label="Limits">
              <StatusChip
                tone={diagnostics.mapping.verified ? "ok" : "warning"}
                label={diagnostics.mapping.verified ? "verified" : "provisional"}
              />
            </KeyValue>
            <KeyValue label="Storage error">
              {diagnostics.storage_error ? (
                <span className="error-text">{diagnostics.storage_error}</span>
              ) : (
                "none"
              )}
            </KeyValue>
          </div>
        </Card>
      </div>

      <Card title="Known vendor defects" subtitle="来自 G0 只读审计；未修补前不得依赖这些路径">
        <div className="table-wrap">
          <table className="data-table">
            <thead>
              <tr>
                <th>ID</th>
                <th>Summary</th>
                <th>Source</th>
                <th>Impact</th>
              </tr>
            </thead>
            <tbody>
              {diagnostics.known_defects.map((defect) => (
                <tr key={defect.id}>
                  <td className="mono">{defect.id}</td>
                  <td>{defect.summary}</td>
                  <td className="mono">{defect.source}</td>
                  <td>{defect.impact}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </Card>

      <div className="grid grid-2">
        <Card
          title="Feedback probe"
          subtitle="单次读取；G1 仅 Mock 可用"
          actions={
            <Button variant="primary" onClick={() => void runProbe()} disabled={busy || !diagnostics.probe_available}>
              {busy ? "探测中…" : "运行探针"}
            </Button>
          }
        >
          {!diagnostics.probe_available ? (
            <EmptyState title="Hardware probe not implemented">
              G1 不打开串口。需要 E7 只读网关与 Board 授权后才能做真实探针。
            </EmptyState>
          ) : probe ? (
            <div className="table-wrap">
              <table className="data-table">
                <thead>
                  <tr>
                    <th>Joint</th>
                    <th>Servo</th>
                    <th>OK</th>
                    <th>Raw</th>
                    <th>Position</th>
                    <th>Duration</th>
                    <th>Truth</th>
                  </tr>
                </thead>
                <tbody>
                  {probe.results.map((result) => (
                    <tr key={result.joint}>
                      <td className="mono">{result.joint}</td>
                      <td className="mono">{result.servo_id}</td>
                      <td>{result.ok ? "yes" : <span className="error-text">no</span>}</td>
                      <td className="mono">{result.raw_position ?? "Unavailable"}</td>
                      <td className="mono">
                        {formatAngle(result.position_rad, preferences.unit, preferences.decimals)}
                      </td>
                      <td className="mono">{result.duration_ms} ms</td>
                      <td><TruthBadge truth={result.truth} /></td>
                    </tr>
                  ))}
                </tbody>
              </table>
              <p className="footnote">{probe.note}</p>
            </div>
          ) : (
            <EmptyState title="尚未运行探针">探针只读取模拟状态，不会发送运动命令。</EmptyState>
          )}
        </Card>

        <Card
          title="Mock fault injection"
          subtitle="仅 Mock 模式；用于验证门禁与 UI 降级"
          actions={
            isMock && (
              <Button variant="secondary" onClick={() => void resetMock()} disabled={busy}>
                清除所有故障
              </Button>
            )
          }
        >
          {!isMock ? (
            <EmptyState title="Fault injection unavailable">Hardware Mode 不允许注入模拟故障。</EmptyState>
          ) : (
            <>
              <div className="form-grid">
                <label className="field">
                  <span>Fault</span>
                  <select
                    className="input"
                    value={faultKind}
                    onChange={(event) => setFaultKind(event.target.value)}
                  >
                    {FAULT_OPTIONS.map((option) => (
                      <option key={option.kind} value={option.kind}>
                        {option.label} ({option.kind})
                      </option>
                    ))}
                  </select>
                </label>
                {faultKind === "missing_servo" && (
                  <label className="field">
                    <span>Joint</span>
                    <select
                      className="input"
                      value={faultJoint}
                      onChange={(event) => setFaultJoint(event.target.value)}
                    >
                      {robot.joints.map((joint) => (
                        <option key={joint.name} value={joint.name}>
                          {joint.name}
                        </option>
                      ))}
                    </select>
                  </label>
                )}
              </div>
              <div className="button-row">
                <Button variant="danger" onClick={() => void injectFault()} disabled={busy}>
                  注入故障
                </Button>
              </div>
              {diagnostics.mock_faults.length > 0 ? (
                <div className="fault-list">
                  {diagnostics.mock_faults.map((fault) => (
                    <div className="fault-item" key={fault.kind}>
                      <StatusChip
                        tone={fault.level === "critical" ? "critical" : "fault"}
                        label={fault.level}
                      />
                      <strong className="mono">{fault.kind}</strong>
                      {fault.joint && <span className="muted">joint={fault.joint}</span>}
                    </div>
                  ))}
                </div>
              ) : (
                <p className="footnote">当前没有注入故障。</p>
              )}
            </>
          )}
        </Card>
      </div>

      <Card title="Safety checks" subtitle="与 Dashboard 相同的 12 项门禁">
        <div className="check-grid">
          {diagnostics.safety.checks.map((check) => (
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
        <p className="footnote">
          反馈年龄阈值：{formatNumber(Number(diagnostics.mapping.safety.feedback_max_age_ms), 0)} ms；
          最大步长：{formatNumber(Number(diagnostics.mapping.safety.max_step_rad), 3)} rad。
        </p>
      </Card>
    </div>
  );
}
