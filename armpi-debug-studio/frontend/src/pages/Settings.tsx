import { api } from "../api/client";
import { Card, EmptyState, KeyValue, StatusChip } from "../components/ui";
import { formatNumber } from "../format";
import { usePreferences, useRobotStore } from "../store/robotStore";

export function SettingsPage() {
  const robot = useRobotStore((store) => store.state);
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const preferences = usePreferences();

  if (!robot || !diagnostics) {
    return <EmptyState title="正在读取配置…">需要后端 /api/state 与 /api/diagnostics。</EmptyState>;
  }

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">Settings</h1>
          <p className="page-subtitle">
            G1 配置只读。危险配置（模式、阈值、映射）需 Board 批准并写审计事件，不能在 UI 直接修改。
          </p>
        </div>
        <div className="page-actions">
          <StatusChip tone="warning" label="read-only in G1" />
        </div>
      </header>

      <div className="grid grid-2">
        <Card title="Connection" subtitle="后端启动参数决定模式；UI 不提供运行时切换">
          <div className="kv-list">
            <KeyValue label="Mode">
              <StatusChip tone={robot.mode === "mock" ? "simulation" : "warning"} label={robot.mode} />
            </KeyValue>
            <KeyValue label="Browser origin">{window.location.origin}</KeyValue>
            <KeyValue label="API path">/api（开发期由 Vite 代理到后端）</KeyValue>
            <KeyValue label="Bind host">{String(diagnostics.environment.bind_host ?? "—")}</KeyValue>
            <KeyValue label="ROS domain">未连接（G1 无 ROS 网关）</KeyValue>
            <KeyValue label="Hardware motion">
              <StatusChip
                tone={robot.capabilities.hardware_motion ? "ok" : "warning"}
                label={robot.capabilities.hardware_motion ? "enabled" : "disabled"}
              />
            </KeyValue>
            <KeyValue label="Hardware read">
              <StatusChip
                tone={robot.capabilities.hardware_read ? "ok" : "warning"}
                label={robot.capabilities.hardware_read ? "enabled" : "not implemented"}
              />
            </KeyValue>
          </div>
          <p className="footnote">
            切换模式需要设置后端环境变量 <code>ARMPI_MODE=mock|hardware</code> 并重启。运行中的 UI 不能切换。
          </p>
        </Card>

        <Card title="Display preferences" subtitle="仅保存在本浏览器；不影响后端">
          <div className="form-grid">
            <label className="field">
              <span>Angle unit</span>
              <select
                className="input"
                value={preferences.unit}
                onChange={(event) => preferences.setUnit(event.target.value as "rad" | "deg")}
              >
                <option value="rad">rad</option>
                <option value="deg">deg</option>
              </select>
            </label>
            <label className="field">
              <span>Decimals</span>
              <select
                className="input"
                value={preferences.decimals}
                onChange={(event) => preferences.setDecimals(Number(event.target.value))}
              >
                <option value={2}>2</option>
                <option value={3}>3</option>
                <option value={4}>4</option>
              </select>
            </label>
            <label className="field">
              <span>Time display</span>
              <select
                className="input"
                value={preferences.timeMode}
                onChange={(event) => preferences.setTimeMode(event.target.value as "local" | "utc")}
              >
                <option value="local">local</option>
                <option value="utc">UTC</option>
              </select>
            </label>
            <label className="field">
              <span>Density</span>
              <select
                className="input"
                value={preferences.density}
                onChange={(event) => preferences.setDensity(event.target.value as "compact" | "comfortable")}
              >
                <option value="compact">compact</option>
                <option value="comfortable">comfortable</option>
              </select>
            </label>
          </div>
        </Card>
      </div>

      <Card
        title="Joint mapping"
        subtitle={`${diagnostics.mapping.mapping_version} · ${diagnostics.mapping.verification_status}`}
        actions={
          <StatusChip
            tone={diagnostics.mapping.verified ? "ok" : "warning"}
            label={diagnostics.mapping.verified ? "verified" : "provisional"}
          />
        }
      >
        <div className="table-wrap">
          <table className="data-table">
            <thead>
              <tr>
                <th>Joint</th>
                <th>Servo</th>
                <th>Init raw</th>
                <th>Raw range</th>
                <th>Direction</th>
                <th>Rad range</th>
                <th>Source</th>
                <th>Verified</th>
              </tr>
            </thead>
            <tbody>
              {diagnostics.mapping.joints.map((joint) => (
                <tr key={joint.name}>
                  <td className="mono">{joint.name}</td>
                  <td className="mono">{joint.servo_id}</td>
                  <td className="mono">{joint.init_raw}</td>
                  <td className="mono">
                    {joint.min_raw} … {joint.max_raw}
                  </td>
                  <td className="mono">{joint.direction}</td>
                  <td className="mono">
                    {formatNumber(joint.min_rad, 3)} … {formatNumber(joint.max_rad, 3)}
                  </td>
                  <td>{joint.limit_source}</td>
                  <td>{joint.verified ? "yes" : "provisional"}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
        <p className="footnote">
          编辑映射被禁用：需要实机测量、版本化与 Board 批准。当前 hash：
          <span className="mono"> {diagnostics.mapping.mapping_hash.slice(0, 24)}…</span>
        </p>
      </Card>

      <div className="grid grid-2">
        <Card title="Safety thresholds" subtitle="G1 只读；修改需二次确认与审计（G2）">
          <div className="kv-list">
            {Object.entries(diagnostics.mapping.safety).map(([key, value]) => (
              <KeyValue key={key} label={key}>
                <span className="mono">{formatNumber(Number(value), 3)}</span>
              </KeyValue>
            ))}
          </div>
        </Card>

        <Card title="Logging" subtitle="append-only SQLite；事件导出包含 schema version">
          <div className="kv-list">
            <KeyValue label="Event store">{String(diagnostics.environment.mapping_version ? "SQLite WAL" : "—")}</KeyValue>
            <KeyValue label="Retention">30 days（设计值，清理任务未实现）</KeyValue>
            <KeyValue label="Privacy">默认脱敏主机名与绝对路径（G2 实现）</KeyValue>
            <KeyValue label="Storage error">
              {diagnostics.storage_error ? (
                <span className="error-text">{diagnostics.storage_error}</span>
              ) : (
                "none"
              )}
            </KeyValue>
          </div>
          <p className="footnote">
            <a href={api.exportUrl()} download>
              导出事件 CSV
            </a>
          </p>
        </Card>
      </div>

      <Card title="Roadmap" subtitle="未实现功能只列出，不放置可点击的假按钮">
        <ul className="roadmap-list">
          <li>
            <strong>G1 supervised motion</strong> — 单关节、限位 provisional、网关按实测脉冲独立复核；
            默认关闭，必须用固定的 ARMPI_HARDWARE_MOTION / ARMPI_SUPERVISED_MOTION_ACK 组合显式启用。
          </li>
          <li>
            <strong>物理急停与独立编码器</strong> — 未实现；Stop 只是保持指令已发送，不是刹车。
          </li>
        </ul>
        <p className="footnote">
          当前版本 {String(diagnostics.environment.backend_version ?? "—")}；G2 Trajectory Inspector、
          G3 Experiment Lab、G4 Visual Workbench 与 G5 记录器已实现。硬件运动仅在受监督实例中可用，
          mapping_verified 仍为 false。
        </p>
      </Card>
    </div>
  );
}
