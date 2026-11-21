import { useEffect, useMemo, useRef, useState } from "react";
import { api, ApiError } from "../api/client";
import type {
  EventRecord,
  MotionPreset,
  MotionPresetList,
  PreviewResult,
  TargetPayload,
  TaskApproach,
  TaskPlan,
  TaskVerify,
} from "../api/types";
import {
  Button,
  Card,
  EmptyState,
  ErrorBanner,
  KeyValue,
  Modal,
  StatusChip,
  TruthBadge,
} from "../components/ui";
import { formatNumber } from "../format";
import { getSessionId } from "../session";
import { useRobotStore } from "../store/robotStore";

const ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"];

function errorText(error: unknown): string {
  return error instanceof ApiError ? `${error.code}: ${error.message}` : String(error);
}

function delay(ms: number): Promise<void> {
  return new Promise((resolve) => window.setTimeout(resolve, ms));
}

function pulsesText(value: unknown): string {
  if (!value || typeof value !== "object") return "—";
  return Object.entries(value as Record<string, { raw_position?: number }>)
    .map(([joint, item]) => `${joint} ${item?.raw_position ?? "?"}`)
    .join(" · ");
}

function approachText(approach: TaskApproach): string {
  const parts: string[] = [];
  if (Math.abs(approach.forward_mm) >= 0.5) {
    parts.push(
      `${approach.forward_mm > 0 ? "前进" : "后退"} ${Math.abs(
        approach.forward_mm,
      ).toFixed(0)} mm`,
    );
  }
  if (Math.abs(approach.left_mm) >= 0.5) {
    parts.push(
      `${approach.left_mm > 0 ? "左移" : "右移"} ${Math.abs(
        approach.left_mm,
      ).toFixed(0)} mm`,
    );
  }
  return parts.join(" / ") || "微调";
}

function deviationsText(value: unknown): string {
  if (!value || typeof value !== "object") return "";
  return Object.entries(value as Record<string, number>)
    .map(([joint, delta]) => `${joint} ${delta > 0 ? "+" : ""}${delta}`)
    .join(" · ");
}

export function TaskTakeover() {
  const robot = useRobotStore((store) => store.state);
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const lease = useRobotStore((store) => store.lease);
  const setLease = useRobotStore((store) => store.setLease);
  const sessionId = getSessionId();

  const [presets, setPresets] = useState<MotionPresetList | null>(null);
  const [plan, setPlan] = useState<TaskPlan | null>(null);
  const [verify, setVerify] = useState<TaskVerify | null>(null);
  const [stepStatus, setStepStatus] = useState<Record<number, string>>({});
  const [stepActual, setStepActual] = useState<Record<number, unknown>>({});
  const [stepDeviations, setStepDeviations] = useState<Record<number, unknown>>({});
  const [busy, setBusy] = useState(false);
  const [running, setRunning] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [note, setNote] = useState<string | null>(null);
  const [confirmOpen, setConfirmOpen] = useState(false);
  const [approachOpen, setApproachOpen] = useState(false);
  const cancelled = useRef(false);

  useEffect(() => {
    let active = true;
    api
      .motionPresets()
      .then((payload) => active && setPresets(payload))
      .catch((caught) => active && setError(errorText(caught)));
    return () => {
      active = false;
    };
  }, []);

  const taskContract = useMemo(() => {
    const raw = presets?.contract as { task?: Record<string, unknown> } | null;
    return raw?.task ?? null;
  }, [presets]);

  const viewPresetId = String(taskContract?.view_preset ?? "view_table");
  const viewPreset = useMemo<MotionPreset | null>(
    () => presets?.presets.find((item) => item.id === viewPresetId) ?? null,
    [presets, viewPresetId],
  );

  if (!robot || !diagnostics) {
    return <EmptyState title="正在读取状态…">需要后端 /api/state 与 /api/diagnostics。</EmptyState>;
  }

  const vmax = robot.capabilities.max_velocity_rad_s ?? 0.5;
  const minDuration = robot.capabilities.min_duration_s ?? 1.0;
  const maxDuration = robot.capabilities.max_duration_s ?? 10.0;
  const poseScope = robot.capabilities.motion_scope === "pose";
  const hardwareBlocked = robot.mode === "hardware" && !robot.capabilities.hardware_motion;
  const safetyReady = diagnostics.safety.ready;

  const current = Object.fromEntries(
    robot.joints.map((joint) => [joint.name, joint.position_rad]),
  ) as Record<string, number | null>;

  const suggestedDuration = (preset: MotionPreset): number => {
    let delta = 0;
    for (const name of ARM_JOINTS) {
      const from = current[name];
      if (from === null || from === undefined) continue;
      delta = Math.max(delta, Math.abs(preset.joints[name] - from));
    }
    return Math.min(
      maxDuration,
      Math.max(minDuration, Math.ceil((delta / vmax) * 10) / 10),
    );
  };

  const ensureLease = async (): Promise<string> => {
    if (lease) {
      try {
        const renewed = await api.heartbeat(lease.lease_id, sessionId);
        setLease(renewed);
        return renewed.lease_id;
      } catch {
        // fall through to a fresh acquire
      }
    }
    const acquired = await api.acquire(sessionId, 600);
    setLease(acquired);
    return acquired.lease_id;
  };

  const executeTargets = async (
    targets: TargetPayload[],
  ): Promise<{
    stage: string;
    actual?: Record<string, unknown>;
    deviations?: Record<string, unknown>;
    error?: string;
  }> => {
    const preview: PreviewResult = await api.preview(sessionId, targets);
    if (!preview.valid || !preview.preview_token) {
      return {
        stage: "Rejected",
        error: preview.errors.map((item) => `${item.code}: ${item.message}`).join("; "),
      };
    }
    const leaseId = await ensureLease();
    const command = await api.execute({
      sessionId,
      leaseId,
      previewToken: preview.preview_token,
      commandId: preview.command_id,
      targets,
      idempotencyKey: crypto.randomUUID(),
    });
    const duration = Math.max(...targets.map((item) => item.duration_s));
    const deadline = Date.now() + (duration + 15) * 1000;
    while (Date.now() < deadline) {
      const payload = await api.events({ commandId: command.command_id, limit: 100 });
      const events = payload.items.slice().sort((a, b) => a.seq - b.seq);
      const terminal = events.find((event: EventRecord) =>
        ["Completed", "Failed", "Unknown"].includes(event.stage),
      );
      if (terminal) {
        return {
          stage: terminal.stage,
          actual: terminal.details?.actual as Record<string, unknown> | undefined,
          deviations: terminal.details?.deviations_pulses as
            | Record<string, unknown>
            | undefined,
          error: terminal.error_code ?? undefined,
        };
      }
      await delay(300);
    }
    return { stage: "Timeout", error: "no terminal event before the deadline" };
  };

  const prepareView = async () => {
    if (!viewPreset) return;
    setBusy(true);
    setError(null);
    setNote(null);
    try {
      const duration = suggestedDuration(viewPreset);
      const targets: TargetPayload[] = ARM_JOINTS.map((name) => ({
        joint: name,
        target_rad: viewPreset.joints[name],
        duration_s: duration,
      }));
      const result = await executeTargets(targets);
      if (result.stage !== "Completed") {
        throw new Error(`view move: ${result.stage} ${result.error ?? ""}`);
      }
      setNote(
        `已到 view（${duration.toFixed(1)} s，总线校验）；现在可以定位并规划。`,
      );
      setPlan(null);
      setVerify(null);
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const requestPlan = async (): Promise<TaskPlan> => {
    const result = await api.taskPlan();
    setPlan(result);
    if (!result.valid) {
      const codes = result.errors.map((item) => item.code);
      if (codes.includes("APPROACH_REQUIRED")) {
        setError(null);
      } else {
        setError(
          result.errors
            .map((item) => `${item.code}: ${item.message}`)
            .join("; "),
        );
      }
    }
    return result;
  };

  const planTask = async () => {
    setBusy(true);
    setError(null);
    setNote(null);
    setVerify(null);
    setStepStatus({});
    setStepActual({});
    setStepDeviations({});
    try {
      await requestPlan();
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const runApproach = async () => {
    setApproachOpen(false);
    setBusy(true);
    setError(null);
    setNote(null);
    try {
      await ensureLease();
      const approachContract = taskContract?.approach as
        | Record<string, unknown>
        | undefined;
      const maxPulses = Number(approachContract?.max_pulses ?? 3);
      const settle = Math.max(
        0.6,
        Number(approachContract?.settle_s ?? 0.5),
      );
      let current = plan;
      if (!current?.approach) return;
      for (let index = 0; index < maxPulses; index += 1) {
        const approach = current.approach;
        if (!approach) break;
        setNote(
          `底盘接近 ${index + 1}/${maxPulses}：${approachText(approach)}…`,
        );
        const leaseNow = await ensureLease();
        const response = await api.chassisPulse({
          sessionId,
          leaseId: leaseNow,
          vx: approach.vx,
          vy: approach.vy,
          duty: approach.duty,
          durationS: approach.duration_s,
        });
        if (response.status !== "Completed") {
          throw new Error(`底盘接近返回 ${response.status}`);
        }
        await delay(Math.round(settle * 1000));
        current = await requestPlan();
        if (current.valid) {
          setNote(
            `已进入抓取带：目标半径 ${(
              Math.hypot(
                (current.target ?? [0])[0],
                (current.target ?? [0, 0])[1] ?? 0,
              )
            ).toFixed(3)} m，可以运行任务。`,
          );
          break;
        }
        const codes = current.errors.map((item) => item.code);
        if (!codes.includes("APPROACH_REQUIRED") || !current.approach) break;
      }
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setBusy(false);
    }
  };

  const runPlan = async () => {
    if (!plan?.valid) return;
    setConfirmOpen(false);
    setRunning(true);
    cancelled.current = false;
    setError(null);
    setNote(null);
    setVerify(null);
    setStepStatus({});
    setStepActual({});
    setStepDeviations({});
    try {
      await ensureLease();
      for (let index = 0; index < plan.steps.length; index += 1) {
        if (cancelled.current) {
          setNote("任务已由操作员停止（保持请求）。");
          break;
        }
        const step = plan.steps[index];
        setStepStatus((state) => ({ ...state, [index]: "running" }));
        if (step.kind === "verify") {
          const result = await api.taskVerify(plan.plan_id);
          setVerify(result);
          if (!result.held) {
            setStepStatus((state) => ({ ...state, [index]: "failed" }));
            setError(
              result.message ??
                (result.code === "TASK_NOT_AT_VIEW"
                  ? "机械臂不在 view，无法验证夹持"
                  : "木块仍在框内：未夹住，已停止，不再放回"),
            );
            break;
          }
          setStepStatus((state) => ({ ...state, [index]: "held" }));
          continue;
        }
        if (!step.targets || step.targets.length === 0) {
          setStepStatus((state) => ({ ...state, [index]: "skipped" }));
          continue;
        }
        const result = await executeTargets(step.targets);
        setStepActual((state) => ({ ...state, [index]: result.actual }));
        setStepDeviations((state) => ({ ...state, [index]: result.deviations }));
        if (result.stage !== "Completed") {
          setStepStatus((state) => ({ ...state, [index]: result.stage }));
          setError(
            `${step.label}: ${result.stage}${result.error ? ` (${result.error})` : ""}`,
          );
          break;
        }
        setStepStatus((state) => ({ ...state, [index]: "completed" }));
      }
    } catch (caught) {
      setError(errorText(caught));
    } finally {
      setRunning(false);
    }
  };

  const stopTask = async () => {
    cancelled.current = true;
    try {
      await api.stop({ sessionId, leaseId: lease?.lease_id });
    } catch (caught) {
      setError(errorText(caught));
    }
  };

  const requiredApproach =
    plan && !plan.valid && plan.approach
      ? plan.errors.some((item) => item.code === "APPROACH_REQUIRED")
        ? plan.approach
        : null
      : null;
  const planPoint = plan?.point;
  const planTarget = plan?.target;
  const fit = plan?.fit;
  const verifyTone =
    verify?.held === true ? "ok" : verify?.held === false ? "fault" : "warning";
  const verifyLabel =
    verify?.held === true ? "HELD" : verify?.held === false ? "NOT HELD" : "UNVERIFIED";

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">Task Takeover</h1>
          <p className="page-subtitle">
            G8 任务级接管：view 定位 → 深位姿抓取 → 闭爪 → 抬升 → 夹持验证 → 放回。
            每一步仍走预览/执行与总线校验；无急停，停止只是保持请求。
          </p>
        </div>
        <div className="page-actions">
          <StatusChip tone="ok" label={robot.source_label} />
          <StatusChip tone={poseScope ? "ok" : "fault"} label={poseScope ? "pose scope" : "no pose scope"} />
          <StatusChip tone={lease ? "ok" : "warning"} label={lease ? "lease held" : "no lease"} />
        </div>
      </header>

      {!poseScope && (
        <div className="banner banner-fault">
          <strong>Task takeover requires pose scope</strong>
          <span>以 ARMPI_MOTION_SCOPE=pose 与姿态网关启动后端。</span>
        </div>
      )}
      {hardwareBlocked && (
        <div className="banner banner-warning">
          <strong>Hardware motion disabled</strong>
          <span>后端或网关未启用运动；任务不会执行。</span>
        </div>
      )}
      <div className="banner banner-warning">
        <strong>G8 TASK · 无急停 / 无自碰撞模型</strong>
        <span>
          任务会连续执行 13 个动作（约 25 s）。手边保持可断电；第一次运行请清空工作区。
          夹持验证用 view 相机，失败即停，不自动重试。
        </span>
      </div>
      {error && <ErrorBanner message={error} onDismiss={() => setError(null)} />}
      {note && <div className="banner banner-info">{note}</div>}

      <div className="grid grid-2">
        <Card
          title="准备与定位"
          subtitle="先在 view 位测一次点云；定位/规划本身不发运动命令"
          actions={
            <StatusChip
              tone={
                plan?.valid
                  ? "ok"
                  : requiredApproach
                    ? "warning"
                    : plan
                      ? "fault"
                      : "neutral"
              }
              label={
                plan
                  ? plan.valid
                    ? "PLAN VALID"
                    : requiredApproach
                      ? "APPROACH REQUIRED"
                      : "PLAN REJECTED"
                  : "NO PLAN"
              }
            />
          }
        >
          <div className="joint-readout">
            <KeyValue label="View preset">
              <span className="mono">{viewPresetId}</span>
            </KeyValue>
            <KeyValue label="Position source">
              <span className="mono">{presets?.current.source ?? "—"}</span>
            </KeyValue>
            <KeyValue label="Safety">
              <StatusChip tone={safetyReady ? "ok" : "fault"} label={safetyReady ? "ready" : "blocked"} />
            </KeyValue>
          </div>
          <div className="button-row">
            <Button variant="secondary" onClick={() => void prepareView()} disabled={busy || running || !viewPreset}>
              {busy ? "执行中…" : "准备（移动到 view）"}
            </Button>
            <Button variant="primary" onClick={() => void planTask()} disabled={busy || running || !viewPreset}>
              {busy ? "定位中…" : "定位并规划"}
            </Button>
            {requiredApproach && (
              <Button
                variant="primary"
                onClick={() => setApproachOpen(true)}
                disabled={busy || running || hardwareBlocked || !safetyReady}
                title="底盘会慢速脉冲，然后重新拍点云并规划"
              >
                底盘接近（{approachText(requiredApproach)}）
              </Button>
            )}
            <Button
              variant="danger"
              onClick={() => setConfirmOpen(true)}
              disabled={!plan?.valid || running || hardwareBlocked || !safetyReady}
              title={!plan?.valid ? "先定位并规划一个有效的任务" : "确认后连续执行任务"}
            >
              运行任务（需确认）
            </Button>
            <Button variant="secondary" onClick={() => void stopTask()} disabled={!running}>
              停止请求
            </Button>
          </div>
          {requiredApproach && (
            <p className="muted">
              木块在抓取带外：需要底盘{approachText(requiredApproach)}（duty{" "}
              {requiredApproach.duty}，{requiredApproach.duration_s} s，约{" "}
              {requiredApproach.nominal_travel_mm.toFixed(0)} mm），接近后自动重新定位。
            </p>
          )}
          {plan && !plan.valid && plan.errors.length > 0 && !requiredApproach && (
            <ul className="error-list">
              {plan.errors.map((issue, index) => (
                <li key={`${issue.code}-${index}`}>
                  <strong>{issue.code}</strong> {issue.message}
                </li>
              ))}
            </ul>
          )}
          {plan?.valid && (
            <div className="joint-readout">
              <KeyValue label="Cube point">
                <span className="mono">
                  {planPoint
                    ? planPoint.map((value) => formatNumber(value, 4)).join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="Target (+ correction)">
                <span className="mono">
                  {planTarget
                    ? planTarget.map((value) => formatNumber(value, 4)).join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="Fit (j1..j4)">
                <span className="mono">
                  {fit
                    ? fit.joints
                        .slice(0, 4)
                        .map((value) => formatNumber(value, 3))
                        .join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="Fit xy error">
                <span className="mono">
                  {fit ? `${formatNumber(fit.xy_error_mm, 2)} mm` : "—"}
                </span>
              </KeyValue>
              <KeyValue label="Locate source">
                <span className="mono">{plan.source}</span>
              </KeyValue>
              <KeyValue label="Simulated">
                <StatusChip tone={plan.simulated ? "simulation" : "ok"} label={plan.simulated ? "mock" : "live"} />
              </KeyValue>
            </div>
          )}
          {plan?.warnings?.length ? (
            <ul className="warning-list">
              {plan.warnings.map((warning, index) => (
                <li key={`${warning.code}-${index}`}>{warning.message}</li>
              ))}
            </ul>
          ) : null}
        </Card>

        <Card
          title="Steps"
          subtitle={plan?.valid ? `${plan.steps.length} steps · ${plan.view_preset} → ${plan.lift_preset}` : "规划后显示"}
          actions={<StatusChip tone={running ? "warning" : "neutral"} label={running ? "RUNNING" : "IDLE"} />}
        >
          {!plan?.valid ? (
            <p className="muted">先准备 view，然后“定位并规划”。</p>
          ) : (
            <div className="table-wrap">
              <table className="data-table">
                <thead>
                  <tr>
                    <th>#</th>
                    <th>Step</th>
                    <th>Kind</th>
                    <th>Duration</th>
                    <th>Target / raw</th>
                    <th>Status</th>
                    <th>Bus actual</th>
                  </tr>
                </thead>
                <tbody>
                  {plan.steps.map((step, index) => (
                    <tr key={`${step.label}-${index}`}>
                      <td className="mono">{index + 1}</td>
                      <td className="mono">{step.label}</td>
                      <td>{step.kind}</td>
                      <td className="mono">
                        {step.duration_s !== undefined && step.duration_s !== null
                          ? `${step.duration_s} s`
                          : "—"}
                      </td>
                      <td className="mono">
                        {step.kind === "pose"
                          ? ARM_JOINTS.map((name) =>
                              step.target_raw && typeof step.target_raw === "object"
                                ? step.target_raw[name] ?? "?"
                                : "?",
                            ).join("/")
                          : step.kind === "grip"
                            ? String(step.target_raw ?? "—")
                            : "—"}
                      </td>
                      <td>
                        <StatusChip
                          tone={
                            stepStatus[index] === "completed" || stepStatus[index] === "held"
                              ? "ok"
                              : stepStatus[index] === "failed"
                                ? "fault"
                                : stepStatus[index] === "running"
                                  ? "warning"
                                  : "neutral"
                          }
                          label={stepStatus[index] ?? step.status}
                        />
                      </td>
                      <td className="mono">
                        {stepActual[index] ? pulsesText(stepActual[index]) : "—"}
                        {stepDeviations[index] ? (
                          <span className="muted"> ({deviationsText(stepDeviations[index])})</span>
                        ) : null}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
          <p className="footnote">
            每个 pose/grip 步骤仍走 /api/motion/preview → execute；完成后由网关
            <code> GetBusServoState </code> 校验实际寄存器，Unknown 立即停止任务。
          </p>
        </Card>
      </div>

      <Card
        title="夹持验证"
        subtitle="抬升后回到 view，用相机检查木块框是否为空"
        actions={<StatusChip tone={verifyTone} label={verifyLabel} />}
      >
        {!verify ? (
          <p className="muted">任务运行到 verify 步骤时自动执行；也可在 view 位手动验证。</p>
        ) : (
          <>
            <div className="joint-readout">
              <KeyValue label="Held">
                <StatusChip tone={verify.held ? "ok" : "fault"} label={verify.held ? "YES" : "NO"} />
              </KeyValue>
              <KeyValue label="Clusters">
                <span className="mono">{verify.clusters ?? "—"}</span>
              </KeyValue>
              <KeyValue label="Source">
                <span className="mono">{verify.source ?? (verify.simulated ? "simulated" : "—")}</span>
              </KeyValue>
              <KeyValue label="Truth">
                <TruthBadge truth={verify.simulated ? "simulated" : verify.verified ? "measured_on_demand" : "unknown"} />
              </KeyValue>
            </div>
            {verify.warning && <p className="warning-list">{verify.warning}</p>}
            {verify.message && <p className="muted">{verify.message}</p>}
          </>
        )}
      </Card>

      <Modal
        open={approachOpen}
        title="确认底盘接近"
        onClose={() => setApproachOpen(false)}
        footer={
          <>
            <Button variant="ghost" onClick={() => setApproachOpen(false)}>
              取消
            </Button>
            <Button variant="primary" onClick={() => void runApproach()}>
              确认接近
            </Button>
          </>
        }
      >
        {requiredApproach && (
          <>
            <p>
              底盘会慢速脉冲 <strong>{approachText(requiredApproach)}</strong>
              ，机械臂保持在 view；每次脉冲后重新拍点云定位，最多 3 次。底盘没有编码器，
              位置由相机闭环确认。
            </p>
            <KeyValue label="Pulse">
              <span className="mono">
                vx {formatNumber(requiredApproach.vx, 3)} · vy{" "}
                {formatNumber(requiredApproach.vy, 3)} · duty{" "}
                {requiredApproach.duty} · {requiredApproach.duration_s} s
              </span>
            </KeyValue>
            <KeyValue label="Nominal travel">
              <span className="mono">
                {requiredApproach.nominal_travel_mm.toFixed(0)} mm
              </span>
            </KeyValue>
            <KeyValue label="Aim point">
              <span className="mono">
                {requiredApproach.target_point_m
                  .map((value) => formatNumber(value, 3))
                  .join(", ")}{" "}
                m
              </span>
            </KeyValue>
          </>
        )}
      </Modal>

      <Modal
        open={confirmOpen}
        title="确认执行任务"
        onClose={() => setConfirmOpen(false)}
        footer={
          <>
            <Button variant="ghost" onClick={() => setConfirmOpen(false)}>
              取消
            </Button>
            <Button variant="danger" onClick={() => void runPlan()}>
              确认执行 {plan?.steps.length ?? 0} 步
            </Button>
          </>
        }
      >
        {plan?.valid && (
          <>
            <p>
              即将连续执行 <strong>{plan.steps.length}</strong> 个动作：open → approach →
              pre-grasp → grasp → close → lift → view → verify → put-back → open → view。
              每一步都会总线校验；任何 Unknown 立即停止。
            </p>
            <KeyValue label="Cube point">
              <span className="mono">
                {planPoint?.map((value) => formatNumber(value, 4)).join(", ")}
              </span>
            </KeyValue>
            <KeyValue label="Grasp joints">
              <span className="mono">
                {plan.grasp_joints
                  ? ARM_JOINTS.map((name) => formatNumber(plan.grasp_joints?.[name] ?? 0, 3)).join(", ")
                  : "—"}
              </span>
            </KeyValue>
            <KeyValue label="View / lift">
              <span className="mono">
                {plan.view_preset} / {plan.lift_preset}
              </span>
            </KeyValue>
            <p className="muted">
              无急停/抱闸；保持手边可断电。夹持验证失败时不会放回。
            </p>
          </>
        )}
      </Modal>
    </div>
  );
}
