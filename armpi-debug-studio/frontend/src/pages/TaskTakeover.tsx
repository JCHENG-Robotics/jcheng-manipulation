import { useEffect, useMemo, useRef, useState } from "react";
import { api, ApiError } from "../api/client";
import type {
  EventRecord,
  MotionPreset,
  MotionPresetList,
  PreviewProfile,
  PreviewResult,
  TargetPayload,
  TaskApproach,
  TaskPlan,
  TaskStep,
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
import { commandLabel, stageLabel, stepKindLabel, stepStatusLabel, taskStepLabel, truthLabel } from "../labels";
import { getSessionId } from "../session";
import { useRobotStore } from "../store/robotStore";

const ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5"];

type TaskProfile = PreviewProfile;

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

function stepCommandsText(step: TaskStep): string {
  if (step.kind === "verify") return "在观察位取点云并检查木块框";
  if (!step.commands || step.commands.length === 0) return "—";
  return step.commands.map((command) => commandLabel(command.label)).join(" → ");
}

function stepRawText(step: TaskStep): string {
  const commands = step.commands ?? [];
  const last = commands[commands.length - 1];
  if (!last) return "—";
  if (last.kind === "pose") {
    const raw = last.target_raw as Record<string, number | null> | null;
    return ARM_JOINTS.map((name) => raw?.[name] ?? "?").join("/");
  }
  return String(last.target_raw ?? "—");
}

function stepSpeedText(step: TaskStep): string {
  const values = (step.commands ?? [])
    .filter((command) => command.profile)
    .map((command) => command.profile?.vmax_rad_s ?? 0);
  if (values.length === 0) return "";
  const unique = Array.from(new Set(values));
  return `vmax ${unique.join("/")} rad/s`;
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
    options?: { profile?: TaskProfile; fast?: boolean },
  ): Promise<{
    stage: string;
    actual?: Record<string, unknown>;
    deviations?: Record<string, unknown>;
    error?: string;
  }> => {
    const preview: PreviewResult = await api.preview(sessionId, targets, options);
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
      profile: options?.profile,
      fast: options?.fast,
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
      await delay(150);
    }
    return { stage: "Timeout", error: "等待超时：截止前没有终态事件" };
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
        throw new Error(`移动到观察位失败：${stageLabel(result.stage)} ${result.error ?? ""}`);
      }
      setNote(
        `已到观察位（${duration.toFixed(1)} s，总线校验）；现在可以定位并规划。`,
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

  const ensureInBand = async (current: TaskPlan | null): Promise<TaskPlan | null> => {
    if (!current) return null;
    const codes = current.errors.map((item) => item.code);
    if (current.valid) return current;
    if (!codes.includes("APPROACH_REQUIRED") || !current.approach) return null;
    const approachContract = taskContract?.approach as
      | Record<string, unknown>
      | undefined;
    const maxPulses = Number(approachContract?.max_pulses ?? 3);
    const settle = Math.max(0.6, Number(approachContract?.settle_s ?? 0.5));
    let next = current;
    for (let index = 0; index < maxPulses; index += 1) {
      if (cancelled.current) return null;
      const approach = next.approach;
      if (!approach) break;
      setNote(`底盘接近 ${index + 1}/${maxPulses}：${approachText(approach)}…`);
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
      next = await requestPlan();
      if (next.valid) {
        setNote(
          `已进入抓取带：目标半径 ${Math.hypot(
            (next.target ?? [0])[0],
            (next.target ?? [0, 0])[1] ?? 0,
          ).toFixed(3)} m。`,
        );
        return next;
      }
      const nextCodes = next.errors.map((item) => item.code);
      if (!nextCodes.includes("APPROACH_REQUIRED") || !next.approach) return null;
    }
    return null;
  };

  const runPlan = async () => {
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
      // One run covers everything: if the cube is out of the grasp band, the
      // base pulses first (camera-closed), the plan is rebuilt, and only then
      // does the arm move.
      const executable = await ensureInBand(plan);
      if (!executable || !executable.valid) {
        const wasApproach =
          plan?.errors.some((item) => item.code === "APPROACH_REQUIRED") ?? false;
        setError(
          wasApproach || !plan || plan.errors.length === 0
            ? "底盘接近后仍未进入抓取带；任务未执行。"
            : plan.errors
                .map((item) => `${item.code}: ${item.message}`)
                .join("; "),
        );
        return;
      }
      setPlan(executable);
      let success = true;
      for (let index = 0; index < executable.steps.length; index += 1) {
        if (cancelled.current) {
          setNote("任务已由操作员停止（保持请求）。");
          success = false;
          break;
        }
        const step = executable.steps[index];
        setStepStatus((state) => ({ ...state, [index]: "running" }));
        if (step.kind === "verify") {
          const result = await api.taskVerify(executable.plan_id);
          setVerify(result);
          if (!result.held) {
            setStepStatus((state) => ({ ...state, [index]: "failed" }));
            setError(
              result.message ??
                (result.code === "TASK_NOT_AT_VIEW"
                  ? "机械臂不在 view，无法验证夹持"
                  : "木块仍在框内：未夹住，已停止，不再放回"),
            );
            success = false;
            break;
          }
          setStepStatus((state) => ({ ...state, [index]: "held" }));
          continue;
        }
        let failed = false;
        for (const command of step.commands) {
          if (cancelled.current) break;
          if (!command.targets || command.targets.length === 0) continue;
          const result = await executeTargets(command.targets, {
            profile: command.profile,
            fast: command.fast,
          });
          setStepActual((state) => ({ ...state, [index]: result.actual }));
          setStepDeviations((state) => ({ ...state, [index]: result.deviations }));
          if (result.stage !== "Completed") {
            setStepStatus((state) => ({ ...state, [index]: result.stage }));
            setError(
              `${step.label} / ${command.label}: ${result.stage}${
                result.error ? ` (${result.error})` : ""
              }`,
            );
            failed = true;
            break;
          }
        }
        if (failed) {
          success = false;
          break;
        }
        if (cancelled.current) {
          setNote("任务已由操作员停止（保持请求）。");
          success = false;
          break;
        }
        setStepStatus((state) => ({ ...state, [index]: "completed" }));
      }
      if (success && !cancelled.current) {
        setNote("任务完成：木块已放回。");
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
    verify?.held === true ? "已夹持" : verify?.held === false ? "未夹住" : "未验证";

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">任务接管</h1>
          <p className="page-subtitle">
            G8 任务级接管：观察位定位 → 深位姿抓取 → 闭爪 → 抬升 → 夹持验证 → 放回。
            每一步仍走预览/执行与总线校验；无急停，停止只是保持请求。
          </p>
        </div>
        <div className="page-actions">
          <StatusChip
            tone={robot.source_label === "SIMULATION" ? "simulation" : "ok"}
            label={robot.source_label === "SIMULATION" ? "模拟" : "实机"}
            detail={robot.source_label}
          />
          <StatusChip tone={poseScope ? "ok" : "fault"} label={poseScope ? "姿态作用域" : "无姿态作用域"} />
          <StatusChip tone={lease ? "ok" : "warning"} label={lease ? "持有控制权" : "无控制权"} />
        </div>
      </header>

      {!poseScope && (
        <div className="banner banner-fault">
          <strong>任务接管需要姿态作用域</strong>
          <span>以 ARMPI_MOTION_SCOPE=pose 与姿态网关启动后端。</span>
        </div>
      )}
      {hardwareBlocked && (
        <div className="banner banner-warning">
          <strong>硬件运动未启用</strong>
          <span>后端或网关未启用运动；任务不会执行。</span>
        </div>
      )}
      <div className="banner banner-warning">
        <strong>G8 任务 · 无急停 / 无自碰撞模型</strong>
        <span>
          一次“运行任务”完成全部动作：木块在抓取带外时先自动底盘接近并重新定位，然后以
          S 曲线（4.0 rad/s / 20 rad/s²）执行 7 个阶段步骤，每步仍走预览/执行与总线校验。
          手边保持可断电；第一次运行请清空工作区。夹持验证失败即停，不自动重试。
        </span>
      </div>
      {error && <ErrorBanner message={error} onDismiss={() => setError(null)} />}
      {note && <div className="banner banner-info">{note}</div>}

      <div className="grid grid-2">
        <Card
          title="准备与定位"
          subtitle="先在观察位测一次点云；定位/规划本身不发运动命令"
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
                    ? "规划有效"
                    : requiredApproach
                      ? "需要底盘接近"
                      : "规划被拒绝"
                  : "未规划"
              }
            />
          }
        >
          <div className="joint-readout">
            <KeyValue label="观察位预设">
              <span className="mono">{viewPresetId}</span>
            </KeyValue>
            <KeyValue label="位置来源">
              <span className="mono">
                {presets?.current.source
                  ? truthLabel(presets.current.source)
                  : "—"}
              </span>
            </KeyValue>
            <KeyValue label="安全门禁">
              <StatusChip tone={safetyReady ? "ok" : "fault"} label={safetyReady ? "就绪" : "已阻止"} />
            </KeyValue>
          </div>
          <div className="button-row">
            <Button variant="secondary" onClick={() => void prepareView()} disabled={busy || running || !viewPreset}>
              {busy ? "执行中…" : "准备（移动到观察位）"}
            </Button>
            <Button variant="primary" onClick={() => void planTask()} disabled={busy || running || !viewPreset}>
              {busy ? "定位中…" : "定位并规划"}
            </Button>
            <Button
              variant="danger"
              onClick={() => setConfirmOpen(true)}
              disabled={(!plan?.valid && !requiredApproach) || running || hardwareBlocked || !safetyReady}
              title={
                !plan
                  ? "先定位并规划"
                  : plan.valid
                    ? "确认后连续执行任务"
                    : requiredApproach
                      ? "运行时会先底盘接近，再重新规划并执行"
                      : "规划无效"
              }
            >
              运行任务（需确认）
            </Button>
            <Button variant="secondary" onClick={() => void stopTask()} disabled={!running}>
              停止请求
            </Button>
          </div>
          {requiredApproach && (
            <p className="muted">
              木块在抓取带外：运行任务时会先自动底盘{approachText(requiredApproach)}
              （duty {requiredApproach.duty}，{requiredApproach.duration_s} s，约{" "}
              {requiredApproach.nominal_travel_mm.toFixed(0)} mm），重新定位后再执行机械臂步骤。
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
              <KeyValue label="木块点">
                <span className="mono">
                  {planPoint
                    ? planPoint.map((value) => formatNumber(value, 4)).join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="目标（含修正）">
                <span className="mono">
                  {planTarget
                    ? planTarget.map((value) => formatNumber(value, 4)).join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="拟合 (j1..j4)">
                <span className="mono">
                  {fit
                    ? fit.joints
                        .slice(0, 4)
                        .map((value) => formatNumber(value, 3))
                        .join(", ")
                    : "—"}
                </span>
              </KeyValue>
              <KeyValue label="拟合 xy 误差">
                <span className="mono">
                  {fit ? `${formatNumber(fit.xy_error_mm, 2)} mm` : "—"}
                </span>
              </KeyValue>
              <KeyValue label="定位来源">
                <span className="mono">{plan.source}</span>
              </KeyValue>
              <KeyValue label="模拟">
                <StatusChip tone={plan.simulated ? "simulation" : "ok"} label={plan.simulated ? "Mock" : "实机"} />
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
          title="阶段"
          subtitle={
            plan?.valid
              ? `${plan.steps.length} 个阶段 · S 曲线 · ${plan.view_preset}`
              : "规划后显示"
          }
          actions={<StatusChip tone={running ? "warning" : "neutral"} label={running ? "运行中" : "空闲"} />}
        >
          {!plan?.valid ? (
            <p className="muted">先准备观察位，然后“定位并规划”。</p>
          ) : (
            <div className="table-wrap">
              <table className="data-table">
                <thead>
                  <tr>
                    <th>#</th>
                    <th>阶段</th>
                    <th>类型</th>
                    <th>时长</th>
                    <th>目标 / 原始值</th>
                    <th>状态</th>
                    <th>总线实测</th>
                  </tr>
                </thead>
                <tbody>
                  {plan.steps.map((step, index) => (
                    <tr key={`${step.label}-${index}`}>
                      <td className="mono">{index + 1}</td>
                      <td className="mono">
                        <strong>{taskStepLabel(step.label)}</strong>
                        <span className="muted"> {stepCommandsText(step)}</span>
                        {stepSpeedText(step) ? (
                          <span className="muted"> ({stepSpeedText(step)})</span>
                        ) : null}
                      </td>
                      <td>{stepKindLabel(step.kind)}</td>
                      <td className="mono">
                        {step.duration_s !== undefined && step.duration_s !== null
                          ? `${formatNumber(step.duration_s, 2)} s`
                          : "—"}
                      </td>
                      <td className="mono">{stepRawText(step)}</td>
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
                          label={stepStatusLabel(stepStatus[index] ?? step.status)}
                          detail={stepStatus[index] ?? step.status}
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
            每条姿态命令是一条 S 曲线（密集路径点，网关逐段复核速度/加速度）；
            夹爪命令走同一条预览/执行与总线校验；结果未知（Unknown）立即停止任务。
          </p>
        </Card>
      </div>

      <Card
        title="夹持验证"
        subtitle="抬升后回到观察位，用相机检查木块框是否为空"
        actions={<StatusChip tone={verifyTone} label={verifyLabel} />}
      >
        {!verify ? (
          <p className="muted">任务运行到验证阶段时自动执行；也可在观察位手动验证。</p>
        ) : (
          <>
            <div className="joint-readout">
              <KeyValue label="已夹住">
                <StatusChip tone={verify.held ? "ok" : "fault"} label={verify.held ? "是" : "否"} />
              </KeyValue>
              <KeyValue label="点云簇">
                <span className="mono">{verify.clusters ?? "—"}</span>
              </KeyValue>
              <KeyValue label="来源">
                <span className="mono">{verify.source ?? (verify.simulated ? "模拟" : "—")}</span>
              </KeyValue>
              <KeyValue label="真值">
                <TruthBadge truth={verify.simulated ? "simulated" : verify.verified ? "measured_on_demand" : "unknown"} />
              </KeyValue>
            </div>
            {verify.warning && <p className="warning-list">{verify.warning}</p>}
            {verify.message && <p className="muted">{verify.message}</p>}
          </>
        )}
      </Card>

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
              {requiredApproach
                ? "确认：底盘接近 + 执行任务"
                : `确认执行 ${plan?.steps.length ?? 0} 个阶段`}
            </Button>
          </>
        }
      >
        {plan && (
          <>
            {requiredApproach ? (
              <p>
                木块在抓取带外。确认后先底盘<strong>{approachText(requiredApproach)}</strong>
                （<span className="mono">
                  vx {formatNumber(requiredApproach.vx, 3)} · vy{" "}
                  {formatNumber(requiredApproach.vy, 3)} · duty{" "}
                  {requiredApproach.duty} · {requiredApproach.duration_s} s
                </span>
                ，约 {requiredApproach.nominal_travel_mm.toFixed(0)} mm），机械臂保持 view；
                每次脉冲后重新定位，进入抓取带后自动执行机械臂步骤。
              </p>
            ) : (
              <p>
                即将连续执行 <strong>{plan.steps.length}</strong> 个阶段：
                {plan.steps.map((step) => step.label).join(" → ")}。
                每一步都会总线校验；任何 Unknown 立即停止。
              </p>
            )}
            {plan.valid && (
              <>
                <KeyValue label="木块点">
                  <span className="mono">
                    {planPoint?.map((value) => formatNumber(value, 4)).join(", ")}
                  </span>
                </KeyValue>
                <KeyValue label="抓取关节">
                  <span className="mono">
                    {plan.grasp_joints
                      ? ARM_JOINTS.map((name) => formatNumber(plan.grasp_joints?.[name] ?? 0, 3)).join(", ")
                      : "—"}
                  </span>
                </KeyValue>
                <KeyValue label="观察位 / 速度">
                  <span className="mono">
                    {plan.view_preset} /{" "}
                    {(plan.fast?.vmax_rad_s as number | undefined) ?? "?"} rad/s ·{" "}
                    {(plan.fast?.amax_rad_s2 as number | undefined) ?? "?"} rad/s²
                  </span>
                </KeyValue>
              </>
            )}
            <p className="muted">
              无急停/抱闸；保持手边可断电。夹持验证失败时不会放回。
            </p>
          </>
        )}
      </Modal>
    </div>
  );
}
