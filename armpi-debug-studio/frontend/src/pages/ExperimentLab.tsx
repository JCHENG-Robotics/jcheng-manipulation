import { useCallback, useEffect, useMemo, useState } from "react";
import { api, ApiError } from "../api/client";
import type {
  ExperimentDetail,
  ExperimentSummary,
  PlanEntry,
  TrajectorySummary,
  TrialRecord,
} from "../api/types";
import {
  Button,
  Card,
  EmptyState,
  KeyValue,
  MetricValue,
  StatusChip,
  TruthBadge,
} from "../components/ui";
import { useRobotStore } from "../store/robotStore";

type Method = "baseline" | "intervention";

interface CreateFormState {
  name: string;
  purpose: string;
  data_origin: "hardware" | "simulated";
  operator: string;
  planned_pairs: string;
  seed: string;
  fixtures: string;
  hardware_config: string;
  calibration_version: string;
  mapping_version: string;
  stop_rules: string;
  notes: string;
}

interface TrialFormState {
  pair_id: string;
  method: Method;
  trial_order: number;
  fixture: string;
  grasp_success: boolean;
  error: string;
  rotation: string;
  duration: string;
  failure_code: string;
  failure_detail: string;
  measurement_source: string;
  trajectory_id: string;
  operator_notes: string;
}

const MEASUREMENT_SOURCES = [
  "unknown",
  "vision",
  "caliper",
  "fixture",
  "operator",
  "simulated",
];

function formatError(caught: unknown): string {
  if (caught instanceof ApiError) {
    const issues = caught.details.issues;
    if (Array.isArray(issues)) {
      const text = issues
        .map((issue) => {
          if (issue && typeof issue === "object") {
            const item = issue as { path?: unknown; message?: unknown };
            return `${String(item.path ?? "?")}: ${String(item.message ?? "invalid")}`;
          }
          return String(issue);
        })
        .join("; ");
      return `${caught.code}: ${caught.message} — ${text}`;
    }
    return `${caught.code}: ${caught.message}`;
  }
  return caught instanceof Error ? caught.message : String(caught);
}

function fmt(value: number | null | undefined, decimals = 3): string {
  return value === null || value === undefined ? "—" : value.toFixed(decimals);
}

function fmtPct(value: number | null | undefined): string {
  return value === null || value === undefined ? "—" : `${(value * 100).toFixed(1)}%`;
}

function fmtP(value: number | null | undefined): string {
  if (value === null || value === undefined) return "—";
  if (value < 0.001) return value.toExponential(2);
  return value.toFixed(4);
}

function emptyCreateForm(): CreateFormState {
  return {
    name: "",
    purpose: "",
    data_origin: "hardware",
    operator: "",
    planned_pairs: "0",
    seed: "7",
    fixtures: "",
    hardware_config: "",
    calibration_version: "",
    mapping_version: "",
    stop_rules: "",
    notes: "",
  };
}

function trialFormFromEntry(
  entry: PlanEntry | null,
  dataOrigin: "hardware" | "simulated" = "hardware",
): TrialFormState {
  return {
    pair_id: entry?.pair_id ?? "",
    method: entry?.method ?? "baseline",
    trial_order: entry?.trial_order ?? 1,
    fixture: entry?.fixture ?? "",
    grasp_success: true,
    error: "",
    rotation: "",
    duration: "",
    failure_code: "GRASP_FAILED",
    failure_detail: "",
    measurement_source: dataOrigin === "simulated" ? "simulated" : "unknown",
    trajectory_id: "",
    operator_notes: "",
  };
}

function nextPlanEntry(detail: ExperimentDetail | null): PlanEntry | null {
  if (!detail) return null;
  const recorded = new Set(detail.trials.map((trial) => `${trial.pair_id}|${trial.method}`));
  return (
    detail.experiment.plan.entries.find(
      (entry) => !recorded.has(`${entry.pair_id}|${entry.method}`),
    ) ?? null
  );
}

export function ExperimentLab() {
  const diagnostics = useRobotStore((store) => store.diagnostics);
  const [items, setItems] = useState<ExperimentSummary[]>([]);
  const [failureCodes, setFailureCodes] = useState<Record<string, string>>({});
  const [methodLabels, setMethodLabels] = useState<Record<string, string>>({});
  const [trajectories, setTrajectories] = useState<TrajectorySummary[]>([]);
  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [detail, setDetail] = useState<ExperimentDetail | null>(null);
  const [loading, setLoading] = useState(true);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [notice, setNotice] = useState<string | null>(null);
  const [showCreate, setShowCreate] = useState(false);
  const [createForm, setCreateForm] = useState<CreateFormState>(emptyCreateForm);
  const [trialForm, setTrialForm] = useState<TrialFormState>(() => trialFormFromEntry(null));

  const refreshList = useCallback(async (prefer?: string) => {
    const list = await api.experiments();
    setItems(list.items);
    setFailureCodes(list.failure_codes);
    setMethodLabels(list.methods);
    setSelectedId((current) => prefer ?? current ?? list.items[0]?.experiment_id ?? null);
  }, []);

  useEffect(() => {
    void (async () => {
      try {
        const [list, trajectoryList] = await Promise.all([api.experiments(), api.trajectories()]);
        setItems(list.items);
        setFailureCodes(list.failure_codes);
        setMethodLabels(list.methods);
        setTrajectories(trajectoryList.items);
        setSelectedId(list.items[0]?.experiment_id ?? null);
      } catch (caught) {
        setError(formatError(caught));
      } finally {
        setLoading(false);
      }
    })();
  }, []);

  useEffect(() => {
    if (!selectedId) {
      setDetail(null);
      return;
    }
    let cancelled = false;
    setBusy(true);
    setError(null);
    void api
      .experiment(selectedId)
      .then((loaded) => {
        if (cancelled) return;
        setDetail(loaded);
        setTrialForm(trialFormFromEntry(nextPlanEntry(loaded), loaded.experiment.data_origin));
      })
      .catch((caught) => {
        if (!cancelled) setError(formatError(caught));
      })
      .finally(() => {
        if (!cancelled) setBusy(false);
      });
    return () => {
      cancelled = true;
    };
  }, [selectedId]);

  const experiment = detail?.experiment ?? null;
  const analysis = detail?.analysis ?? null;
  const planComplete = Boolean(
    experiment &&
      experiment.plan.entries.length > 0 &&
      nextPlanEntry(detail) === null,
  );
  const recordedKeys = useMemo(
    () => new Set((detail?.trials ?? []).map((trial) => `${trial.pair_id}|${trial.method}`)),
    [detail],
  );
  const availablePairs = useMemo(() => {
    if (!experiment) return [];
    return Array.from(
      new Set(
        experiment.plan.entries
          .filter((entry) => !recordedKeys.has(`${entry.pair_id}|${entry.method}`))
          .map((entry) => entry.pair_id),
      ),
    );
  }, [experiment, recordedKeys]);

  const handleDemo = async () => {
    setBusy(true);
    setError(null);
    setNotice(null);
    try {
      const loaded = await api.demoExperiment();
      setDetail(loaded);
      setTrialForm(trialFormFromEntry(nextPlanEntry(loaded), loaded.experiment.data_origin));
      setNotice(
        loaded.duplicate
          ? "示例实验已存在，已选中原记录。"
          : "已加载模拟配对实验（16 对 / 32 trials）；全部标记 simulated。",
      );
      await refreshList(loaded.experiment.experiment_id);
    } catch (caught) {
      setError(formatError(caught));
    } finally {
      setBusy(false);
    }
  };

  const handleCreate = async () => {
    setBusy(true);
    setError(null);
    setNotice(null);
    try {
      const payload = {
        name: createForm.name,
        purpose: createForm.purpose,
        data_origin: createForm.data_origin,
        planned_pairs: Number(createForm.planned_pairs || "0"),
        seed: Number(createForm.seed || "7"),
        fixtures: createForm.fixtures
          .split(",")
          .map((item) => item.trim())
          .filter(Boolean),
        hardware_config: createForm.hardware_config,
        calibration_version: createForm.calibration_version,
        mapping_version: createForm.mapping_version,
        stop_rules: createForm.stop_rules,
        notes: createForm.notes,
        operator: createForm.operator ?? "",
      };
      const created = await api.createExperiment(payload);
      setDetail(created);
      setTrialForm(trialFormFromEntry(nextPlanEntry(created), created.experiment.data_origin));
      setShowCreate(false);
      setCreateForm(emptyCreateForm());
      setNotice(`实验已创建：${created.experiment.name}`);
      await refreshList(created.experiment.experiment_id);
    } catch (caught) {
      setError(formatError(caught));
    } finally {
      setBusy(false);
    }
  };

  const applyEntry = (entry: PlanEntry | null) => {
    if (!entry) return;
    setTrialForm((current) => ({
      ...current,
      pair_id: entry.pair_id,
      method: entry.method,
      trial_order: entry.trial_order,
      fixture: entry.fixture,
    }));
  };

  const handlePairChange = (pairId: string) => {
    const entry =
      experiment?.plan.entries.find(
        (item) => item.pair_id === pairId && !recordedKeys.has(`${item.pair_id}|${item.method}`),
      ) ?? experiment?.plan.entries.find((item) => item.pair_id === pairId);
    setTrialForm((current) => ({
      ...current,
      pair_id: pairId,
      method: entry?.method ?? current.method,
      trial_order: entry?.trial_order ?? current.trial_order,
      fixture: entry?.fixture ?? current.fixture,
    }));
  };

  const handleMethodChange = (method: Method) => {
    const entry = experiment?.plan.entries.find(
      (item) => item.pair_id === trialForm.pair_id && item.method === method,
    );
    setTrialForm((current) => ({
      ...current,
      method,
      trial_order: entry?.trial_order ?? current.trial_order,
      fixture: entry?.fixture ?? current.fixture,
    }));
  };

  const handleRecordTrial = async () => {
    if (!experiment) return;
    setBusy(true);
    setError(null);
    setNotice(null);
    try {
      const payload = {
        pair_id: trialForm.pair_id,
        trial_order: trialForm.trial_order,
        method: trialForm.method,
        fixture: trialForm.fixture,
        grasp_success: trialForm.grasp_success,
        continuous_error_mm:
          trialForm.error.trim() === "" ? null : Number(trialForm.error),
        rotation_error_deg:
          trialForm.rotation.trim() === "" ? null : Number(trialForm.rotation),
        failure_code: trialForm.grasp_success ? null : trialForm.failure_code,
        failure_detail: trialForm.grasp_success ? "" : trialForm.failure_detail,
        duration_s:
          trialForm.duration.trim() === "" ? null : Number(trialForm.duration),
        measurement_source: trialForm.measurement_source,
        trajectory_id: trialForm.trajectory_id || null,
        operator_notes: trialForm.operator_notes,
      };
      await api.recordTrial(experiment.experiment_id, payload);
      const loaded = await api.experiment(experiment.experiment_id);
      setDetail(loaded);
      setTrialForm(trialFormFromEntry(nextPlanEntry(loaded), loaded.experiment.data_origin));
      setNotice(`已记录 trial：${trialForm.pair_id} / ${trialForm.method}`);
      await refreshList(experiment.experiment_id);
    } catch (caught) {
      setError(formatError(caught));
    } finally {
      setBusy(false);
    }
  };

  const fillNext = () => {
    const entry = nextPlanEntry(detail);
    if (entry) applyEntry(entry);
  };

  return (
    <div className="page">
      <header className="page-header">
        <div>
          <h1 className="page-title">Experiment Lab</h1>
          <p className="page-subtitle">
            G3 实验定义与配对 trial 记录：baseline / intervention、失败代码、连续误差与可复算的配对分析。
          </p>
        </div>
        <div className="page-actions">
          <Button variant="secondary" onClick={() => setShowCreate((current) => !current)}>
            {showCreate ? "取消新建" : "新建实验"}
          </Button>
          <Button variant="primary" onClick={() => void handleDemo()} disabled={busy}>
            加载示例实验
          </Button>
        </div>
      </header>

      {error && (
        <div className="banner banner-fault">
          <strong>错误</strong>
          <span className="mono">{error}</span>
        </div>
      )}
      {notice && (
        <div className="banner banner-simulation">
          <strong>实验记录</strong>
          <span>{notice}</span>
        </div>
      )}

      {showCreate && (
        <Card title="New experiment" subtitle="实验定义创建后不可修改；trial 只能追加">
          <div className="experiment-form">
            <label className="field">
              <span>Name *</span>
              <input
                className="input"
                value={createForm.name}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, name: event.target.value }))
                }
              />
            </label>
            <label className="field">
              <span>Data origin</span>
              <select
                className="input"
                value={createForm.data_origin}
                onChange={(event) =>
                  setCreateForm((current) => ({
                    ...current,
                    data_origin: event.target.value as "hardware" | "simulated",
                  }))
                }
              >
                <option value="hardware">hardware</option>
                <option value="simulated">simulated</option>
              </select>
            </label>
            <label className="field">
              <span>Operator</span>
              <input
                className="input"
                value={createForm.operator}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, operator: event.target.value }))
                }
              />
            </label>
            <label className="field experiment-form-wide">
              <span>Purpose *</span>
              <textarea
                className="input"
                rows={2}
                value={createForm.purpose}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, purpose: event.target.value }))
                }
              />
            </label>
            <label className="field">
              <span>Planned pairs</span>
              <input
                className="input"
                type="number"
                min={0}
                max={200}
                value={createForm.planned_pairs}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, planned_pairs: event.target.value }))
                }
              />
            </label>
            <label className="field">
              <span>Randomization seed</span>
              <input
                className="input"
                type="number"
                value={createForm.seed}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, seed: event.target.value }))
                }
              />
            </label>
            <label className="field">
              <span>Fixtures (comma separated)</span>
              <input
                className="input"
                placeholder="fixture_A or fixture_A, fixture_B"
                value={createForm.fixtures}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, fixtures: event.target.value }))
                }
              />
            </label>
            <label className="field">
              <span>Hardware config</span>
              <input
                className="input"
                value={createForm.hardware_config}
                onChange={(event) =>
                  setCreateForm((current) => ({
                    ...current,
                    hardware_config: event.target.value,
                  }))
                }
              />
            </label>
            <label className="field">
              <span>Calibration version</span>
              <input
                className="input"
                value={createForm.calibration_version}
                onChange={(event) =>
                  setCreateForm((current) => ({
                    ...current,
                    calibration_version: event.target.value,
                  }))
                }
              />
            </label>
            <label className="field">
              <span>Mapping version</span>
              <input
                className="input"
                value={createForm.mapping_version}
                onChange={(event) =>
                  setCreateForm((current) => ({
                    ...current,
                    mapping_version: event.target.value,
                  }))
                }
              />
            </label>
            <label className="field experiment-form-wide">
              <span>Stop rules</span>
              <textarea
                className="input"
                rows={2}
                value={createForm.stop_rules}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, stop_rules: event.target.value }))
                }
              />
            </label>
            <label className="field experiment-form-wide">
              <span>Notes</span>
              <textarea
                className="input"
                rows={2}
                value={createForm.notes}
                onChange={(event) =>
                  setCreateForm((current) => ({ ...current, notes: event.target.value }))
                }
              />
            </label>
          </div>
          <div className="button-row form-actions">
            <Button variant="ghost" onClick={() => setShowCreate(false)}>
              取消
            </Button>
            <Button
              variant="primary"
              onClick={() => void handleCreate()}
              disabled={busy || !createForm.name.trim() || !createForm.purpose.trim()}
            >
              创建实验
            </Button>
          </div>
        </Card>
      )}

      <Card
        title="Experiments"
        subtitle={`${items.length} 个实验定义；trial 追加写入本机 SQLite`}
        actions={
          <Button variant="ghost" onClick={() => void refreshList()} disabled={busy}>
            刷新列表
          </Button>
        }
      >
        {loading ? (
          <p className="muted">加载中…</p>
        ) : items.length === 0 ? (
          <EmptyState title="还没有实验定义">
            新建一个实验，或加载模拟示例查看 G3 记录与分析流程。
          </EmptyState>
        ) : (
          <div className="table-wrap">
            <table className="data-table">
              <thead>
                <tr>
                  <th>Name</th>
                  <th>Origin</th>
                  <th>Status</th>
                  <th>Trials</th>
                  <th>Complete pairs</th>
                  <th>Baseline / intervention</th>
                </tr>
              </thead>
              <tbody>
                {items.map((item) => (
                  <tr
                    key={item.experiment_id}
                    className={`row-clickable${selectedId === item.experiment_id ? " row-selected" : ""}`}
                    onClick={() => setSelectedId(item.experiment_id)}
                  >
                    <td>
                      <strong>{item.name}</strong>
                      <div className="mono ellipsis">{item.experiment_id}</div>
                    </td>
                    <td>
                      <TruthBadge truth={item.truth} />
                    </td>
                    <td>
                      <StatusChip
                        tone={
                          item.status === "complete"
                            ? "ok"
                            : item.status === "in_progress"
                              ? "warning"
                              : "neutral"
                        }
                        label={item.status}
                      />
                    </td>
                    <td className="mono">{item.n_trials}</td>
                    <td className="mono">
                      {item.n_complete_pairs}
                      {item.planned_pairs > 0 ? ` / ${item.planned_pairs}` : ""}
                    </td>
                    <td className="mono">
                      {fmtPct(item.baseline_success_rate)} / {fmtPct(item.intervention_success_rate)}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}
      </Card>

      {experiment && analysis && (
        <>
          <Card
            title="Paired analysis"
            subtitle={`${experiment.name} · analysis ${analysis.analysis_version} · seed ${analysis.seed}`}
            actions={<TruthBadge truth={analysis.data_origin === "simulated" ? "simulated" : "measured"} />}
          >
            {!analysis.available ? (
              <div className="banner banner-warning">
                <strong>Analysis unavailable</strong>
                <span>{analysis.reason}</span>
              </div>
            ) : (
              <>
                <div className="metric-row metric-row-4">
                  <MetricValue
                    label="Mean improvement"
                    value={fmt(analysis.paired?.improvement_mm, 3)}
                    unit="mm"
                    tone={(analysis.paired?.improvement_mm ?? 0) > 0 ? "ok" : "warning"}
                    hint="baseline - intervention; positive is better"
                  />
                  <MetricValue
                    label={`${Math.round((analysis.confidence ?? 0.95) * 100)}% CI (diff)`}
                    value={
                      analysis.paired
                        ? `[${fmt(analysis.paired.ci95_difference_mm.low, 2)}, ${fmt(
                            analysis.paired.ci95_difference_mm.high,
                            2,
                          )}]`
                        : "—"
                    }
                    unit="mm"
                    hint="intervention - baseline; negative favors intervention"
                  />
                  <MetricValue
                    label="Complete pairs"
                    value={analysis.validation.n_complete_pairs}
                    hint={`${analysis.validation.n_input} trials recorded`}
                  />
                  <MetricValue
                    label="Sign test p"
                    value={fmtP(analysis.paired?.sign_test.p_value)}
                    hint={
                      analysis.paired
                        ? `${analysis.paired.sign_test.wins} wins / ${analysis.paired.sign_test.losses} losses / ${analysis.paired.sign_test.ties} ties`
                        : "—"
                    }
                  />
                </div>

                {analysis.warnings.length > 0 && (
                  <div className="banner banner-warning">
                    <strong>Interpretation limits</strong>
                    <ul className="warning-list">
                      {analysis.warnings.map((warning) => (
                        <li key={warning}>{warning}</li>
                      ))}
                    </ul>
                  </div>
                )}

                <div className="analysis-grid">
                  <div>
                    <h3 className="analysis-heading">Continuous outcome (mm)</h3>
                    <div className="kv-list">
                      <KeyValue label="Pairs with error">
                        {analysis.paired?.n_pairs_with_error ?? "—"}
                      </KeyValue>
                      <KeyValue label="Mean difference">
                        {fmt(analysis.paired?.difference_mm.mean, 4)} mm
                      </KeyValue>
                      <KeyValue label="Effect size dz">
                        {fmt(analysis.paired?.effect_size_dz, 3)}
                      </KeyValue>
                      <KeyValue label="Order difference">
                        {fmt(analysis.order_effect.difference_mm, 3)} mm
                      </KeyValue>
                      <KeyValue label="Bootstrap">
                        {analysis.bootstrap_samples} draws, seed {analysis.seed}
                      </KeyValue>
                    </div>
                  </div>
                  <div>
                    <h3 className="analysis-heading">Binary outcome</h3>
                    <div className="kv-list">
                      <KeyValue label="Baseline success">
                        {fmtPct(analysis.success?.baseline_rate)}
                      </KeyValue>
                      <KeyValue label="Intervention success">
                        {fmtPct(analysis.success?.intervention_rate)}
                      </KeyValue>
                      <KeyValue label="Discordant (base/int)">
                        {analysis.success
                          ? `${analysis.success.discordant.baseline_only} / ${analysis.success.discordant.intervention_only}`
                          : "—"}
                      </KeyValue>
                      <KeyValue label="McNemar p">
                        {fmtP(analysis.success?.mcnemar_exact_p)}
                      </KeyValue>
                    </div>
                  </div>
                </div>

                <h3 className="analysis-heading">Method summary</h3>
                <div className="table-wrap">
                  <table className="data-table">
                    <thead>
                      <tr>
                        <th>Method</th>
                        <th>n</th>
                        <th>Error mean</th>
                        <th>Error SD</th>
                        <th>Rotation mean</th>
                        <th>Success</th>
                        <th>Failure codes</th>
                      </tr>
                    </thead>
                    <tbody>
                      {(["baseline", "intervention"] as Method[]).map((method) => {
                        const summary = analysis.methods[method];
                        if (!summary) return null;
                        const codes = Object.entries(summary.failure_codes)
                          .map(([code, count]) => `${code}=${count}`)
                          .join(", ");
                        return (
                          <tr key={method}>
                            <td>
                              <strong>{methodLabels[method] ?? method}</strong>
                              <div className="muted">{method}</div>
                            </td>
                            <td className="mono">{summary.n}</td>
                            <td className="mono">{fmt(summary.error.mean, 3)} mm</td>
                            <td className="mono">{fmt(summary.error.std, 3)} mm</td>
                            <td className="mono">{fmt(summary.rotation_error_deg.mean, 3)}°</td>
                            <td className="mono">{fmtPct(summary.success_rate)}</td>
                            <td className="mono ellipsis" title={codes || undefined}>
                              {codes || "—"}
                            </td>
                          </tr>
                        );
                      })}
                    </tbody>
                  </table>
                </div>
              </>
            )}
          </Card>

          <div className="experiment-layout">
            <div className="experiment-main">
              <Card
                title="Record trial"
                subtitle="追加式记录；成功后 pair/method 不可重复"
                actions={
                  <Button
                    variant="secondary"
                    onClick={fillNext}
                    disabled={busy || !nextPlanEntry(detail)}
                  >
                    下一计划配对
                  </Button>
                }
              >
                {planComplete ? (
                  <div className="banner banner-warning">
                    <strong>Plan complete</strong>
                    <span>随机化计划中的全部 pair/method 都已记录；不能再追加。</span>
                  </div>
                ) : (
                  <>
                    <div className="trial-form">
                      <label className="field">
                        <span>Pair ID</span>
                        {experiment.plan.entries.length > 0 ? (
                          <select
                            className="input"
                            value={trialForm.pair_id}
                            onChange={(event) => handlePairChange(event.target.value)}
                          >
                            {availablePairs.map((pairId) => (
                              <option key={pairId} value={pairId}>
                                {pairId}
                              </option>
                            ))}
                          </select>
                        ) : (
                          <input
                            className="input"
                            value={trialForm.pair_id}
                            onChange={(event) =>
                              setTrialForm((current) => ({
                                ...current,
                                pair_id: event.target.value,
                              }))
                            }
                          />
                        )}
                      </label>
                      <label className="field">
                        <span>Method</span>
                        <select
                          className="input"
                          value={trialForm.method}
                          onChange={(event) => handleMethodChange(event.target.value as Method)}
                        >
                          <option value="baseline">baseline</option>
                          <option value="intervention">intervention</option>
                        </select>
                      </label>
                      <label className="field">
                        <span>Trial order</span>
                        <select
                          className="input"
                          value={trialForm.trial_order}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              trial_order: Number(event.target.value),
                            }))
                          }
                        >
                          <option value={1}>1</option>
                          <option value={2}>2</option>
                        </select>
                      </label>
                      <label className="field">
                        <span>Fixture</span>
                        <input
                          className="input"
                          value={trialForm.fixture}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              fixture: event.target.value,
                            }))
                          }
                        />
                      </label>
                      <label className="field">
                        <span>Result</span>
                        <select
                          className="input"
                          value={trialForm.grasp_success ? "success" : "failure"}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              grasp_success: event.target.value === "success",
                            }))
                          }
                        >
                          <option value="success">success</option>
                          <option value="failure">failure</option>
                        </select>
                      </label>
                      <label className="field">
                        <span>Error (mm) *</span>
                        <input
                          className="input"
                          type="number"
                          min={0}
                          step="0.01"
                          value={trialForm.error}
                          onChange={(event) =>
                            setTrialForm((current) => ({ ...current, error: event.target.value }))
                          }
                        />
                      </label>
                      <label className="field">
                        <span>Rotation error (°)</span>
                        <input
                          className="input"
                          type="number"
                          min={0}
                          step="0.01"
                          value={trialForm.rotation}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              rotation: event.target.value,
                            }))
                          }
                        />
                      </label>
                      <label className="field">
                        <span>Duration (s)</span>
                        <input
                          className="input"
                          type="number"
                          min={0}
                          step="0.1"
                          value={trialForm.duration}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              duration: event.target.value,
                            }))
                          }
                        />
                      </label>
                      <label className="field">
                        <span>Failure code</span>
                        <select
                          className="input"
                          value={trialForm.failure_code}
                          disabled={trialForm.grasp_success}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              failure_code: event.target.value,
                            }))
                          }
                        >
                          {Object.entries(failureCodes).map(([code, label]) => (
                            <option key={code} value={code}>
                              {code} — {label}
                            </option>
                          ))}
                        </select>
                      </label>
                      <label className="field">
                        <span>Failure detail</span>
                        <input
                          className="input"
                          value={trialForm.failure_detail}
                          disabled={trialForm.grasp_success}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              failure_detail: event.target.value,
                            }))
                          }
                        />
                      </label>
                      <label className="field">
                        <span>Measurement source</span>
                        <select
                          className="input"
                          value={trialForm.measurement_source}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              measurement_source: event.target.value,
                            }))
                          }
                        >
                          {MEASUREMENT_SOURCES.map((source) => (
                            <option key={source} value={source}>
                              {source}
                            </option>
                          ))}
                        </select>
                      </label>
                      <label className="field">
                        <span>Trajectory dataset</span>
                        <select
                          className="input"
                          value={trialForm.trajectory_id}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              trajectory_id: event.target.value,
                            }))
                          }
                        >
                          <option value="">none</option>
                          {trajectories.map((trajectory) => (
                            <option key={trajectory.trajectory_id} value={trajectory.trajectory_id}>
                              {trajectory.name}
                            </option>
                          ))}
                        </select>
                      </label>
                      <label className="field trial-form-wide">
                        <span>Operator notes</span>
                        <input
                          className="input"
                          value={trialForm.operator_notes}
                          onChange={(event) =>
                            setTrialForm((current) => ({
                              ...current,
                              operator_notes: event.target.value,
                            }))
                          }
                        />
                      </label>
                    </div>
                    <div className="button-row form-actions">
                      <Button variant="primary" onClick={() => void handleRecordTrial()} disabled={busy}>
                        记录 trial
                      </Button>
                      <span className="hint">
                        成功 trial 必须填写连续误差；失败 trial 必须选择失败代码。数据来源由实验定义决定。
                      </span>
                    </div>
                  </>
                )}
              </Card>

              <Card
                title="Recorded trials"
                subtitle={`${detail?.trials.length ?? 0} 条追加记录；按 pair 和 trial_order 排序`}
              >
                {detail && detail.trials.length > 0 ? (
                  <div className="table-wrap">
                    <table className="data-table">
                      <thead>
                        <tr>
                          <th>Pair</th>
                          <th>#</th>
                          <th>Method</th>
                          <th>Fixture</th>
                          <th>Result</th>
                          <th>Error (mm)</th>
                          <th>Rotation (°)</th>
                          <th>Duration (s)</th>
                          <th>Truth</th>
                          <th>Trajectory</th>
                        </tr>
                      </thead>
                      <tbody>
                        {detail.trials.map((trial: TrialRecord) => (
                          <tr key={trial.trial_id}>
                            <td className="mono">{trial.pair_id}</td>
                            <td className="mono">{trial.trial_order}</td>
                            <td>{trial.method}</td>
                            <td>{trial.fixture}</td>
                            <td>
                              <StatusChip
                                tone={trial.grasp_success ? "ok" : "fault"}
                                label={trial.grasp_success ? "success" : "failure"}
                                detail={trial.failure_detail || undefined}
                              />
                              {trial.failure_code && (
                                <div className="mono trial-failure">{trial.failure_code}</div>
                              )}
                            </td>
                            <td className="mono">
                              {trial.continuous_error_mm === null
                                ? "—"
                                : trial.continuous_error_mm.toFixed(3)}
                            </td>
                            <td className="mono">
                              {trial.rotation_error_deg === null
                                ? "—"
                                : trial.rotation_error_deg.toFixed(2)}
                            </td>
                            <td className="mono">
                              {trial.duration_s === null ? "—" : trial.duration_s.toFixed(2)}
                            </td>
                            <td>
                              <TruthBadge truth={trial.truth} />
                            </td>
                            <td className="mono ellipsis">
                              {trial.trajectory_id ?? "—"}
                            </td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                ) : (
                  <EmptyState title="还没有 trial">
                    使用上面的表单记录第一条 baseline 或 intervention trial。
                  </EmptyState>
                )}
              </Card>
            </div>

            <aside className="experiment-side">
              <Card title="Definition" subtitle="创建后不可修改">
                <div className="kv-list">
                  <KeyValue label="ID">{experiment.experiment_id}</KeyValue>
                  <KeyValue label="Created">{experiment.created_at}</KeyValue>
                  <KeyValue label="Origin">
                    <TruthBadge truth={experiment.data_origin === "simulated" ? "simulated" : "measured"} />
                  </KeyValue>
                  <KeyValue label="Operator">{experiment.operator || "—"}</KeyValue>
                  <KeyValue label="Robot">{experiment.robot_id || "—"}</KeyValue>
                  <KeyValue label="Calibration">{experiment.calibration_version || "—"}</KeyValue>
                  <KeyValue label="Mapping">{experiment.mapping_version || "—"}</KeyValue>
                  <KeyValue label="Plan">
                    {experiment.protocol.planned_pairs > 0
                      ? `${experiment.plan.n_pairs} pairs, seed ${experiment.plan.seed}`
                      : "manual (no plan)"}
                  </KeyValue>
                  <KeyValue label="Successes">
                    {analysis?.success
                      ? `${fmtPct(analysis.success.baseline_rate)} / ${fmtPct(
                          analysis.success.intervention_rate,
                        )}`
                      : "—"}
                  </KeyValue>
                </div>
                <p className="hint">{experiment.stop_rules || "No stop rules recorded."}</p>
              </Card>

              <Card title="Exports" subtitle="CSV 可用脚本复算；JSON 含定义、trials 与分析">
                <div className="button-row export-row">
                  <a
                    className="button button-secondary"
                    href={api.experimentExportUrl(experiment.experiment_id, "csv")}
                    download
                  >
                    Trials CSV
                  </a>
                  <a
                    className="button button-secondary"
                    href={api.experimentExportUrl(experiment.experiment_id, "json")}
                    download
                  >
                    Metadata JSON
                  </a>
                </div>
              </Card>

              <Card title="Failure codes" subtitle="失败 trial 必须归类">
                {analysis && analysis.available ? (
                  <div className="table-wrap">
                    <table className="data-table">
                      <thead>
                        <tr>
                          <th>Code</th>
                          <th>Baseline</th>
                          <th>Intervention</th>
                        </tr>
                      </thead>
                      <tbody>
                        {Object.keys(analysis.methods.baseline?.failure_codes ?? {}).length === 0 &&
                        Object.keys(analysis.methods.intervention?.failure_codes ?? {}).length ===
                          0 ? (
                          <tr>
                            <td colSpan={3} className="muted">
                              没有失败记录
                            </td>
                          </tr>
                        ) : (
                          Array.from(
                            new Set([
                              ...Object.keys(analysis.methods.baseline?.failure_codes ?? {}),
                              ...Object.keys(analysis.methods.intervention?.failure_codes ?? {}),
                            ]),
                          )
                            .sort()
                            .map((code) => (
                              <tr key={code}>
                                <td className="mono">{code}</td>
                                <td className="mono">
                                  {analysis.methods.baseline?.failure_codes[code] ?? 0}
                                </td>
                                <td className="mono">
                                  {analysis.methods.intervention?.failure_codes[code] ?? 0}
                                </td>
                              </tr>
                            ))
                        )}
                      </tbody>
                    </table>
                  </div>
                ) : (
                  <p className="muted">暂无分析</p>
                )}
              </Card>

              <Card title="Protocol notes" subtitle="方法学限制与可追溯信息">
                <ul className="note-list">
                  {analysis?.limitations.map((item) => <li key={item}>{item}</li>)}
                  <li>
                    分析版本 {analysis?.analysis_version ?? "—"}；同一 trials 与 seed 可复算。
                  </li>
                  <li>
                    {diagnostics?.mapping.verification_status === "provisional" && experiment.data_origin === "hardware"
                      ? "当前映射仍为 provisional；不要据此宣称硬件显著改善。"
                      : "模拟实验只用于展示流程，不是硬件验证。"}
                  </li>
                </ul>
              </Card>
            </aside>
          </div>
        </>
      )}
    </div>
  );
}
