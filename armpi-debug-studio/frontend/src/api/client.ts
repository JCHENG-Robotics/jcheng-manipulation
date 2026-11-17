import { z } from "zod";
import type {
  CommandRecord,
  ConnectionReport,
  Diagnostics,
  EventRecord,
  EventsPayload,
  Lease,
  Peripherals,
  PreviewResult,
  ProbeResult,
  RobotState,
  SafetyReport,
  StopResult,
  TargetPayload,
} from "./types";

const API_BASE = (import.meta.env.VITE_API_BASE ?? "").replace(/\/$/, "");

export class ApiError extends Error {
  code: string;
  status: number;
  details: Record<string, unknown>;

  constructor(code: string, message: string, status = 0, details: Record<string, unknown> = {}) {
    super(message);
    this.name = "ApiError";
    this.code = code;
    this.status = status;
    this.details = details;
  }
}

const JointSchema = z
  .object({
    name: z.string(),
    truth: z.string(),
    validity: z.string(),
    target_rad: z.number().nullable(),
    position_rad: z.number().nullable(),
  })
  .passthrough();

const RobotStateSchema = z
  .object({
    robot_id: z.string(),
    mode: z.enum(["mock", "hardware"]),
    connection_state: z.string(),
    source_label: z.enum(["LIVE", "SIMULATION", "REPLAY"]),
    joints: z.array(JointSchema),
    faults: z.array(z.object({ level: z.string(), code: z.string() }).passthrough()),
    capabilities: z.object({ hardware_motion: z.boolean() }).passthrough(),
    mapping_version: z.string(),
  })
  .passthrough();

const EventSchema = z
  .object({
    seq: z.number(),
    event_id: z.string(),
    stage: z.string(),
    wall_time: z.string(),
    source: z.string(),
    result: z.string(),
  })
  .passthrough();

const EventsPayloadSchema = z
  .object({
    items: z.array(EventSchema),
    last_seq: z.number(),
  })
  .passthrough();

const ErrorEnvelopeSchema = z
  .object({
    error: z.object({
      code: z.string(),
      message: z.string(),
      details: z.record(z.unknown()).optional(),
    }),
  })
  .passthrough();

async function request<T>(path: string, init: RequestInit = {}): Promise<T> {
  const headers = new Headers(init.headers);
  if (init.body && !headers.has("Content-Type")) {
    headers.set("Content-Type", "application/json");
  }
  const response = await fetch(`${API_BASE}${path}`, { ...init, headers });
  const text = await response.text();
  let payload: unknown = null;
  if (text) {
    try {
      payload = JSON.parse(text);
    } catch {
      payload = text;
    }
  }
  if (!response.ok) {
    const parsed = ErrorEnvelopeSchema.safeParse(payload);
    if (parsed.success) {
      throw new ApiError(
        parsed.data.error.code,
        parsed.data.error.message,
        response.status,
        parsed.data.error.details ?? {},
      );
    }
    throw new ApiError("HTTP_ERROR", `request failed with HTTP ${response.status}`, response.status, {
      body: payload,
    });
  }
  return payload as T;
}

function parseRobotState(payload: unknown): RobotState {
  const parsed = RobotStateSchema.safeParse(payload);
  if (!parsed.success) {
    throw new ApiError("CONTRACT_ERROR", "robot state failed schema validation", 0, {
      issues: parsed.error.issues,
    });
  }
  return parsed.data as unknown as RobotState;
}

function parseEvents(payload: unknown): EventsPayload {
  const parsed = EventsPayloadSchema.safeParse(payload);
  if (!parsed.success) {
    throw new ApiError("CONTRACT_ERROR", "events payload failed schema validation", 0, {
      issues: parsed.error.issues,
    });
  }
  return parsed.data as unknown as EventsPayload;
}

function parseEvent(payload: unknown): EventRecord {
  const parsed = EventSchema.safeParse(payload);
  if (!parsed.success) {
    throw new ApiError("CONTRACT_ERROR", "event failed schema validation", 0, {
      issues: parsed.error.issues,
    });
  }
  return parsed.data as unknown as EventRecord;
}

export const api = {
  async state(): Promise<RobotState> {
    return parseRobotState(await request("/api/state"));
  },
  async connection(): Promise<ConnectionReport> {
    return request<ConnectionReport>("/api/connection");
  },
  async safety(params?: { leaseId?: string; sessionId?: string }): Promise<SafetyReport> {
    const query = new URLSearchParams();
    if (params?.leaseId) query.set("lease_id", params.leaseId);
    if (params?.sessionId) query.set("session_id", params.sessionId);
    query.set("require_lease", params?.leaseId ? "true" : "false");
    return request<SafetyReport>(`/api/safety?${query.toString()}`);
  },
  async diagnostics(): Promise<Diagnostics> {
    return request<Diagnostics>("/api/diagnostics");
  },
  async peripherals(): Promise<Peripherals> {
    return request<Peripherals>("/api/peripherals");
  },
  cameraSnapshotUrl(tick: number): string {
    return `${API_BASE}/api/peripherals/camera/snapshot?t=${tick}`;
  },
  async probe(): Promise<ProbeResult> {
    return request<ProbeResult>("/api/diagnostics/probe", { method: "POST" });
  },
  async acquire(sessionId: string, ttlS?: number): Promise<Lease> {
    return request<Lease>("/api/control/acquire", {
      method: "POST",
      body: JSON.stringify({ session_id: sessionId, ttl_s: ttlS }),
    });
  },
  async heartbeat(leaseId: string, sessionId: string): Promise<Lease> {
    return request<Lease>("/api/control/heartbeat", {
      method: "POST",
      body: JSON.stringify({ lease_id: leaseId, session_id: sessionId }),
    });
  },
  async release(leaseId: string, sessionId: string): Promise<{ released: boolean }> {
    return request<{ released: boolean }>("/api/control/release", {
      method: "POST",
      body: JSON.stringify({ lease_id: leaseId, session_id: sessionId }),
    });
  },
  async preview(sessionId: string, targets: TargetPayload[]): Promise<PreviewResult> {
    return request<PreviewResult>("/api/motion/preview", {
      method: "POST",
      body: JSON.stringify({ session_id: sessionId, targets }),
    });
  },
  async execute(input: {
    sessionId: string;
    leaseId: string;
    previewToken: string;
    commandId: string;
    targets: TargetPayload[];
    idempotencyKey: string;
  }): Promise<CommandRecord> {
    return request<CommandRecord>("/api/motion/execute", {
      method: "POST",
      headers: { "Idempotency-Key": input.idempotencyKey },
      body: JSON.stringify({
        session_id: input.sessionId,
        lease_id: input.leaseId,
        preview_token: input.previewToken,
        command_id: input.commandId,
        targets: input.targets,
      }),
    });
  },
  async stop(input: { sessionId: string; leaseId?: string; commandId?: string }): Promise<StopResult> {
    return request<StopResult>("/api/motion/stop", {
      method: "POST",
      body: JSON.stringify({
        session_id: input.sessionId,
        lease_id: input.leaseId,
        command_id: input.commandId,
      }),
    });
  },
  async events(params?: { limit?: number; stage?: string; source?: string; commandId?: string }): Promise<EventsPayload> {
    const query = new URLSearchParams();
    if (params?.limit) query.set("limit", String(params.limit));
    if (params?.stage) query.set("stage", params.stage);
    if (params?.source) query.set("source", params.source);
    if (params?.commandId) query.set("command_id", params.commandId);
    return parseEvents(await request(`/api/events?${query.toString()}`));
  },
  async injectFault(kind: string, joint?: string): Promise<Record<string, unknown>> {
    return request("/api/mock/faults", {
      method: "POST",
      body: JSON.stringify({ kind, joint }),
    });
  },
  async resetMock(): Promise<Record<string, unknown>> {
    return request("/api/mock/reset", { method: "POST" });
  },
  wsUrl(path: string): string {
    const base = API_BASE || window.location.origin;
    return `${base.replace(/^http/, "ws")}${path}`;
  },
  exportUrl(): string {
    return `${API_BASE}/api/events/export.csv`;
  },
  parseState: parseRobotState,
  parseEvent,
};
