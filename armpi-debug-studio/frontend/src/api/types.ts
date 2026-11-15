export type Mode = "mock" | "hardware";
export type SourceLabel = "LIVE" | "SIMULATION" | "REPLAY";

export interface Capabilities {
  hardware_motion: boolean;
  hardware_read: boolean;
  mock_faults: boolean;
  replay: boolean;
  min_duration_s: number;
  max_duration_s: number;
  max_step_rad: number;
}

export interface JointState {
  name: string;
  servo_id: number;
  target_rad: number | null;
  target_raw: number | null;
  position_rad: number | null;
  raw_position: number | null;
  truth: string;
  validity: string;
  sample_age_ms: number | null;
  source: string;
  limit_min_rad: number;
  limit_max_rad: number;
  limit_verified: boolean;
}

export interface Fault {
  fault_id: string;
  level: string;
  code: string;
  message: string;
  source: string;
  first_seen: string;
  last_seen: string;
  remediation: string;
  details: Record<string, unknown>;
}

export interface RobotState {
  robot_id: string;
  mode: Mode;
  connection_state: string;
  source_label: SourceLabel;
  control_owner: string | null;
  received_at: string;
  monotonic_ms: number;
  ros_time_ns: number | null;
  joints: JointState[];
  faults: Fault[];
  mapping_version: string;
  limit_version: string;
  mapping_verified: boolean;
  capabilities: Capabilities;
  stop_state: string | null;
  last_command_id: string | null;
}

export interface SegmentStatus {
  name: string;
  state: string;
  detail: string;
  evidence: string;
}

export interface ConnectionReport {
  segments: SegmentStatus[];
  serial_available: boolean;
  driver_online: boolean;
  gateway_online: boolean;
  feedback_fresh: boolean;
  notes: string[];
}

export interface SafetyCheck {
  check_id: string;
  label: string;
  passed: boolean;
  blocking: boolean;
  detail: string;
  severity: string;
}

export interface SafetyReport {
  ready: boolean;
  checks: SafetyCheck[];
  blockers: string[];
  warnings: string[];
  mode: Mode;
  generated_at: string;
}

export interface MappingJoint {
  name: string;
  servo_id: number;
  init_raw: number;
  min_raw: number;
  max_raw: number;
  direction: number;
  radians_per_tick: number;
  min_rad: number;
  max_rad: number;
  limit_source: string;
  verified: boolean;
}

export interface MappingInfo {
  schema_version: number;
  mapping_version: string;
  source: string;
  verified: boolean;
  verification_status: string;
  radians_per_tick: number;
  mapping_hash: string;
  joints: MappingJoint[];
  excluded: Array<Record<string, unknown>>;
  safety: Record<string, number>;
}

export interface KnownDefect {
  id: string;
  summary: string;
  source: string;
  impact: string;
}

export interface MockFault {
  kind: string;
  code: string;
  level: string;
  joint: string | null;
}

export interface Diagnostics {
  mode: Mode;
  connection: ConnectionReport;
  mapping: MappingInfo;
  safety: SafetyReport;
  capabilities: Capabilities;
  known_defects: KnownDefect[];
  probe_available: boolean;
  environment: Record<string, unknown>;
  mock_faults: MockFault[];
  storage_error: string | null;
}

export interface EventRecord {
  seq: number;
  event_id: string;
  command_id: string | null;
  stage: string;
  wall_time: string;
  monotonic_ms: number;
  source: string;
  result: string;
  error_code: string | null;
  details: Record<string, unknown>;
  schema_version: number;
}

export interface EventsPayload {
  items: EventRecord[];
  last_seq: number;
  schema_version: number;
}

export interface ValidationIssue {
  code: string;
  message: string;
  joint: string | null;
  details: Record<string, unknown>;
}

export interface PreviewTarget {
  joint: string;
  servo_id: number;
  from_rad: number;
  target_rad: number;
  delta_rad: number;
  duration_s: number;
  target_raw: number;
  limit_min_rad: number;
  limit_max_rad: number;
  limit_verified: boolean;
  limit_source: string;
}

export interface PreviewResult {
  command_id: string;
  session_id: string;
  valid: boolean;
  targets: PreviewTarget[];
  errors: ValidationIssue[];
  warnings: ValidationIssue[];
  preview_token: string | null;
  expires_at: string | null;
  mapping_version: string;
  evaluated_at: string;
}

export interface Lease {
  lease_id: string;
  session_id: string;
  acquired_at: string;
  expires_at: string;
  ttl_s: number;
}

export interface CommandRecord {
  command_id: string;
  session_id: string;
  status: string;
  targets: PreviewTarget[];
  created_at: string;
  updated_at: string;
  error_code: string | null;
  details: Record<string, unknown>;
  stage_history: Array<{ stage: string; at: string; error_code?: string | null; details?: Record<string, unknown> }>;
}

export interface StopResult {
  state: string;
  command_id: string;
  detail: string;
  evidence: string;
  requested_at: string;
  observed_at: string | null;
  storage_error: Record<string, unknown> | null;
}

export interface ProbeResult {
  truth: string;
  note: string;
  results: Array<{
    joint: string;
    servo_id: number;
    ok: boolean;
    raw_position: number | null;
    position_rad: number | null;
    duration_ms: number;
    truth: string;
    error: string | null;
  }>;
}

export interface TargetPayload {
  joint: string;
  target_rad: number;
  duration_s: number;
}
