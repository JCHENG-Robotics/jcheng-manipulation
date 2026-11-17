#!/usr/bin/env python3
"""
Read-only host and ROS/DDS benchmark for the ArmPi experiment setup.

Two modes:

* ``host`` collects CPU, memory, disk, temperature, kernel, Python and ROS
  environment facts. It uses only the standard library and a short CPU
  sampling window.
* ``ros`` subscribes to one or more topics for a fixed duration and reports
  received-message rate, inter-arrival jitter, header age and publisher
  counts. It never publishes and never commands the robot.

The file is intentionally standalone so it can be piped to the robot or a
container over SSH without installing the package there.

Header age compares the sender's ``header.stamp`` with the receiver's wall
clock. On different hosts it includes clock offset; the inter-arrival
statistics are clock-offset-free.
"""

import argparse
from datetime import datetime, timezone
import glob
import json
import os
import platform
import shutil
import socket
import statistics
import sys
import time

BENCHMARK_LIMITATIONS = [
    'No command or motion is issued; both modes are read-only.',
    'Header age across hosts includes sender/receiver clock offset.',
    'Topic rate is the received rate, not the sensor exposure rate.',
    'CPU utilization is a short /proc/stat sample, not a long-run average.',
    'Container CPU/memory limits are reported only when cgroup files are '
    'visible.',
]


def _read_first_line(path):
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return handle.readline().strip()
    except OSError:
        return None


def _read_cpuinfo():
    values = {}
    try:
        with open('/proc/cpuinfo', 'r', encoding='utf-8') as handle:
            for line in handle:
                if ':' not in line:
                    continue
                key, value = line.split(':', 1)
                values.setdefault(key.strip().lower(), value.strip())
    except OSError:
        return None
    for key in ('model name', 'hardware', 'model'):
        if values.get(key):
            return values[key]
    return None


def _read_cpu_times():
    line = _read_first_line('/proc/stat')
    if not line or not line.startswith('cpu '):
        return None
    try:
        values = [int(item) for item in line.split()[1:]]
    except ValueError:
        return None
    if len(values) < 4:
        return None
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def _cpu_utilization(sample_s):
    start = _read_cpu_times()
    if start is None:
        return None
    time.sleep(sample_s)
    end = _read_cpu_times()
    if end is None:
        return None
    total_delta = end[0] - start[0]
    idle_delta = end[1] - start[1]
    if total_delta <= 0:
        return None
    utilization = 100.0 * (1.0 - float(idle_delta) / float(total_delta))
    return max(0.0, min(100.0, utilization))


def _read_meminfo():
    values = {}
    try:
        with open('/proc/meminfo', 'r', encoding='utf-8') as handle:
            for line in handle:
                if ':' not in line:
                    continue
                key, value = line.split(':', 1)
                parts = value.split()
                if not parts:
                    continue
                try:
                    values[key.strip()] = int(parts[0]) * 1024
                except ValueError:
                    continue
    except OSError:
        return {}
    return values


def _read_load():
    line = _read_first_line('/proc/loadavg')
    if not line:
        return {}
    parts = line.split()
    if len(parts) < 3:
        return {}
    try:
        return {
            'load_1m': float(parts[0]),
            'load_5m': float(parts[1]),
            'load_15m': float(parts[2]),
        }
    except ValueError:
        return {}


def _read_temperature():
    values = []
    for path in sorted(glob.glob('/sys/class/thermal/thermal_zone*/temp')):
        raw = _read_first_line(path)
        if raw is None:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        if abs(value) > 1000.0:
            value /= 1000.0
        if -40.0 <= value <= 150.0:
            values.append(value)
    return max(values) if values else None


def _read_cgroup_memory_limit_mb():
    for path in (
        '/sys/fs/cgroup/memory.max',
        '/sys/fs/cgroup/memory/memory.limit_in_bytes',
    ):
        raw = _read_first_line(path)
        if raw is None or raw == 'max':
            continue
        try:
            limit = int(raw)
        except ValueError:
            continue
        if limit > 0:
            return limit / (1024.0 * 1024.0)
    return None


def _in_container():
    return bool(
        os.path.exists('/.dockerenv')
        or os.path.exists('/run/.containerenv')
    )


def _ros_environment():
    return {
        'distro': os.environ.get('ROS_DISTRO'),
        'rmw': os.environ.get('RMW_IMPLEMENTATION'),
        'domain_id': os.environ.get('ROS_DOMAIN_ID'),
        'localhost_only': os.environ.get('ROS_LOCALHOST_ONLY'),
        'cyclonedds_uri': os.environ.get('CYCLONEDDS_URI'),
    }


def collect_host(sample_s=1.0):
    """Collect host facts with a short CPU sampling window."""
    if sample_s < 0.0 or sample_s > 30.0:
        raise ValueError('sample_s must be between 0 and 30')
    memory = _read_meminfo()
    total = memory.get('MemTotal')
    available = memory.get('MemAvailable')
    used = total - available if total and available is not None else None
    memory_percent = (
        used / total * 100.0 if total and used is not None else None
    )
    disk = shutil.disk_usage('/')
    disk_percent = disk.used / disk.total * 100.0 if disk.total else None
    cpu_utilization = _cpu_utilization(sample_s) if sample_s > 0 else None
    cgroup_limit = _read_cgroup_memory_limit_mb()
    return {
        'schema_version': 1,
        'mode': 'host',
        'collected_at': datetime.now(timezone.utc).isoformat(),
        'hostname': socket.gethostname(),
        'platform': platform.platform(),
        'kernel': platform.release(),
        'architecture': platform.machine(),
        'python': platform.python_version(),
        'container': _in_container(),
        'cpu': {
            'model': _read_cpuinfo(),
            'logical_cores': os.cpu_count(),
            'utilization_percent': cpu_utilization,
            'sample_s': sample_s,
            'load': _read_load(),
        },
        'memory': {
            'total_mb': total / (1024.0 * 1024.0) if total else None,
            'available_mb': (
                available / (1024.0 * 1024.0)
                if available is not None
                else None
            ),
            'used_mb': used / (1024.0 * 1024.0) if used is not None else None,
            'used_percent': memory_percent,
            'cgroup_limit_mb': cgroup_limit,
        },
        'disk': {
            'path': '/',
            'total_gb': disk.total / (1024.0 ** 3),
            'used_gb': disk.used / (1024.0 ** 3),
            'free_gb': disk.free / (1024.0 ** 3),
            'used_percent': disk_percent,
        },
        'temperature_c': _read_temperature(),
        'ros': _ros_environment(),
        'limitations': BENCHMARK_LIMITATIONS,
    }


def _summary_ns(values):
    if not values:
        return {
            'n': 0,
            'mean_ms': None,
            'std_ms': None,
            'min_ms': None,
            'max_ms': None,
            'p95_ms': None,
        }
    milliseconds = [value / 1e6 for value in values]
    ordered = sorted(milliseconds)
    index = int(round(0.95 * (len(ordered) - 1)))
    return {
        'n': len(milliseconds),
        'mean_ms': statistics.mean(milliseconds),
        'std_ms': (
            statistics.stdev(milliseconds)
            if len(milliseconds) > 1
            else 0.0
        ),
        'min_ms': min(milliseconds),
        'max_ms': max(milliseconds),
        'p95_ms': ordered[index],
    }


def _summarize_topic(topic, message_type, publishers, receipts, ages,
                     payload_bytes):
    intervals = [
        receipts[index] - receipts[index - 1]
        for index in range(1, len(receipts))
    ]
    rate = None
    if len(receipts) > 1 and receipts[-1] > receipts[0]:
        rate = (
            (len(receipts) - 1)
            / ((receipts[-1] - receipts[0]) / 1e9)
        )
    return {
        'topic': topic,
        'type': message_type,
        'count': len(receipts),
        'rate_hz': rate,
        'interval': _summary_ns(intervals),
        'header_age': _summary_ns(ages),
        'publishers': publishers,
        'payload_bytes': payload_bytes,
        'error': None if receipts else 'no messages received',
    }


def measure_ros(topics, duration_s=10.0, qos='sensor_data',
                node_name='armpi_host_benchmark'):
    """Subscribe to topics for a fixed duration and summarize receipt."""
    if duration_s <= 0.0 or duration_s > 60.0:
        raise ValueError('duration_s must be between 0 and 60')
    if not topics:
        raise ValueError('at least one topic is required')
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import (
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            'ROS 2 Python modules are unavailable: {}'.format(exc)
        )
    rclpy.init(args=None)
    node = Node(node_name)
    subscriptions = []
    state = {}
    try:
        deadline = time.monotonic() + 5.0
        graph = {}
        while time.monotonic() < deadline:
            graph = dict(node.get_topic_names_and_types())
            if all(topic in graph for topic in topics):
                break
            time.sleep(0.1)
        if qos == 'reliable':
            profile = QoSProfile(
                depth=10,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        else:
            profile = qos_profile_sensor_data
        for topic in topics:
            types = graph.get(topic)
            if not types:
                state[topic] = {
                    'topic': topic,
                    'type': None,
                    'count': 0,
                    'rate_hz': None,
                    'interval': _summary_ns([]),
                    'header_age': _summary_ns([]),
                    'publishers': 0,
                    'payload_bytes': 0,
                    'error': 'topic not present in the ROS graph',
                }
                continue
            message_type = get_message(types[0])
            receipts = []
            ages = []
            payload = {'bytes': 0}

            def callback(message, receipts=receipts, ages=ages,
                         payload=payload):
                receipts.append(time.monotonic_ns())
                data = getattr(message, 'data', None)
                if data is not None and hasattr(data, '__len__'):
                    payload['bytes'] += len(data)
                header = getattr(message, 'header', None)
                if header is not None:
                    stamp = header.stamp
                    stamp_ns = (
                        int(stamp.sec) * 1_000_000_000
                        + int(stamp.nanosec)
                    )
                    if stamp_ns > 0:
                        ages.append(time.time_ns() - stamp_ns)

            subscription = node.create_subscription(
                message_type, topic, callback, profile
            )
            subscriptions.append(subscription)
            state[topic] = {
                'topic': topic,
                'type': types[0],
                'receipts': receipts,
                'ages': ages,
                'payload': payload,
            }
        start = time.monotonic()
        while time.monotonic() - start < duration_s:
            rclpy.spin_once(node, timeout_sec=0.1)
        publishers = {}
        for topic in topics:
            try:
                publishers[topic] = len(
                    node.get_publishers_info_by_topic(topic)
                )
            except Exception:
                publishers[topic] = None
        try:
            nodes = len(node.get_node_names())
        except Exception:
            nodes = None
        try:
            topic_count = len(node.get_topic_names_and_types())
        except Exception:
            topic_count = None
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    results = {}
    for topic in topics:
        entry = state[topic]
        if entry.get('receipts') is None:
            results[topic] = entry
            continue
        results[topic] = _summarize_topic(
            topic,
            entry['type'],
            publishers.get(topic),
            entry['receipts'],
            entry['ages'],
            entry['payload']['bytes'],
        )
    return {
        'schema_version': 1,
        'mode': 'ros',
        'collected_at': datetime.now(timezone.utc).isoformat(),
        'node_name': node_name,
        'duration_s': duration_s,
        'qos': qos,
        'topics': results,
        'graph': {'nodes': nodes, 'topics': topic_count},
        'ros': _ros_environment(),
        'limitations': BENCHMARK_LIMITATIONS,
    }


def _format_number(value, digits=3):
    if value is None:
        return 'n/a'
    return '{:.{digits}f}'.format(value, digits=digits)


def print_host(report):
    """Print a compact host summary."""
    cpu = report['cpu']
    memory = report['memory']
    disk = report['disk']
    ros = report['ros']
    print(
        'HOST name={} arch={} python={} container={}'.format(
            report['hostname'],
            report['architecture'],
            report['python'],
            report['container'],
        )
    )
    print(
        'CPU model={} cores={} util_percent={} load1={}'.format(
            cpu['model'],
            cpu['logical_cores'],
            _format_number(cpu['utilization_percent']),
            _format_number(cpu['load'].get('load_1m')),
        )
    )
    print(
        'MEM total_mb={} available_mb={} used_percent={} '
        'cgroup_limit_mb={}'.format(
            _format_number(memory['total_mb'], 1),
            _format_number(memory['available_mb'], 1),
            _format_number(memory['used_percent']),
            _format_number(memory['cgroup_limit_mb'], 1),
        )
    )
    print(
        'DISK total_gb={} free_gb={} used_percent={}'.format(
            _format_number(disk['total_gb'], 1),
            _format_number(disk['free_gb'], 1),
            _format_number(disk['used_percent']),
        )
    )
    print(
        'TEMP c={} ROS distro={} rmw={} domain={}'.format(
            _format_number(report['temperature_c'], 1),
            ros['distro'],
            ros['rmw'],
            ros['domain_id'],
        )
    )


def print_ros(report):
    """Print a compact ROS summary and return the result code."""
    graph = report['graph']
    print(
        'ROS_GRAPH nodes={} topics={} duration_s={} qos={}'.format(
            graph['nodes'], graph['topics'],
            report['duration_s'], report['qos']
        )
    )
    missing = 0
    for topic in sorted(report['topics']):
        item = report['topics'][topic]
        print(
            'TOPIC {} type={} count={} rate_hz={} interval_mean_ms={} '
            'jitter_ms={} p95_ms={} age_mean_ms={} publishers={}'.format(
                topic,
                item['type'],
                item['count'],
                _format_number(item['rate_hz']),
                _format_number(item['interval']['mean_ms']),
                _format_number(item['interval']['std_ms']),
                _format_number(item['interval']['p95_ms']),
                _format_number(item['header_age']['mean_ms']),
                item['publishers'],
            )
        )
        if item['error']:
            missing += 1
            print('WARN {}: {}'.format(topic, item['error']))
    return missing


def _write_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Read-only host and ROS/DDS benchmark. No command is sent.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--mode', choices=('host', 'ros'), required=True)
    parser.add_argument('--sample-s', type=float, default=1.0)
    parser.add_argument(
        '--topics',
        default='/joint_states',
        help='comma-separated ROS topic names',
    )
    parser.add_argument('--duration-s', type=float, default=10.0)
    parser.add_argument(
        '--qos',
        choices=('sensor_data', 'reliable'),
        default='sensor_data',
    )
    parser.add_argument('--node-name', default='armpi_host_benchmark')
    parser.add_argument('--json-out', default=None)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the command-line entry point."""
    args = _parse_args(argv)
    try:
        if args.mode == 'host':
            report = collect_host(sample_s=args.sample_s)
            if args.json_out:
                _write_json(args.json_out, report)
            print_host(report)
            print('RESULT PASS')
            return 0
        topics = [
            item.strip()
            for item in args.topics.split(',')
            if item.strip()
        ]
        report = measure_ros(
            topics,
            duration_s=args.duration_s,
            qos=args.qos,
            node_name=args.node_name,
        )
        if args.json_out:
            _write_json(args.json_out, report)
        missing = print_ros(report)
        print('RESULT PARTIAL' if missing else 'RESULT PASS')
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
