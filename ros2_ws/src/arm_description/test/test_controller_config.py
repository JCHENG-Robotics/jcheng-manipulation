"""Offline regression coverage for controller profiles and launch event wiring."""

import importlib.util
import shutil
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import yaml

from arm_description.controller_config import ControllerConfigError, resolve_controller_config


class ControllerConfigTest(unittest.TestCase):
    def setUp(self):
        self.package = Path(__file__).resolve().parents[1]

    def copy_share(self):
        directory = tempfile.TemporaryDirectory()
        copied = Path(directory.name) / "share"
        shutil.copytree(self.package / "config", copied / "config")
        self.addCleanup(directory.cleanup)
        return copied

    def test_approved_profiles_have_exact_gain_scope_and_hashes(self):
        low = resolve_controller_config(str(self.package), "gain_0p1")
        high = resolve_controller_config(str(self.package), "gain_0p2")
        self.assertEqual(low.sha256, "708408e9d77dec8aacc6a657ccc2adaca932731e320b0aaf5a70355676d567a3")
        self.assertEqual(high.sha256, "5ceea231197035fc41a08fce0dc3e9ccab2a77b00efd4620adfbfbf358abe0cd")
        low_bytes = low.realpath.read_bytes()
        self.assertEqual(low_bytes.count(b"    position_proportional_gain: 0.1\n"), 1)
        self.assertEqual(low_bytes.replace(b"    position_proportional_gain: 0.1\n", b"    position_proportional_gain: 0.2\n"), high.realpath.read_bytes())
        low_data = yaml.safe_load(low_bytes)
        high_data = yaml.safe_load(high.realpath.read_bytes())
        self.assertEqual(type(low_data["/gz_ros_control"]["ros__parameters"]["position_proportional_gain"]), float)
        self.assertEqual(low_data["/gz_ros_control"]["ros__parameters"]["position_proportional_gain"], 0.1)
        self.assertEqual(high_data["/gz_ros_control"]["ros__parameters"]["position_proportional_gain"], 0.2)
        del low_data["/gz_ros_control"]["ros__parameters"]["position_proportional_gain"]
        del high_data["/gz_ros_control"]["ros__parameters"]["position_proportional_gain"]
        self.assertEqual(low_data, high_data)

    def test_default_remains_compatible_and_v2_schema_is_strict(self):
        default = resolve_controller_config(str(self.package), "default")
        self.assertEqual(default.realpath.name, "arm_controllers.yaml")
        copied = self.copy_share()
        cases = {
            "integer_gain": lambda data: data["/gz_ros_control"]["ros__parameters"].__setitem__("position_proportional_gain", 1),
            "boolean_gain": lambda data: data["/gz_ros_control"]["ros__parameters"].__setitem__("position_proportional_gain", True),
            "wrong_namespace": lambda data: data.__setitem__("gz_ros_control", data.pop("/gz_ros_control")),
            "extra_leaf": lambda data: data["arm_controller"]["ros__parameters"].__setitem__("unexpected", False),
            "wrong_boolean": lambda data: data["arm_controller"]["ros__parameters"].__setitem__("open_loop_control", True),
        }
        for label, mutation in cases.items():
            with self.subTest(label=label):
                path = copied / "config" / f"{label}.yaml"
                data = yaml.safe_load((copied / "config" / "arm_controllers_gain_0p1.yaml").read_text(encoding="utf-8"))
                mutation(data)
                path.write_text(yaml.safe_dump(data), encoding="utf-8")
                with self.assertRaises(ControllerConfigError):
                    resolve_controller_config(str(copied), "gain_0p1", str(path))

    def test_yaml_parser_rejects_duplicate_multidoc_and_features(self):
        copied = self.copy_share()
        cases = {
            "duplicate.yaml": "controller_manager: {}\ncontroller_manager: {}\n",
            "multiple.yaml": "---\n{}\n---\n{}\n",
            "anchor.yaml": "controller_manager: &same {}\narm_controller: *same\n",
            "tag.yaml": "!unsafe {}\n",
        }
        for name, body in cases.items():
            path = copied / "config" / name
            path.write_text(body, encoding="utf-8")
            with self.subTest(name=name), self.assertRaises(ControllerConfigError):
                resolve_controller_config(str(copied), "default", str(path))

    def test_paths_are_share_relative_and_expand_user(self):
        copied = self.copy_share()
        relative = resolve_controller_config(str(copied), "gain_0p1", "arm_controllers_gain_0p1.yaml")
        self.assertEqual(relative.realpath, (copied / "config" / "arm_controllers_gain_0p1.yaml").resolve())
        unicode_name = "配置 空格.yaml"
        shutil.copy(copied / "config" / "arm_controllers_gain_0p1.yaml", copied / "config" / unicode_name)
        self.assertEqual(resolve_controller_config(str(copied), "gain_0p1", unicode_name).realpath.name, unicode_name)
        home_config = Path.home() / "controller-config-test.yaml"
        self.addCleanup(lambda: home_config.unlink(missing_ok=True))
        shutil.copy(copied / "config" / "arm_controllers_gain_0p1.yaml", home_config)
        self.assertEqual(resolve_controller_config(str(copied), "gain_0p1", "~/controller-config-test.yaml").realpath, home_config.resolve())
        for raw in (" bad.yaml", "bad.yaml ", "bad\n.yaml", "$HOME/x.yaml", "missing.yaml"):
            with self.subTest(raw=raw), self.assertRaises(ControllerConfigError):
                resolve_controller_config(str(copied), "gain_0p1", raw)


class LaunchWiringTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        launch_path = Path(__file__).resolve().parents[1] / "launch" / "gazebo.launch.py"
        spec = importlib.util.spec_from_file_location("gate2_gazebo_launch", launch_path)
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def test_success_starts_next_stage_and_its_own_watchdog(self):
        next_action = object()
        context = SimpleNamespace(launch_configurations={})
        result = self.module._after_success("spawn", ("jsb", next_action))(SimpleNamespace(returncode=0, process_name="spawn"), context)
        self.assertEqual(context.launch_configurations["gate2_spawn_completed"], "true")
        self.assertIs(result[0], next_action)
        self.assertEqual(result[1].period, 60.0)

    def test_nonzero_does_not_start_next_stage(self):
        context = SimpleNamespace(launch_configurations={})
        result = self.module._after_success("jsb", ("arm", object()))(SimpleNamespace(returncode=3, process_name="jsb"), context)
        self.assertEqual(len(result), 2)
        self.assertNotIn("gate2_jsb_completed", context.launch_configurations)

    def test_completed_stage_watchdog_is_inert(self):
        timer = self.module._watchdog("arm")
        callback = timer.actions[0].execute
        context = SimpleNamespace(launch_configurations={"gate2_arm_completed": "true"})
        self.assertEqual(callback(context), [])

    def test_gazebo_include_requests_shutdown_on_exit(self):
        source = (Path(__file__).resolve().parents[1] / "launch" / "gazebo.launch.py").read_text(encoding="utf-8")
        self.assertIn('"on_exit_shutdown": "true"', source)


if __name__ == "__main__":
    unittest.main()
