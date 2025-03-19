#!/usr/bin/env python3
"""
Migrated benchmark.py to ROS2 (no restarts, use taskgen APIs)

Các giả định:
  • Các file cấu hình (YAML) vẫn nằm tại các vị trí như ban đầu.
  • Các API của task_generator (Constants, Namespace, TM_Module, …) có sẵn.
  • Dịch vụ ChangeDirectory (arena_evaluation_msgs.srv.ChangeDirectory) được định nghĩa trong workspace ROS2.
  • Các tham số như "benchmark_resume", "headless" và "default_timeout" được khai báo dưới dạng ROS2 parameters.
"""

import datetime
import hashlib
import json
import pathlib
import typing
import os
import yaml
import subprocess
import logging

import ament_index_python.packages as ament_index
import arena_evaluation_msgs.srv

import rclpy
from rclpy.node import Node

from task_generator.constants import Constants
from task_generator.shared import Namespace, rosparam_get
from task_generator.tasks.modules import TM_Module


def _get_rosmaster_pid() -> int:
    try:
        return int(subprocess.check_output(
            ["ps", "-C", "rosmaster", "-o", "pid", "h"]).decode())
    except Exception as e:
        raise RuntimeError("could not determine rosmaster pid") from e


# --- Cấu trúc cấu hình ---
class _Config(typing.NamedTuple):

    @classmethod
    def parse(cls, obj: typing.Dict):
        print("Loaded config:", obj)
        return cls(
            suite=cls.Suite(**obj["suite"]),
            contest=cls.Contest(**obj["contest"]),
            general=cls.General(**obj["general"])
        )

    class Suite(typing.NamedTuple):
        config: str
        scale_episodes: float = 1

    class Contest(typing.NamedTuple):
        config: str

    class General(typing.NamedTuple):
        simulator: str

    suite: Suite
    contest: Contest
    general: General


class Suite(typing.NamedTuple):

    @classmethod
    def parse(cls, name: str, obj: typing.Dict, default_timeout: float):
        return cls(
            name=name,
            stages=[cls.Stage.parse(stage, default_timeout=default_timeout)
                    for stage in obj["stages"]]
        )

    class Index(int):
        ...

    class Stage(typing.NamedTuple):
        name: str
        episodes: int
        robot: str
        map: str
        tm_robots: Constants.TaskMode.TM_Robots
        tm_obstacles: Constants.TaskMode.TM_Obstacles
        config: typing.Dict
        seed: int
        timeout: float

        @classmethod
        def hash(cls, obj: typing.Dict) -> int:
            return 0x7f_ff_ff_ff & int.from_bytes(
                hashlib.sha1(json.dumps(obj).encode()).digest()[-4:],
                byteorder="big"
            )

        @classmethod
        def parse(cls, obj: typing.Dict, default_timeout: float) -> "Suite.Stage":
            obj.setdefault("timeout", default_timeout)
            obj.setdefault("seed", cls.hash(obj))
            return cls(**obj)

    name: str
    stages: typing.List[Stage]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> "Suite.Index":
        return self.Index(len(self.stages) - 1)

    def config(self, index: "Suite.Index") -> "Suite.Stage":
        return self.stages[index]


class Contest(typing.NamedTuple):

    @classmethod
    def parse(cls, name: str, obj: typing.Dict):
        return cls(
            name=name,
            contestants=[cls.Contestant.parse(contestant)
                         for contestant in obj["contestants"]]
        )

    class Index(int):
        ...

    class Contestant(typing.NamedTuple):
        name: str
        local_planner: str
        inter_planner: str

        @classmethod
        def parse(cls, obj: typing.Dict) -> "Contest.Contestant":
            obj.setdefault("inter_planner", "bypass")
            return cls(**obj)

    name: str
    contestants: typing.List[Contestant]

    @property
    def min_index(self):
        return self.Index()

    @property
    def max_index(self) -> "Contest.Index":
        return self.Index(len(self.contestants) - 1)

    def config(self, index: "Contest.Index") -> "Contest.Contestant":
        return self.contestants[index]


# --- Node Benchmark ROS2 ---
class Mod_Benchmark(TM_Module, Node):

    # Thiết lập các đường dẫn cấu hình theo thư mục share của package arena_bringup
    DIR: Namespace = Namespace(
        os.path.join(
            ament_index.get_package_share_directory("arena_bringup"),
            "configs",
            "benchmark"
        )
    )
    LOCK_FILE = "resume.lock"
    LOG_DIR = DIR("logs")
    TASK_GENERATOR_CONFIG = os.path.join(
        ament_index.get_package_share_directory("arena_bringup"),
        "configs",
        "task_generator.yaml"
    )
    TASK_GENERATOR_CONFIG_BKUP = TASK_GENERATOR_CONFIG + ".bkup"

    _config: _Config
    _suite: Suite
    _contest: Contest
    _episode_index: int

    _runid: str
    _contest_index: Contest.Index
    _suite_index: Suite.Index
    _headless: int

    _requires_restart: bool

    # --- PHƯƠNG THỨC CẤU HÌNH ---
    @classmethod
    def _load_config(cls) -> _Config:
        try:
            with open(cls.DIR("config.yaml")) as f:
                config = yaml.load(f, yaml.FullLoader)
            return _Config.parse(config)
        except Exception as e:
            raise RuntimeError(f"Error loading config.yaml: {e}")

    @classmethod
    def _load_contest(cls, contest: str) -> Contest:
        try:
            with open(cls.DIR("contests", contest)) as f:
                data = yaml.load(f, yaml.FullLoader)
            return Contest.parse(pathlib.Path(contest).stem, data)
        except Exception as e:
            raise RuntimeError(f"Error loading contest file {contest}: {e}")

    @classmethod
    def _load_suite(cls, suite: str, default_timeout: float) -> Suite:
        try:
            with open(cls.DIR("suites", suite)) as f:
                data = yaml.load(f, yaml.FullLoader)
            return Suite.parse(pathlib.Path(suite).stem, data, default_timeout=default_timeout)
        except Exception as e:
            raise RuntimeError(f"Error loading suite file {suite}: {e}")

    @classmethod
    def _resume(cls) -> typing.Tuple[str, Contest.Index, Suite.Index, int]:
        try:
            with open(cls.DIR(cls.LOCK_FILE)) as f:
                runid, contest, suite, headless = f.read().split(" ")
            return runid, Contest.Index(int(contest)), Suite.Index(int(suite)), int(headless)
        except Exception as e:
            raise RuntimeError(f"Error resuming from lock file: {e}")

    @classmethod
    def _taskgen_backup(cls):
        if os.path.exists(cls.TASK_GENERATOR_CONFIG_BKUP):
            return
        try:
            with open(cls.TASK_GENERATOR_CONFIG) as fr, open(cls.TASK_GENERATOR_CONFIG_BKUP, "w") as fw:
                fw.write(fr.read())
        except Exception as e:
            raise RuntimeError(f"Error backing up task_generator config: {e}")

    @classmethod
    def _taskgen_write(cls, *configs: typing.Dict):
        def overwrite(source: typing.Dict, target: typing.Dict):
            for k, v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        try:
            with open(cls.TASK_GENERATOR_CONFIG, "r") as f:
                joint_config = yaml.load(f, yaml.FullLoader)
            for config in configs:
                overwrite(config, joint_config)
            with open(cls.TASK_GENERATOR_CONFIG, "w") as f:
                yaml.dump(joint_config, f)
        except Exception as e:
            raise RuntimeError(f"Error writing task_generator config: {e}")

    @classmethod
    def _taskgen_restore(cls, cleanup: bool = False):
        try:
            with open(cls.TASK_GENERATOR_CONFIG_BKUP) as fr, open(cls.TASK_GENERATOR_CONFIG, "w") as fw:
                fw.write(fr.read())
            if cleanup:
                os.remove(cls.TASK_GENERATOR_CONFIG_BKUP)
        except Exception as e:
            raise RuntimeError(f"Error restoring task_generator config: {e}")

    # --- PHẦN RUNTIME ---
    def __init__(self, **kwargs):
        Node.__init__(self, 'benchmark_node')
        TM_Module.__init__(self, **kwargs)

        # Khai báo các ROS2 parameters
        self.declare_parameter("benchmark_resume", False)
        self.declare_parameter("headless", 1)
        self.declare_parameter("default_timeout", 60.0)

        # Log giá trị tham số để debug
        try:
            default_timeout = self.get_parameter("default_timeout").value
            self.get_logger().info(f"default_timeout = {default_timeout}")
        except Exception as e:
            self.get_logger().error(f"Error getting default_timeout parameter: {e}")
            raise

        # Load file cấu hình
        try:
            self._config = self._load_config()
        except Exception as e:
            self.get_logger().error(str(e))
            raise

        try:
            self._suite = self._load_suite(self._config.suite.config, default_timeout=default_timeout)
            self._contest = self._load_contest(self._config.contest.config)
        except Exception as e:
            self.get_logger().error(str(e))
            raise

        self._requires_restart = False

        if not rosparam_get(bool, "benchmark_resume", False):
            self._taskgen_backup()
            self._runid = f"{self._contest.name}_{datetime.datetime.now().strftime('%y-%m-%d_%H-%M-%S')}"
            self._contest_index = self._contest.min_index
            self._suite_index = self._suite.min_index
            self._headless = rosparam_get(int, "headless", 1)

            self._taskgen_backup()
            try:
                with open(self.TASK_GENERATOR_CONFIG_BKUP) as f:
                    base_config = f.read()
            except Exception as e:
                self.get_logger().error(f"Error reading backup config: {e}")
                raise

            try:
                os.makedirs(self.LOG_DIR, exist_ok=True)
            except Exception as e:
                self.get_logger().error(f"Error creating log directory: {e}")
                raise

            try:
                with open(self.LOG_DIR(f"{self._runid}.log"), "w") as f:
                    f.write(f"run {self._runid}\n")
                    f.write(f"of contest {self._contest.name} with {len(self._contest.contestants)} contestants\n")
                    f.write(f"on suite {self._suite.name} with {len(self._suite.stages)} stages\n")
                    total_eps = len(self._contest.contestants) * sum(
                        [int(self._config.suite.scale_episodes * self._suite.config(Suite.Index(index)).episodes)
                         for index in range(self._suite.min_index, self._suite.max_index + 1)]
                    )
                    f.write(f"total of {total_eps} episodes\n")
                    f.write("\n")
                    f.write(f"Simulator: {self._config.general.simulator}\n")
                    f.write(f"Base Config: {json.dumps(base_config)}\n")
                    f.write("=" * 80 + "\n")
                    f.write("\n")
            except Exception as e:
                self.get_logger().error(f"Error writing log file: {e}")
                raise

            self._log_contest()
            self._log_suite()

            self._requires_restart = True
            self._reincarnate()
        else:
            try:
                self._runid, contest_index, suite_index, headless = self._resume()
                self._contest_index, self._suite_index, self._headless = contest_index, suite_index, headless
                self._episode_index = -1
            except Exception as e:
                self.get_logger().error(f"Error resuming state: {e}")
                raise

        # Tạo timer xử lý mỗi episode: tăng giá trị _episode mỗi giây
        self.create_timer(1.0, lambda: self._episode.__set__(self, self._episode + 1))

    def before_reset(self):
        pass

    def after_reset(self):
        self._episode += 1

    _logger_object: logging.Logger

    @property
    def custom_logger(self) -> logging.Logger:
        if not hasattr(self, "_logger_object"):
            handler = logging.FileHandler(self.LOG_DIR(f"{self._runid}.log"))
            handler.setFormatter(logging.Formatter('%(created)f: %(message)s'))
            logger = logging.getLogger("benchmark")
            logger.setLevel(logging.DEBUG)
            logger.addHandler(handler)
            self._logger_object = logger
        return self._logger_object

    def _log_contest(self):
        self.custom_logger.info(f"\tC [{1 + self.contest_index:0>{len(str(1 + self._contest.max_index))}}/{1 + self._contest.max_index}] {self._contest.config(self._contest_index).name}")

    def _log_suite(self):
        self.custom_logger.info(f"\t\tS [{1 + self.suite_index:0>{len(str(1 + self._suite.max_index))}}/{1 + self._suite.max_index}] {self._suite.config(self._suite_index).name}")

    def _log_episode(self):
        if self._episode < 0:
            return
        episode_limit = int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes)
        self.custom_logger.info(f"\t\t\tE [{1 + self._episode:0>{len(str(episode_limit))}}/{episode_limit}]")

    @property
    def contest_index(self) -> Contest.Index:
        return self._contest_index

    @contest_index.setter
    def contest_index(self, index: int):
        self._contest_index = Contest.Index(index)
        if self._contest_index > self._contest.max_index:
            self.custom_logger.info("BENCHMARK COMPLETED SUCCESSFULLY")
            os.remove(self.DIR(self.LOCK_FILE))
            self._taskgen_restore(cleanup=True)
            self._suicide()
        else:
            self._log_contest()
            self.custom_logger.debug("contestant change requires restart")
            self._requires_restart = True
            self._reincarnate()

    @property
    def suite_index(self) -> Suite.Index:
        return self._suite_index

    @suite_index.setter
    def suite_index(self, index: int):
        old_config = self._suite.config(self._suite_index)
        self._suite_index = Suite.Index(index)
        if self._suite_index > self._suite.max_index:
            self._suite_index = self._suite.min_index
            self.contest_index += 1
        else:
            self._log_suite()
            new_config = self._suite.config(self._suite_index)
            if new_config.map != old_config.map:
                self.custom_logger.debug("map change requires restart")
                self._requires_restart = True
            if new_config.robot != old_config.robot:
                self.custom_logger.debug("robot change requires restart")
                self._requires_restart = True
            self._reincarnate()

    @property
    def _episode(self) -> int:
        return self._episode_index

    @_episode.setter
    def _episode(self, episode: int):
        if episode >= int(self._suite.config(self._suite_index).episodes * self._config.suite.scale_episodes):
            self._episode_index = 0
            self.suite_index += 1
        else:
            self._episode_index = episode
            self._log_episode()

    def _reincarnate(self):
        try:
            with open(self.DIR(self.LOCK_FILE), "w") as f:
                f.write(f"{self._runid} {self._contest_index} {self._suite_index} {self._headless}")
        except Exception as e:
            self.get_logger().error(f"Error writing lock file: {e}")
            raise
        config = self._config
        contest_config = self._contest.config(self._contest_index)
        suite_config = self._suite.config(self._suite_index)
        # self._taskgen_restore()
        # self._taskgen_write(
        #     {"episodes": -1,
        #      "RANDOM": {
        #          "seed": suite_config.seed ^ Suite.Stage.hash({"": self._runid})
        #      }
        #      },
        #     suite_config.config
        # )
        record_data_dir = f"{self._runid}/{contest_config.name}/{suite_config.name}"

        # Sử dụng ROS2 service client để gọi ChangeDirectory service
        client = self.create_client(arena_evaluation_msgs.srv.ChangeDirectory, f"/{suite_config.robot}/change_directory")
        if not client.wait_for_service(timeout_sec=10.0):
            self.custom_logger.error("ChangeDirectory service not available after 10 seconds")
        else:
            req = arena_evaluation_msgs.srv.ChangeDirectory.Request()
            req.data = record_data_dir
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                self.custom_logger.info(f"ChangeDirectory service call succeeded: {future.result().result}")
            else:
                self.custom_logger.error("ChangeDirectory service call failed")
        self._episode = 0

    def _taskgen_write(self, *configs: typing.Dict):
        def overwrite(source: typing.Dict, target: typing.Dict):
            for k, v in source.items():
                if isinstance(v, dict):
                    target.setdefault(k, dict())
                    overwrite(v, target[k])
                else:
                    target[k] = v
            return target

        with open(self.TASK_GENERATOR_CONFIG, "r") as f:
            joint_config = yaml.load(f, yaml.FullLoader)

        for config in configs:
            overwrite(config, joint_config)

        with open(self.TASK_GENERATOR_CONFIG, "w") as f:
            yaml.dump(joint_config, f)

    def _taskgen_restore(self, cleanup: bool = False):
        with open(self.TASK_GENERATOR_CONFIG_BKUP) as fr, open(self.TASK_GENERATOR_CONFIG, "w") as fw:
            fw.write(fr.read())
        if cleanup:
            os.remove(self.TASK_GENERATOR_CONFIG_BKUP)

    def _suicide(self):
        self.custom_logger.info("Shutting down node (suicide).")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Mod_Benchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.custom_logger.info("Benchmark node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
