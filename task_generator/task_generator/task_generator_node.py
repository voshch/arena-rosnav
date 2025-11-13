#! /usr/bin/env python3
import asyncio
import multiprocessing
import queue
import traceback
import typing

import launch
import rclpy
import rclpy.executors
import os
import signal
import sys
import threading
import time


def init_launch_service(
    CONCURRENT: bool
) -> tuple[
    typing.Callable[[], typing.Any],
    typing.Callable[[launch.LaunchDescription], typing.Any],
    typing.Callable[[], None]
]:
    """
    Initiate launch service.
    Args:
        CONCURRENT: is node run concurrently in python thread?
    Returns:
        A tuple consisting of:
            loop function to call after `executor.spin_once()`,
            function that accepts a `launch.LaunchDescription` and schedules its launch.
            cleanup function to call at shutdown
    """

    def _do_launch(
            launch_description: launch.LaunchDescription
    ) -> typing.Callable[[], typing.Any]:

        # https://github.com/ros2/launch/issues/724#issue-1851039469
        def run_process(stop_event, launch_description):

            # https://github.com/ros2/launch/issues/724#issuecomment-1829050299
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

            launch_service = launch.launch_service.LaunchService()
            launch_service.include_launch_description(launch_description)
            launch_task = loop.create_task(launch_service.run_async())

            try:
                loop.run_until_complete(
                    loop.run_in_executor(
                        None,
                        stop_event.wait
                    )
                )
            except KeyboardInterrupt:
                stop_event.set()

            if not launch_task.done():
                asyncio.ensure_future(launch_service.shutdown(), loop=loop)
                try:
                    loop.run_until_complete(launch_task)
                except KeyboardInterrupt:
                    stop_event.set()

        stop_event = multiprocessing.Event()
        process = multiprocessing.Process(
            target=run_process, args=(
                stop_event,
                launch_description
            ),
            daemon=True
        )
        process.start()

        def shutdown():
            stop_event.set()

        return shutdown

    if CONCURRENT:
        launch_queue = queue.Queue[launch.LaunchDescription]()
        shutdown_queue = queue.Queue[typing.Callable[[], typing.Any]]()

        def process_queue() -> None:
            try:
                launch_description = launch_queue.get(False)
            except queue.Empty:
                return None

            shutdown_queue.put(_do_launch(launch_description))
            return process_queue()

        def launch_soon(launch_description: launch.LaunchDescription) -> None:
            launch_queue.put(launch_description)

        def cleanup():
            while not shutdown_queue.empty():
                do_shutdown = shutdown_queue.get(True)
                do_shutdown()

        return process_queue, launch_soon, cleanup

    def _noop() -> None:
        return None

    return _noop, _do_launch, _noop


def main(args=None):
    rclpy.init()

    CONCURRENT = True

    if CONCURRENT:
        executor = rclpy.executors.MultiThreadedExecutor()
    else:
        executor = rclpy.executors.SingleThreadedExecutor()

    launch_loop, do_launch, launch_cleanup = init_launch_service(
        CONCURRENT=CONCURRENT
    )

    from . import NodeInterface

    node = NodeInterface.init_task_gen_node(do_launch=do_launch)

    shutdown_requested = {"flag": False}

    def _shutdown():
        if shutdown_requested["flag"]:
            return
        shutdown_requested["flag"] = True
        try:
            node.get_logger().info("Shutting down task_generator (graceful)...")
        except Exception:
            pass
        try:
            launch_cleanup()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    def _signal_handler(signum, frame):
        _shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    sim_lock_path = os.environ.get('ARENA_SIM_LOCK', '/tmp/arena_sim.lock')

    def _is_pid_alive(pid: int) -> bool:
        try:
            os.kill(pid, 0)
        except Exception:
            return False
        return True

    # track whether we've seen a valid simulator lock; used by the watcher
    seen_lock = {"val": False}

    if os.path.exists(sim_lock_path):
        try:
            # read pid from lock file and validate
            valid_lock = False
            try:
                with open(sim_lock_path, 'r') as f:
                    data = f.read().strip().split()
                    if data:
                        pid = int(data[0])
                        if _is_pid_alive(pid):
                            valid_lock = True
                        else:
                            # stale lock, remove it
                            try:
                                os.remove(sim_lock_path)
                            except Exception:
                                pass
            except Exception:
                # if file malformed or unreadable, try to remove it
                try:
                    os.remove(sim_lock_path)
                except Exception:
                    pass

            if valid_lock:
                try:
                    node.get_logger().info(f"Simulator lock found ({sim_lock_path}), performing startup sync/reset...")
                except Exception:
                    pass

                def _delayed_reset():
                    try:
                        time.sleep(1.0)
                        try:
                            node.reset_task(first_map=True)
                        except Exception as e:
                            try:
                                node.get_logger().warn(f"Startup reset failed: {e}")
                            except Exception:
                                pass
                    except Exception:
                        pass

                threading.Thread(target=_delayed_reset, daemon=True).start()
                # mark that we've seen a valid lock at startup
                try:
                    seen_lock["val"] = True
                except Exception:
                    pass
        except Exception:
            # ignore failures around checking the lock
            pass

    executor.add_node(node)

    # Background watcher: poll for the simulator lock file appearing/disappearing.
    def _sim_watcher():
        poll_interval = 1.0
        while rclpy.ok() and not shutdown_requested["flag"]:
            try:
                if os.path.exists(sim_lock_path):
                    # validate pid inside lock
                    try:
                        with open(sim_lock_path, 'r') as f:
                            data = f.read().strip().split()
                            if data:
                                pid = int(data[0])
                                if _is_pid_alive(pid):
                                    if not seen_lock["val"]:
                                        # newly appeared valid lock -> perform reset
                                        try:
                                            node.get_logger().info(
                                                f"Simulator lock appeared ({sim_lock_path}), performing sync/reset..."
                                            )
                                        except Exception:
                                            pass

                                        def _delayed_reset_on_appear():
                                            try:
                                                time.sleep(1.0)
                                                try:
                                                    node.reset_task(first_map=True)
                                                except Exception as e:
                                                    try:
                                                        node.get_logger().warn(f"Startup reset failed: {e}")
                                                    except Exception:
                                                        pass
                                            except Exception:
                                                pass

                                        threading.Thread(target=_delayed_reset_on_appear, daemon=True).start()
                                        seen_lock["val"] = True
                                else:
                                    # stale lock, remove it and mark unseen
                                    try:
                                        os.remove(sim_lock_path)
                                    except Exception:
                                        pass
                                    seen_lock["val"] = False
                    except Exception:
                        # malformed/unreadable -> try remove
                        try:
                            os.remove(sim_lock_path)
                        except Exception:
                            pass
                        seen_lock["val"] = False
                else:
                    # no lock present
                    if seen_lock["val"]:
                        # mark unseen so future appearances trigger reset
                        seen_lock["val"] = False
                time.sleep(poll_interval)
            except Exception:
                # swallow errors in watcher to avoid crashing main thread
                try:
                    time.sleep(poll_interval)
                except Exception:
                    pass

    threading.Thread(target=_sim_watcher, daemon=True).start()

    try:
        node.get_logger().info('Beginning client, shut down with CTRL-C')
        while rclpy.ok() and not shutdown_requested["flag"]:
            executor.spin_once(timeout_sec=0.1)
            try:
                launch_loop()
            except Exception:
                pass
    except Exception as e:
        try:
            node.get_logger().error(f"Unhandled exception in main loop: {e}")
            node.get_logger().debug(traceback.format_exc())
        except Exception:
            pass
    finally:
        _shutdown()
