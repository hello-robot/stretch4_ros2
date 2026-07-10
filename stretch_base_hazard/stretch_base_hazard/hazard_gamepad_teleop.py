"""Gamepad teleop entry point guarded by the base hazard map."""

from __future__ import annotations

import argparse
import sys
import threading
from typing import Any

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from stretch_base_hazard.hazard_point_cache import (
    declare_hazard_filter_parameters,
    hazard_point_cache_config_from_params,
    HazardPointCache,
)


class HazardFilteredBase:
    """Proxy that filters base velocity commands before forwarding them."""

    def __init__(self, base: Any, node: Node, cache: HazardPointCache):
        object.__setattr__(self, '_base', base)
        object.__setattr__(self, '_node', node)
        object.__setattr__(self, '_cache', cache)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._base, name)

    def __setattr__(self, name: str, value: Any) -> None:
        if name in {'_base', '_node', '_cache'}:
            object.__setattr__(self, name, value)
            return
        setattr(self._base, name, value)

    def set_velocity(
        self,
        vx_m: float,
        vy_m: float,
        w_r: float,
        a_m: float | None = None,
        a_r: float | None = None,
        *args: Any,
        **kwargs: Any,
    ) -> Any:
        decision = self._cache.filter_velocity(vx_m, vy_m, w_r)
        result = decision.result
        self._log_decision(vx_m, vy_m, w_r, decision.stale, result)
        return self._base.set_velocity(
            result.vx,
            result.vy,
            result.wz,
            a_m,
            a_r,
            *args,
            **kwargs,
        )

    def _log_decision(
        self,
        vx_m: float,
        vy_m: float,
        w_r: float,
        stale: bool,
        result: Any,
    ) -> None:
        cfg = self._cache.config.filter_config
        moving_linear = (
            abs(vx_m) > cfg.min_linear_speed_mps
            or abs(vy_m) > cfg.min_linear_speed_mps
        )
        if stale and moving_linear:
            self._node.get_logger().warning(
                'Blocking gamepad linear velocity because hazard point clouds '
                'are stale or missing',
                throttle_duration_sec=1.0,
            )
            return
        if result.blocked_by_obstacle or result.blocked_by_cliff:
            self._node.get_logger().warning(
                'Blocking gamepad linear velocity '
                f'vx={vx_m:.3f} vy={vy_m:.3f} wz={w_r:.3f}; '
                f'obstacle_cells={result.blocking_obstacle_count} '
                f'cliff_cells={result.blocking_cliff_count}',
                throttle_duration_sec=0.5,
            )
        elif result.slowed_by_obstacle:
            self._node.get_logger().warning(
                'Slowing gamepad linear velocity for tight clearance '
                f'vx={vx_m:.3f}->{result.vx:.3f} '
                f'vy={vy_m:.3f}->{result.vy:.3f} '
                f'scale={result.obstacle_speed_scale:.2f} '
                f'soft_obstacle_cells={result.soft_obstacle_count}',
                throttle_duration_sec=0.5,
            )


def main(args: list[str] | None = None) -> None:
    raw_args = list(sys.argv if args is None else ['hazard_gamepad_teleop', *args])
    cli_args = _parse_cli(remove_ros_args(args=raw_args)[1:])

    rclpy.init(args=args)
    node = Node('hazard_gamepad_teleop')
    declare_hazard_filter_parameters(node)
    cache = HazardPointCache(node, hazard_point_cache_config_from_params(node))
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    gamepad_teleop = None
    try:
        from stretch4_body.core.gamepad_control_mappings import ControlMapping
        from stretch4_body.core.gamepad_teleop import GamePadTeleop
        from stretch4_body.core.hello_utils import print_stretch_re_use
        from stretch4_body.tools.stretch_gamepad_teleop import (
            check_gamepad_teleop_singleton,
        )

        print_stretch_re_use()
        if not cli_args.no_singleton_lock and not check_gamepad_teleop_singleton():
            print('Gamepad teleop is already running!')
            return

        for mapping in ControlMapping._get_cycleable_options():
            print(mapping.description())

        gamepad_teleop = GamePadTeleop(use_server=not cli_args.direct)
        gamepad_teleop.startup()
        _install_base_filter(
            gamepad_teleop.robot,
            HazardFilteredBase(gamepad_teleop.robot.base, node, cache),
        )
        node.get_logger().info('Started hazard-aware gamepad teleop')
        gamepad_teleop.mainloop()
    except Exception:
        if gamepad_teleop is not None:
            gamepad_teleop.stop()
        raise
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


def _parse_cli(args: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Control Stretch from a gamepad while blocking hazardous base motion.',
    )
    parser.add_argument(
        '-d',
        '--direct',
        help='Use direct stretch4_body API instead of RobotClient/server',
        action='store_true',
    )
    parser.add_argument(
        '--no-singleton-lock',
        help='Do not take the stretch_gamepad_teleop singleton lock',
        action='store_true',
    )
    return parser.parse_args(args)


def _install_base_filter(robot: Any, filtered_base: HazardFilteredBase) -> None:
    original_base = robot.base
    robot.base = filtered_base
    if hasattr(robot, 'omnibase') and robot.omnibase is original_base:
        robot.omnibase = filtered_base


if __name__ == '__main__':
    main()
