#!/usr/bin/env python3
"""Controller switching utilities for UR10e servo control.

Provides helper functions to switch between ros2_control controllers
(e.g. joint_trajectory_controller <-> forward_position_controller).
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import SwitchController, ListControllers

from robot.config import (
    FORWARD_POSITION_CONTROLLER,
    FORWARD_VELOCITY_CONTROLLER,
    JOINT_NAMES,
    SCALED_TRAJECTORY_CONTROLLER,
    TRAJECTORY_CONTROLLER,
)


class ControllerSwitcher:
    """Handles controller switching via controller_manager services."""

    def __init__(self, node: Node):
        self.node = node
        self._switch_client = node.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        self._list_client = node.create_client(
            ListControllers,
            '/controller_manager/list_controllers'
        )
        self._original_active: list[str] | None = None

    def wait_for_services(self, timeout_sec: float = 10.0) -> bool:
        """Wait for controller_manager services to be available."""
        self.node.get_logger().info('Waiting for controller_manager services...')
        if not self._switch_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('switch_controller service not available')
            return False
        if not self._list_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().error('list_controllers service not available')
            return False
        self.node.get_logger().info('controller_manager services ready.')
        return True

    def list_controllers(self) -> list:
        """Get list of all controllers and their states."""
        req = ListControllers.Request()
        future = self._list_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if future.result() is None:
            self.node.get_logger().error('Failed to list controllers')
            return []
        return future.result().controller

    def get_active_controllers(self) -> list[str]:
        """Get names of currently active controllers."""
        controllers = self.list_controllers()
        return [c.name for c in controllers if c.state == 'active']

    def switch_controller(
        self,
        start: list[str],
        stop: list[str],
        strictness: int = 2  # STRICT
    ) -> bool:
        """Switch controllers: activate `start` and deactivate `stop`.

        Args:
            start: Controllers to activate.
            stop: Controllers to deactivate.
            strictness: 1=BEST_EFFORT, 2=STRICT.

        Returns:
            True if switch succeeded.
        """
        req = SwitchController.Request()
        req.activate_controllers = start
        req.deactivate_controllers = stop
        req.strictness = strictness

        self.node.get_logger().info(
            f'Switching controllers: start={start}, stop={stop}'
        )

        future = self._switch_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)

        if future.result() is None:
            self.node.get_logger().error('Switch controller service call failed')
            return False

        ok = future.result().ok
        if ok:
            self.node.get_logger().info('Controller switch successful.')
        else:
            self.node.get_logger().error('Controller switch FAILED.')
        return ok

    def activate_forward_position(self) -> bool:
        """Activate forward_position_controller, deactivating trajectory controllers.

        Saves the currently active trajectory controller for later restoration.
        """
        active = self.get_active_controllers()

        # Determine which trajectory controller to stop
        stop_controllers = []
        for name in [TRAJECTORY_CONTROLLER, SCALED_TRAJECTORY_CONTROLLER]:
            if name in active:
                stop_controllers.append(name)

        if FORWARD_POSITION_CONTROLLER in active:
            self.node.get_logger().info(
                f'{FORWARD_POSITION_CONTROLLER} is already active.'
            )
            return True

        # Save original state for restoration
        self._original_active = stop_controllers.copy()

        return self.switch_controller(
            start=[FORWARD_POSITION_CONTROLLER],
            stop=stop_controllers
        )

    def restore_original(self) -> bool:
        """Restore the original trajectory controller that was active before switching."""
        if not self._original_active:
            self.node.get_logger().warn(
                'No original controller to restore. Using joint_trajectory_controller.'
            )
            self._original_active = [TRAJECTORY_CONTROLLER]

        active = self.get_active_controllers()

        stop_controllers = []
        if FORWARD_POSITION_CONTROLLER in active:
            stop_controllers.append(FORWARD_POSITION_CONTROLLER)
        if FORWARD_VELOCITY_CONTROLLER in active:
            stop_controllers.append(FORWARD_VELOCITY_CONTROLLER)

        if not stop_controllers:
            self.node.get_logger().info('No forward controller to stop.')
            return True

        return self.switch_controller(
            start=self._original_active,
            stop=stop_controllers
        )

    def print_status(self):
        """Print current controller states."""
        controllers = self.list_controllers()
        self.node.get_logger().info('--- Controller Status ---')
        for c in controllers:
            if c.state == 'active':
                self.node.get_logger().info(f'  [ACTIVE]   {c.name} ({c.type})')
            elif c.state == 'inactive':
                self.node.get_logger().info(f'  [inactive] {c.name} ({c.type})')
