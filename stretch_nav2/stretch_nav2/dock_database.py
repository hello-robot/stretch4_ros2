#!/usr/bin/env python3

import os
import yaml
from collections import UserDict
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.task import Future
from geometry_msgs.msg import PoseStamped


def pose_dict_to_pose_stamped(pose_dict, frame_id='map', stamp=None) -> PoseStamped:
    """
    Convert a pose dictionary (containing position [x,y,z] and orientation [qx,qy,qz,qw])
    into a geometry_msgs/msg/PoseStamped message compatible with Nav2 Simple Commander.

    Args:
        pose_dict (dict): Dictionary with 'position' and 'orientation' keys.
        frame_id (str): Reference frame name, defaults to 'map'.
        stamp: Optional ROS 2 builtin_interfaces/msg/Time stamp.

    Returns:
        PoseStamped: The converted PoseStamped message.
    """
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    if stamp is not None:
        msg.header.stamp = stamp

    msg.pose.position.x = float(pose_dict['position'][0])
    msg.pose.position.y = float(pose_dict['position'][1])
    msg.pose.position.z = float(pose_dict['position'][2])

    msg.pose.orientation.x = float(pose_dict['orientation'][0])
    msg.pose.orientation.y = float(pose_dict['orientation'][1])
    msg.pose.orientation.z = float(pose_dict['orientation'][2])
    msg.pose.orientation.w = float(pose_dict['orientation'][3])
    return msg


class DockDatabase(UserDict):
    """
    A helper class for managing the dock database.
    It inherits from collections.UserDict so that you can index/iterate directly
    on the database object (e.g., if dock_id in db: return db[dock_id]).

    It automatically queries map_server for the active map name,
    resolves the corresponding YAML database file under ~/stretch_user/maps/docks/,
    and loads/saves the dock parameters.
    """

    # Class-level version configuration
    DATABASE_VERSION = '1.0'

    # Route supported versions to their corresponding loading methods
    SUPPORTED_VERSIONS = {
        '1.0': '_load_v1_0'
    }

    @staticmethod
    def dict_to_pose_stamped(pose_dict, frame_id='map', stamp=None) -> PoseStamped:
        """Helper static method alias for pose_dict_to_pose_stamped."""
        return pose_dict_to_pose_stamped(pose_dict, frame_id, stamp)

    def __repr__(self) -> str:
        """String representation of the database with map name and loaded dock IDs."""
        return f"DockDatabase(map_name='{self.map_name}', version='{self.DATABASE_VERSION}', valid_dock_ids={list(self.keys())})"

    def __init__(self, node, default_map_name='map', on_load_callback=None) -> None:
        """
        Initialize the DockDatabase manager.

        Args:
            node: The parent ROS 2 node.
            default_map_name: Fallback map name if map_server is not queryable.
            on_load_callback: A parameterless callback function called after the DB is loaded.
        """
        super().__init__()

        self.node = node
        self.map_name = default_map_name
        self.on_load_callback = on_load_callback

        # Resolve initial DB filepath
        self.db_filepath = self._get_db_filepath(self.map_name)
        self.load_database()

        # Dedicated callback groups to avoid thread blocks
        self.cb_group = ReentrantCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Client to query map_server
        self.param_client = self.node.create_client(
            GetParameters, '/map_server/get_parameters', callback_group=self.cb_group
        )
        self._query_pending = False
        self.query_timer = self.node.create_timer(
            1.0, self._query_map_server_parameter, callback_group=self.timer_group
        )

    def _get_db_filepath(self, map_name: str) -> str:
        """Resolve and guarantee the directory exists for map docks."""
        config_dir = os.path.expanduser('~/stretch_user/maps/docks')
        os.makedirs(config_dir, exist_ok=True)
        return os.path.join(config_dir, f'{map_name}_docks.yaml')

    def load_database(self) -> bool:
        """Load the docks database from the resolved file path and route by version.

        Returns True if the database is in a well-defined state afterwards -- including a file
        that simply does not exist yet, which just means no docks have been discovered for this
        map. Returns False only when a file was present but could not be parsed.
        """
        self.clear()
        loaded = True
        if not os.path.exists(self.db_filepath):
            self.node.get_logger().info(
                f"No existing docks database found at {self.db_filepath}. A new one will be created."
            )
            self._notify_loaded()
            return loaded

        try:
            with open(self.db_filepath, 'r') as f:
                data = yaml.safe_load(f)
                if not data:
                    self._notify_loaded()
                    return loaded

                # Validate and parse version metadata
                version = data.get('version')
                if version is None:
                    raise ValueError("Database file is missing required 'version' metadata field.")

                if str(version) not in self.SUPPORTED_VERSIONS:
                    raise ValueError(
                        f"Unsupported database version '{version}'. "
                        f"Supported versions by this software release are: {list(self.SUPPORTED_VERSIONS.keys())}"
                    )

                # Route loading to version-specific method
                loader_method_name = self.SUPPORTED_VERSIONS[str(version)]
                loader_method = getattr(self, loader_method_name)
                loader_method(data)

                self.node.get_logger().info(
                    f"Successfully loaded {len(self)} docks from database (version {version}): {self.db_filepath}"
                )
        except Exception as e:
            self.node.get_logger().error(f"Failed to load docks database: {e}")
            loaded = False

        self._notify_loaded()
        return loaded

    def _notify_loaded(self) -> None:
        """Tell the owner the contents changed, so it can mirror them out."""
        if self.on_load_callback:
            try:
                self.on_load_callback()
            except Exception as e:
                self.node.get_logger().error(f"Error during on_load_callback: {e}")

    def reload_from(self, filepath: str) -> bool:
        """Point the database at `filepath` and reload it.

        Backs nav2_msgs/srv/ReloadDockDatabase. An explicit reload is a manual override, so it
        also stops the map_server poll -- otherwise a later map query would quietly swap the
        path back out from under the caller.
        """
        filepath = os.path.expanduser(filepath)
        if not os.path.isfile(filepath):
            self.node.get_logger().error(
                f"Cannot reload dock database: no such file '{filepath}'"
            )
            return False

        if not self.query_timer.is_canceled():
            self.query_timer.cancel()

        self.db_filepath = filepath
        # Keep map_name coherent with the file, so a later save_database writes back here.
        name, _ = os.path.splitext(os.path.basename(filepath))
        self.map_name = name[:-len('_docks')] if name.endswith('_docks') else name
        return self.load_database()

    def _load_v1_0(self, data: dict) -> None:
        """Loading logic for version 1.0 schema database files."""
        raw_docks = data.get('docks')
        if isinstance(raw_docks, dict):
            self.update(raw_docks)
        else:
            raise ValueError("Malformed version 1.0 schema: 'docks' field must be a dictionary map.")

    def save_database(self) -> None:
        """Save the docks database to the resolved file path."""
        try:
            with open(self.db_filepath, 'w') as f:
                db_data = {
                    'version': self.DATABASE_VERSION,
                    'docks': self.data
                }
                yaml.safe_dump(db_data, f, default_flow_style=False)
            self.node.get_logger().info(f"Successfully saved database to {self.db_filepath}")
        except Exception as e:
            self.node.get_logger().error(f"Failed to save docks database: {e}")

    def _query_map_server_parameter(self) -> None:
        """Query map_server node for yaml_filename parameter to dynamically identify map."""
        if not self.param_client.service_is_ready():
            return
        if self._query_pending:
            return

        self._query_pending = True
        req = GetParameters.Request()
        req.names = ['yaml_filename']

        future = self.param_client.call_async(req)
        future.add_done_callback(self._parameter_callback)

    def _parameter_callback(self, future: Future) -> None:
        self._query_pending = False
        try:
            response = future.result()
            if response and response.values:
                yaml_path = response.values[0].string_value
                if yaml_path:
                    filename = os.path.basename(yaml_path)
                    map_name, _ = os.path.splitext(filename)
                    if map_name and map_name != self.map_name:
                        self.map_name = map_name
                        self.db_filepath = self._get_db_filepath(self.map_name)
                        self.node.get_logger().info(
                            f"Queried map name '{self.map_name}'. Database Path: {self.db_filepath}"
                        )
                        self.load_database()
                        self.query_timer.cancel()
        except Exception as e:
            self.node.get_logger().error(f"Failed to get parameter from map_server: {str(e)}")
