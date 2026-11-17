import getpass
import json
import os
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

LAUNCH_LOCK_DIR = Path('/tmp/stretch_ros2_launch')

# Prefixes the message a skipped launch file logs. stretch4_web_teleop's launch
# health check looks for it, so keep it stable.
SINGLETON_LOG_TAG = '[launch singleton]'


def launch_file_id(launch_file: str) -> str:
    """Names a launch file by package and path under its launch/ directory.

    '.../share/stretch_core/launch/stretch_driver.launch.py' -> 'stretch_core/stretch_driver.launch.py'
    The source tree path gives the same id as the install space path, so the lock is shared.
    """
    parts = Path(launch_file).parts[:-1]
    if 'launch' in parts:
        i = len(parts) - 1 - parts[::-1].index('launch')
        if i > 0:
            return '/'.join((parts[i - 1], *parts[i + 1:], Path(launch_file).name))
    return Path(launch_file).name


def _launch_lock_path(launch_id: str) -> Path:
    return LAUNCH_LOCK_DIR / (launch_id.replace('/', '.') + '.lock')


def check_launch_singleton(launch_id: str, acquire: bool = True) -> bool:
    """Returns True if the launch file's lock is available and, if acquire is True, has been acquired.
    Otherwise returns False.

    The lock is held by this process until it exits, so it is released when `ros2 launch` exits.
    A second acquire from the same process returns False too: a launch file included twice
    in one launch is a duplicate as well.
    """
    from stretch4_body.utils.file_access_utils import acquire_lock_if_available, is_locked, setup_shared_directory

    lock_file = _launch_lock_path(launch_id)
    setup_shared_directory(lock_file.parent)

    if not acquire:
        return not is_locked(str(lock_file))
    return acquire_lock_if_available(str(lock_file), remove_if_exists_and_unused=True)


def _own_command() -> str:
    try:
        argv = Path('/proc/self/cmdline').read_bytes().decode(errors='replace').split('\0')
    except OSError:
        return ''
    argv = [a for a in argv if a]
    for i, arg in enumerate(argv):
        if os.path.basename(arg) == 'ros2':
            return ' '.join(['ros2'] + argv[i + 1:])
    return ' '.join(argv)


def _write_holder_info(launch_id: str, args: dict):
    """Records who holds the lock in the lock file, so a skipped launch can say who has it."""
    try:
        user = getpass.getuser()
    except Exception:
        user = str(os.getuid())
    info = {
        'pid': os.getpid(),
        'user': user,
        'started': time.strftime('%Y-%m-%d %H:%M:%S'),
        'command': _own_command(),
        'args': args,
    }
    try:
        # No O_CREAT: the file exists, and creating it is what fs.protected_regular forbids.
        fd = os.open(_launch_lock_path(launch_id), os.O_WRONLY | os.O_TRUNC)
        with os.fdopen(fd, 'w') as f:
            json.dump(info, f)
    except OSError:
        pass


def _read_holder_info(launch_id: str) -> dict:
    try:
        return json.loads(_launch_lock_path(launch_id).read_text())
    except (OSError, ValueError):
        return {}


def _skip_message(launch_id: str, args: dict) -> str:
    info = _read_holder_info(launch_id)
    if info.get('pid') == os.getpid():
        holder = ' earlier in this same launch'
    elif info:
        holder = (f" in another process (pid {info.get('pid')}, user {info.get('user')}, "
                  f"since {info.get('started')}: {info.get('command')})")
    else:
        holder = ' in another process'
    msg = f'{SINGLETON_LOG_TAG} {launch_id} is already running{holder}. Not starting a second copy.'

    theirs = info.get('args') or {}
    diffs = [f'{name}={theirs[name]} (this launch asked for {value})'
             for name, value in args.items()
             if name in theirs and theirs[name] is not None and value is not None and theirs[name] != value]
    if diffs:
        msg += ' The running copy was started with different arguments: ' + ', '.join(diffs) + '.'
    return msg


class _LaunchSingletonGroup(GroupAction):
    """Runs its actions only if this process gets the launch file's lock.

    The lock is taken when the group executes, not when the launch description is generated:
    `launch` generates included descriptions just to list their arguments, even for includes
    whose condition is false (e.g. launch_driver:=false), and those must not take the lock.
    """

    def __init__(self, launch_id: str, actions: list, arg_names: list):
        super().__init__(actions, scoped=False)
        self.__launch_id = launch_id
        self.__arg_names = arg_names

    def execute(self, context):
        args = {name: context.launch_configurations.get(name) for name in self.__arg_names}
        try:
            acquired = check_launch_singleton(self.__launch_id)
        except ImportError:
            # Without stretch4_body (e.g. simulation-only installs) there is no lock to take.
            return super().execute(context)
        except Exception as e:
            # A broken lock directory should not stop the robot from launching.
            note = LogInfo(msg=f'{SINGLETON_LOG_TAG} Could not check whether {self.__launch_id} '
                               f'is already running ({e!r}). Starting it anyway.')
            return [note, *super().execute(context)]
        if not acquired:
            return [LogInfo(msg=_skip_message(self.__launch_id, args))]
        _write_holder_info(self.__launch_id, args)
        return super().execute(context)


def launch_singleton(ld: LaunchDescription, launch_file: str) -> LaunchDescription:
    """
    Makes a launch file a singleton: if another `ros2 launch` (or an earlier include in the
    same launch) is already running it, this one logs who is and starts nothing.

    The launch arguments stay declared either way, so `--show-args` and the
    launch arguments of including files are unaffected.

    Usage:
        def generate_launch_description():
            ...
            return launch_singleton(LaunchDescription([...]), __file__)
    """
    declarations = [e for e in ld.entities if isinstance(e, DeclareLaunchArgument)]
    actions = [e for e in ld.entities if not isinstance(e, DeclareLaunchArgument)]
    group = _LaunchSingletonGroup(launch_file_id(launch_file), actions, [d.name for d in declarations])
    return LaunchDescription(declarations + [group])

def _remove_qt_plugin_env_vars() -> list:
    """
    This resolves the QT plugin that causes rviz to not launch, described in https://github.com/hello-robot/stretch4_ros2/issues/19
    
    Usage:
        return [
            *_remove_qt_plugin_env_vars(),
            rviz_node,
        ]
    """
    return [
        UnsetEnvironmentVariable(name='QT_QPA_PLATFORM_PLUGIN_PATH'),
        UnsetEnvironmentVariable(name='QT_QPA_FONTDIR'),
        UnsetEnvironmentVariable(name='QT_PLUGIN_PATH'),
    ]


def get_rviz_node(rviz_file_path:str|None=None, *, rviz_args:list|None = None, rviz_params:dict|None = None, launch_configration_key:str|None = 'use_rviz') -> list:
    """
    Use this to get the rviz node in your launch file. This automatically adds _remove_qt_plugin_env_vars()

    Only one of rviz_file_path or rviz_args is allowed to be not None.

    If `launch_configration_key` is None, IfCondition will not be checked.

    Usage:
        return [
            your other nodes
        ] + get_rviz_node()

    """
    if rviz_file_path is not None and rviz_args is not None:
        raise ValueError("Only one of rviz_file_path or rviz_args can be used. You may want to add ['-d', rviz_file_path] to your rviz_args instead.")

    arguments = rviz_args
    if rviz_file_path is not None:
        arguments = ["-d", rviz_file_path]

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments= arguments,
        parameters=[rviz_params] if rviz_params is not None else [],
        condition=IfCondition(LaunchConfiguration(launch_configration_key)) if launch_configration_key is not None else None,
    )
    return _remove_qt_plugin_env_vars() + [rviz_node]