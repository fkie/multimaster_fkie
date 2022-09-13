import os
import json
from json import JSONEncoder
from .generic_logger import GenericLogger, LoggingLevel
from .ros1_logger import ROS1Logger
from .ros2_logger import ROS2Logger


class LoggingEncoder(JSONEncoder):
    def default(self, obj):
        return obj.__dict__


def get_ros_version() -> int:
    '''
    Returns the current ROS version, returns -1 if not valid ROS found
    '''
    if 'ROS_VERSION' in os.environ:
        return int(os.environ['ROS_VERSION'])

    # try to get ROS1
    try:
        import rospy
        return 1  # found ROS 1
    except:
        # try to get ROS 2
        try:
            import rclpy
            return 2  # found ROS 2
        except:
            pass

    # invalid ROS
    return -1


ros2_logging_node = None  # Global node required for ROS2 logging


ROS_VERSION = get_ros_version()
logger = None

if ROS_VERSION == 1:
    logger = ROS1Logger()
elif ROS_VERSION == 2:
    logger = ROS2Logger()
else:
    logger = GenericLogger()


class Log:
    '''
    Wrapper class for logging events in ROS 1 and 2
    '''

    @staticmethod
    def debug(*args) -> None:
        logger.log(LoggingLevel.DEBUG, Log._get_string_args(args))

    @staticmethod
    def info(*args) -> None:
        logger.log(LoggingLevel.INFO, Log._get_string_args(args))

    @staticmethod
    def warn(*args) -> None:
        logger.log(LoggingLevel.WARN, Log._get_string_args(args))

    @staticmethod
    def error(*args) -> None:
        logger.log(LoggingLevel.ERROR, Log._get_string_args(args))

    @staticmethod
    def fatal(*args) -> None:
        logger.log(LoggingLevel.FATAL, Log._get_string_args(args))

    @staticmethod
    def set_ros2_logging_node(node) -> None:
        global ros2_logging_node
        ros2_logging_node = node

    @staticmethod
    def _clear_text(text: str) -> str:
        if text is None:
            return ""

        if not isinstance(text, str):
            try:
                # try to get a JSON representation of the object
                text = json.dumps(
                    text, cls=LoggingEncoder, ensure_ascii=False)
            except:
                # if fails, use standard python string conversion
                text = str(text)

        # fix old formatting
        text = text.replace("%s", "")
        return text

    @staticmethod
    def _get_string_args(*args) -> str:
        if args is None:
            return ""

        details = ""
        for arg in args:
            for a in arg:
                details += f' {Log._clear_text(a)} '

        return details
