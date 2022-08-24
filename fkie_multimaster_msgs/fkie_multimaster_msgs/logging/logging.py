import os
from enum import Enum
import json
from json import JSONEncoder


class LoggingEncoder(JSONEncoder):
    def default(self, obj):
        return obj.__dict__


class LoggingLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    FATAL = "FATAL"


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


class ROS1Logger:
    '''
    Logger class compatible with ROS 1 using rospy
    '''

    def __init__(self) -> None:
        self._rospy = __import__('rospy')

    def log(self, level: LoggingLevel, message: str) -> None:
        if level == LoggingLevel.DEBUG:
            self._rospy.logdebug(f'{message}')
        if level == LoggingLevel.INFO:
            self._rospy.loginfo(f'{message}')
        if level == LoggingLevel.WARN:
            self._rospy.logwarn(f'{message}')
        if level == LoggingLevel.ERROR:
            self._rospy.logerr(f'{message}')
        if level == LoggingLevel.FATAL:
            self._rospy.logfatal(f'{message}')


class ROS2Logger:
    '''
    Logger class compatible with ROS 2 using rclpy
    '''

    def __init__(self) -> None:
        self._rclpy = __import__('rclpy')
        self._logger = self._rclpy.logging.get_logger("log")

    def log(self, level: LoggingLevel, message: str) -> None:
        if level == LoggingLevel.DEBUG:
            self._logger.debug(f'{message}')
        if level == LoggingLevel.INFO:
            self._logger.info(f'{message}')
        if level == LoggingLevel.WARN:
            self._logger.warn(f'{message}')
        if level == LoggingLevel.ERROR:
            self._logger.error(f'{message}')
        if level == LoggingLevel.FATAL:
            self._logger.fatal(f'{message}')


class GenericLogger:
    '''
    Generic Logger class that uses print (used as fallback)
    '''

    def log(self, level: LoggingLevel, message: str) -> None:
        if level == LoggingLevel.DEBUG:
            print(f'[DEBUG] {message}')
        if level == LoggingLevel.INFO:
            print(f'[INFO] {message}')
        if level == LoggingLevel.WARN:
            print(f'[WARN] {message}')
        if level == LoggingLevel.ERROR:
            print(f'[ERROR] {message}')
        if level == LoggingLevel.FATAL:
            print(f'[FATAL] {message}')


ROS_VERSION = get_ros_version()
logger = None

if(ROS_VERSION == 1):
    logger = ROS1Logger()
elif (ROS_VERSION == 2):
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
            details += f' {Log._clear_text(arg)} '

        return details
