from .generic_logger import GenericLogger, LoggingLevel


class ROS1Logger:
    '''
    Logger class compatible with ROS 1 using rospy
    '''

    def __init__(self) -> None:
        self._rospy = __import__('rospy')
        self.generic_logger = GenericLogger()

    def log(self, level: LoggingLevel, message: str) -> None:
        if self._rospy.core.is_initialized():
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
        else:
            # use generic logger because rospy is not available (running in a non-ros-node process)
            if level == LoggingLevel.DEBUG:
                self.generic_logger.debug(f'{message}')
            if level == LoggingLevel.INFO:
                self.generic_logger.info(f'{message}')
            if level == LoggingLevel.WARN:
                self.generic_logger.warn(f'{message}')
            if level == LoggingLevel.ERROR:
                self.generic_logger.error(f'{message}')
            if level == LoggingLevel.FATAL:
                self.generic_logger.fatal(f'{message}')
