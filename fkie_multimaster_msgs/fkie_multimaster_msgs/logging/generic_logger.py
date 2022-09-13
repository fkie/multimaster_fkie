from enum import Enum


class LoggingLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    FATAL = "FATAL"


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

    def debug(self, message: str) -> None:
        print(f'[DEBUG] {message}')

    def info(self, message: str) -> None:
        print(f'[INFO] {message}')

    def warn(self, message: str) -> None:
        print(f'[WARN] {message}')

    def error(self, message: str) -> None:
        print(f'[ERROR] {message}')

    def fatal(self, message: str) -> None:
        print(f'[FATAL] {message}')
