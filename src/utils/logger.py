import logging
import os
from datetime import datetime

class FleetLogger:
    def __init__(self, log_file: str = None):
        """Initialize logger with file and console output"""
        # Create logs directory if it doesn't exist
        if not os.path.exists('logs'):
            os.makedirs('logs')

        # If no log file specified, create one with timestamp
        if log_file is None:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_file = f"logs/fleet_logs_{timestamp}.txt"

        # Configure file logging
        logging.basicConfig(
            filename=log_file,
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        # Add console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)
        logging.getLogger().addHandler(console_handler)

        self.logger = logging.getLogger(__name__)

    def info(self, message: str):
        """Log info level message"""
        self.logger.info(message)

    def warning(self, message: str):
        """Log warning level message"""
        self.logger.warning(message)

    def error(self, message: str):
        """Log error level message"""
        self.logger.error(message)

    def debug(self, message: str):
        """Log debug level message"""
        self.logger.debug(message)

    def critical(self, message: str):
        """Log critical level message"""
        self.logger.critical(message) 