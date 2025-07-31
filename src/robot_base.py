"""Base class for Robot
"""

from abc import ABC, abstractmethod

class RobotBase(ABC):
    """Base class for Robot
    """
    @abstractmethod
    def run(self):
        """run robot"""
        pass

    @abstractmethod
    def stop(self):
        """stop robot"""
        pass

    @abstractmethod
    def exec_cmd(self):
        """execute command from controller"""
        pass

    @abstractmethod
    def send_data(self):
        """send data to controller"""
        pass
