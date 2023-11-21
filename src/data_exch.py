"""DataExchange class
"""

import numpy as np
import threading

class DataExchange():
    """DataExchange class
    Implements a data exchange between two threads

    Data includes a ring buffer and a current image
    """
    def __init__(self, ring_buff_size=10000):
        self._ring_buff = np.zeros((ring_buff_size, 4))
        self._image = None
        self._lock = threading.Lock()
    
    def set_data(self, data_entry, image=None):
        self._lock.acquire()
        self._ring_buff = np.roll(self._ring_buff, -1, axis=0)
        self._ring_buff[-1, :] = data_entry
        if image is not None:
            self._image = image.copy()
        self._lock.release()
    
    def get_data(self):
        self._lock.acquire()
        ring_buff = self._ring_buff.copy()
        if self._image is not None:
            current_image = self._image.copy()
        else:
            current_image = None
        self._lock.release()
        return ring_buff, current_image
