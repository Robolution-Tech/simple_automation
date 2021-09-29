import functools
import time
from json import JSONEncoder
import numpy as np

def timer(func):
	@functools.wraps(func)
	def wrapper_timer(*args, **kwargs):
		start_time = time.time()
		value = func(*args, **kwargs)
		end_time = time.time()
		run_time = end_time - start_time
		print("Finished {} in {} secs".format(func.__name__, run_time))
		return value
	return wrapper_timer 	

class NumpyArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)