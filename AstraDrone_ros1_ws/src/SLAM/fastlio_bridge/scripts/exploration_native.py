#!/usr/bin/env python3
"""Load optional compiled grid search; source-only environments use Python.

No run-time compiler or shell execution. Catkin builds/installs the library.
"""
import ctypes
import os
from pathlib import Path
import numpy as np


def load_search():
    paths = [Path(p)/'lib/libastra_exploration_search.so'
             for p in os.environ.get('CMAKE_PREFIX_PATH','').split(':') if p]
    # Source-tree tests without sourcing ROS.
    paths.append(Path(__file__).resolve().parents[4]/'devel/lib/libastra_exploration_search.so')
    for path in paths:
        try:
            library = ctypes.CDLL(str(path))
            function = library.astra_grid_search_weighted
            function.argtypes = [ctypes.c_void_p,ctypes.c_int,ctypes.c_int,ctypes.c_int,
                                 ctypes.c_double,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p]
            function.restype = ctypes.c_int
            return function
        except (OSError,AttributeError):
            continue
    return None


SEARCH = load_search()


def search(safe, start, resolution, penalties=None):
    if SEARCH is None:
        return None
    h,w = safe.shape
    mask = np.ascontiguousarray(safe,dtype=np.uint8)
    distances = np.empty((h,w),dtype=np.float64)
    parents = np.empty((h,w),dtype=np.int32)
    weights = None if penalties is None else np.ascontiguousarray(penalties,dtype=np.float32)
    if weights is not None and weights.shape!=mask.shape:
        raise ValueError('Penalty shape must match safe grid')
    status = SEARCH(mask.ctypes.data,h,w,start[0]*w+start[1],resolution,
                    weights.ctypes.data if weights is not None else None,
                    distances.ctypes.data,parents.ctypes.data)
    if status < 0:
        raise RuntimeError('Native exploration search rejected input: '+str(status))
    return distances,parents
