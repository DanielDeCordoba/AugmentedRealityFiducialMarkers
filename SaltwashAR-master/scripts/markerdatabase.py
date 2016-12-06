# Copyright (C) 2015 Ross D Milligan
# GNU GENERAL PUBLIC LICENSE Version 3 (full notice can be found at https://github.com/rdmilligan/SaltwashAR)

from constants import *

# marker table
MARKER_TABLE = [[[[0, 1, 0, 1, 0, 0, 0, 1, 1],
                  [0, 0, 1, 1, 0, 1, 0, 1, 0],
                  [1, 1, 0, 0, 0, 1, 0, 1, 0],
                  [0, 1, 0, 1, 0, 1, 1, 0, 0]],
                 M0],
                [[[1, 0, 0, 0, 1, 0, 1, 0, 1],
                  [0, 0, 1, 0, 1, 0, 1, 0, 1],
                  [1, 0, 1, 0, 1, 0, 0, 0, 1],
                  [1, 0, 1, 0, 1, 0, 1, 0, 0]],
                 M1],
                [[[1, 0, 1, 1, 1, 0, 0, 0, 1],
                  [1, 0, 1, 0, 1, 0, 1, 1, 0],
                  [1, 0, 0, 0, 1, 1, 1, 0, 1],
                  [0, 1, 1, 0, 1, 0, 1, 0, 1]],
                 M2],
                [[[1, 1, 1, 1, 1, 1, 0, 0, 1],
                  [1, 1, 1, 1, 1, 0, 1, 1, 0],
                  [1, 0, 0, 1, 1, 1, 1, 1, 1],
                  [0, 1, 1, 0, 1, 1, 1, 1, 1]],
                 M3],
                [[[0, 0, 0, 0, 0, 0, 1, 0, 0],
                  [0, 0, 0, 0, 0, 0, 0, 0, 1],
                  [0, 0, 1, 0, 0, 0, 0, 0, 0],
                  [1, 0, 0, 0, 0, 0, 0, 0, 0]],
                 M4],
                [[[0, 1, 0, 1, 1, 1, 1, 0, 0],
                  [0, 1, 0, 1, 1, 0, 0, 1, 1],
                  [0, 0, 1, 1, 1, 1, 0, 1, 0],
                  [1, 1, 0, 0, 1, 1, 0, 1, 0]],
                 M5],
                [[[1, 1, 1, 1, 0, 1, 0, 1, 1],
                  [1, 1, 1, 1, 0, 1, 1, 1, 0],
                  [1, 1, 0, 1, 0, 1, 1, 1, 1],
                  [0, 1, 1, 1, 0, 1, 1, 1, 1]],
                 M6],
                [[[1, 1, 1, 0, 1, 1, 0, 0, 1],
                  [1, 1, 1, 1, 1, 0, 1, 0, 0],
                  [1, 0, 0, 1, 1, 0, 1, 1, 1],
                  [0, 0, 1, 0, 1, 1, 1, 1, 1]],
                 M7],
                [[[1, 1, 1, 0, 1, 0, 0, 1, 1],
                  [1, 0, 1, 1, 1, 1, 1, 0, 0],
                  [1, 1, 0, 0, 1, 0, 1, 1, 1],
                  [0, 0, 1, 1, 1, 1, 1, 0, 1]],
                 M8],
                [[[1, 1, 1, 1, 1, 1, 0, 1, 1],
                  [1, 1, 1, 1, 1, 1, 1, 1, 0],
                  [1, 1, 0, 1, 1, 1, 1, 1, 1],
                  [0, 1, 1, 1, 1, 1, 1, 1, 1]],
                 M9],
                [[[1, 1, 1, 0, 1, 0, 0, 0, 1],
                  [1, 0, 1, 1, 1, 0, 1, 0, 0],
                  [1, 0, 0, 0, 1, 0, 1, 1, 1],
                  [0, 0, 1, 0, 1, 1, 1, 0, 1]],
                 M10]]

# match marker pattern to database record
def match_marker_pattern(marker_pattern):
    # marker_pattern is binary 3x3 array
    marker_found = False
    # store rotation (there are 4 possible rotations)
    marker_rotation = None
    # store which marker it is
    marker_name = None
    
    for marker_record in MARKER_TABLE:
        for idx, val in enumerate(marker_record[0]):    
            if marker_pattern == val: 
                marker_found = True
                marker_rotation = idx
                marker_name = marker_record[1]
                break
        if marker_found: break

    return (marker_found, marker_rotation, marker_name)