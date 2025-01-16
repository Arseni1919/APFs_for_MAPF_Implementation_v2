import re
import os
import math
import json
import time
import heapq
import random
import pstats
import cProfile
import itertools
from typing import *
from datetime import datetime
from collections import deque, defaultdict
from itertools import combinations, permutations, tee, pairwise

import numpy as np
import matplotlib
# matplotlib.use('TkAgg')

import matplotlib.pyplot as plt

# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# GLOBAL OBJECTS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #

color_names = [
    # 'b', 'g', 'r', 'c', 'm', 'y', 'k', 'w',  # Single-letter abbreviations
    'blue', 'green', 'red', 'cyan', 'magenta', 'black',  # Full names except 'white' 'yellow'
    'blueviolet', 'brown', 'burlywood', 'cadetblue', 'chartreuse', 'chocolate',
    'coral', 'cornflowerblue', 'cornsilk', 'crimson', 'cyan', 'darkblue', 'darkcyan', 'darkgoldenrod',
    'darkgray', 'darkgreen', 'darkgrey', 'darkkhaki', 'darkmagenta', 'darkolivegreen', 'darkorange', 'darkorchid',
    'darkred', 'darksalmon', 'darkseagreen', 'darkslateblue', 'darkslategray', 'darkslategrey', 'darkturquoise',
    'darkviolet', 'deeppink', 'deepskyblue', 'dimgray', 'dimgrey', 'dodgerblue', 'firebrick', 'floralwhite',
    'forestgreen', 'fuchsia', 'gainsboro', 'ghostwhite', 'gold', 'goldenrod', 'gray', 'green', 'greenyellow',
    'grey', 'honeydew', 'hotpink', 'indianred', 'indigo', 'ivory', 'khaki', 'lavender', 'lavenderblush', 'lawngreen',
    'lemonchiffon', 'lightblue', 'lightcoral', 'lightcyan', 'lightgoldenrodyellow', 'lightgray', 'lightgreen',
    'lightgrey', 'lightpink', 'lightsalmon', 'lightseagreen', 'lightskyblue', 'lightslategray', 'lightslategrey',
    'lightsteelblue', 'lightyellow', 'lime', 'limegreen', 'linen', 'magenta', 'maroon', 'mediumaquamarine',
    'mediumblue', 'mediumorchid', 'mediumpurple', 'mediumseagreen', 'mediumslateblue', 'mediumspringgreen',
    'mediumturquoise', 'mediumvioletred', 'midnightblue', 'mintcream', 'mistyrose', 'moccasin', 'navajowhite',
    'navy', 'oldlace', 'olive', 'olivedrab', 'orange', 'orangered', 'orchid', 'palegoldenrod', 'palegreen', 'paleturquoise',
    'palevioletred', 'papayawhip', 'peachpuff', 'peru', 'pink', 'plum', 'powderblue', 'purple', 'red', 'rosybrown',
    'royalblue', 'saddlebrown', 'salmon', 'sandybrown', 'seagreen', 'seashell', 'sienna', 'silver', 'skyblue', 'slateblue',
    'slategray', 'slategrey', 'snow', 'springgreen', 'steelblue', 'tan', 'teal', 'thistle', 'tomato', 'turquoise', 'violet',
    'wheat', 'white', 'whitesmoke', 'yellow', 'yellowgreen'
]


markers = [
    # ".",    # point marker
    # ",",    # pixel marker
    "o",    # circle marker
    "v",    # triangle_down marker
    "^",    # triangle_up marker
    "<",    # triangle_left marker
    ">",    # triangle_right marker
    "1",    # tri_down marker
    "2",    # tri_up marker
    "3",    # tri_left marker
    "4",    # tri_right marker
    "s",    # square marker
    "p",    # pentagon marker
    "*",    # star marker
    "h",    # hexagon1 marker
    "H",    # hexagon2 marker
    "+",    # plus marker
    "x",    # x marker
    "D",    # diamond marker
    "d",    # thin_diamond marker
    "P",    # plus (filled) marker
    "X",    # x (filled) marker
]
lines = [
    "-",  # solid line
    "--", # dashed line
    "-.", # dash-dot line
    ":",  # dotted line
]
markers_iter = iter(markers)
colors_iter = iter(color_names)

mrc_dict = {
    "L-PrP-A*":
        {'color': 'blue', 'marker-line': '-^'},
    "APF-L-PrP-A*":
        {'color': 'blue', 'marker-line': '--^'},
    "L-LNS2-A*":
        {'color': 'teal', 'marker-line': '-X'},
    "APF-L-LNS2-A*":
        {'color': 'teal', 'marker-line': '--X'},
    "L-PrP-SIPPS":
        {'color': 'orange', 'marker-line': '-v'},
    "APF-L-PrP-SIPPS":
        {'color': 'orange', 'marker-line': '--v'},
    "L-LNS2-SIPPS":
        {'color': 'peru', 'marker-line': '-P'},
    "APF-L-LNS2-SIPPS":
        {'color': 'peru', 'marker-line': '--P'},
    "L-PIBT":
        {'color': 'salmon', 'marker-line': '-h'},
    "APF-L-PIBT":
        {'color': 'salmon', 'marker-line': '--h'},
    "L-LaCAM":
        {'color': 'indigo', 'marker-line': '-1'},
    "L-LaCAM*":
        {'color': 'plum', 'marker-line': '-2'},
}


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# GLOBAL CLASSES
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
class Node:
    def __init__(self, x: int, y: int, neighbours: List[str] | None = None):
        self.x: int = x
        self.y: int = y
        self.neighbours: List[str] = [] if neighbours is None else neighbours
        self.neighbours_nodes: List[Node] = []
        self.xy_name: str = f'{self.x}_{self.y}'

    @property
    def xy(self):
        return self.x, self.y

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __lt__(self, other):
        return self.xy_name < other.xy_name

    def __gt__(self, other):
        return self.xy_name > other.xy_name

    def __hash__(self):
        return hash(self.xy_name)

    def __str__(self):
        return self.xy_name

    def __repr__(self):
        return self.xy_name


class AgentAlg:
    def __init__(self, num: int, start_node: Node, goal_node: Node):
        self.num = num
        self.name = f'agent_{num}'
        self.start_node: Node = start_node
        self.start_node_name: str = self.start_node.xy_name
        self.curr_node: Node = start_node
        self.curr_node_name: str = self.curr_node.xy_name
        self.goal_node: Node = goal_node
        self.goal_node_name: str = self.goal_node.xy_name
        self.path: List[Node] | None = [self.start_node]
        self.k_path: List[Node] | None = [self.start_node]
        self.k_apfs: np.ndarray | None = None
        self.init_priority: float = random.random()
        self.priority: float = self.init_priority

    @property
    def path_names(self):
        return [n.xy_name for n in self.path]

    def update_curr_node(self, i_time):
        if i_time >= len(self.path):
            self.curr_node = self.path[-1]
            return
        self.curr_node = self.path[i_time]

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

    def __lt__(self, other: Self):
        return self.priority < other.priority

    def __hash__(self):
        return hash(self.num)

    def __eq__(self, other):
        return self.num == other.num

