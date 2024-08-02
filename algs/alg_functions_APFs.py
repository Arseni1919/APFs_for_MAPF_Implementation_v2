import heapq

from globals import *
from functions_general import *
from functions_plotting import *


def init_apfs_map(
        map_dim: Tuple[int, int],
        max_path_len: int,
        params: dict
) -> np.ndarray | None:
    """
    apfs_np: [x, y, t] = float
    """
    if 'w' not in params:
        return None
    if max_path_len == 0:
        return None
    apfs_np = np.zeros((map_dim[0], map_dim[1], max_path_len))
    return apfs_np


def init_pibt_apfs_map(
        map_dim: Tuple[int, int],
        params: dict
) -> np.ndarray | None:
    """
    apfs_np: [x, y, t] = float
    """
    if 'w' not in params:
        return None
    apfs_np = np.zeros((map_dim[0], map_dim[1]))
    return apfs_np


def update_apfs_map(
        path: List[Node],
        apfs_np: np.ndarray,
        params: Dict,
        goal_node: Node | None = None
) -> np.ndarray | None:
    """
    apfs_np: [x, y, t] = float
    """
    if apfs_np is None:
        return None
    # w, d_max, gamma = get_apfs_params(params)
    w, d_max, gamma = params['w'], params['d_max'], params['gamma']
    for t, curr_node in enumerate(path):
        if goal_node is not None and curr_node == goal_node:
            continue
        open_list: Deque[Node] = deque([curr_node])
        closed_list: List[str] = []
        while len(open_list) > 0:
            next_n: Node = open_list.popleft()
            heapq.heappush(closed_list, next_n.xy_name)
            # np.abs(n1.x - n2.x) + np.abs(n1.y - n2.y)
            dist_x = abs(curr_node.x - next_n.x)
            if dist_x > d_max:
                continue
            dist_y = abs(curr_node.y - next_n.y)
            next_n_dist = dist_y + dist_x
            if next_n_dist > d_max:
                continue
            # next_n_dist = manhattan_dist(curr_node, next_n)
            # if next_n_dist <= d_max:
            apf_force: float = w * (gamma**(d_max - next_n_dist) / (gamma**d_max))
            apfs_np[next_n.x, next_n.y, t] += apf_force
            # extend open
            for nei_n in next_n.neighbours_nodes:
                if nei_n.xy_name in closed_list:
                    continue
                open_list.append(nei_n)
    return apfs_np


def update_pibt_apfs_map(
        next_node: Node,
        pibt_apfs_np: np.ndarray,
        params: Dict
) -> np.ndarray | None:
    """
    apfs_np: [x, y, t] = float
    """
    if pibt_apfs_np is None:
        return None
    # w, d_max, gamma = get_apfs_params(params)
    w, d_max, gamma = params['w'], params['d_max'], params['gamma']
    open_list: Deque[Node] = deque([next_node])
    closed_list: List[str] = []
    while len(open_list) > 0:
        next_n: Node = open_list.popleft()
        heapq.heappush(closed_list, next_n.xy_name)
        dist_x = abs(next_node.x - next_n.x)
        if dist_x > d_max:
            continue
        dist_y = abs(next_node.y - next_n.y)
        next_n_dist = dist_y + dist_x
        if next_n_dist > d_max:
            continue
        # next_n_dist = manhattan_dist(next_node, next_n)
        # if next_n_dist < d_max:
        apf_force: float = w * (gamma**(d_max - next_n_dist) / gamma**d_max)
        pibt_apfs_np[next_n.x, next_n.y] += apf_force
        # extend open
        for nei_n in next_n.neighbours_nodes:
            if nei_n.xy_name in closed_list:
                continue
            open_list.append(nei_n)
    return pibt_apfs_np


def get_k_apfs(
        path: List[Node],
        map_dim: Tuple[int, int],
        max_path_len: int,
        params: dict,
        goal_node: Node | None = None
):
    apfs_np = init_apfs_map(map_dim, max_path_len, params)
    return update_apfs_map(path, apfs_np, params, goal_node)


def append_apfs(
        apfs_np: np.ndarray,
        agent: AgentAlg | Any,
        params: dict
):
    if 'w' not in params:
        return None
    apfs_np[:, :, :agent.k_apfs.shape[2]] += agent.k_apfs
    return apfs_np


# def get_apfs_params(params: dict) -> Tuple[float | None, int | None, int | None]:
#     if 'w' not in params:
#         return None, None, None
#     w, d_max, gamma = params['w'], params['d_max'], params['gamma']
#     return w, d_max, gamma


# def get_apfs(
#         path: List[Node],
#         map_dim: Tuple[int, int],
#         max_path_len: int,
#         params: dict
# ):
#     apfs_np = init_apfs_map(map_dim, max_path_len, params)
#     return update_apfs_map(path, apfs_np, params)


# def update_sipps_apfs_map(
#         path: List[Node],
#         sipps_path: List[SIPPSNode],
#         apfs_np: np.ndarray,
#         params: Dict
# ) -> np.ndarray | None:
#     """
#     apfs_np: [x, y, t] = float
#     """
#     if apfs_np is None:
#         return None
#     # w, d_max, gamma = get_apfs_params(params)
#     w, d_max, gamma = params['w'], params['d_max'], params['gamma']
#     for curr_sipps_node in sipps_path:
#         for t in
#         open_list: Deque[Node] = deque([curr_node])
#         closed_list: List[str] = []
#         while len(open_list) > 0:
#             next_n: Node = open_list.popleft()
#             heapq.heappush(closed_list, next_n.xy_name)
#
#             next_n_dist = manhattan_dist(curr_node, next_n)
#             if next_n_dist < d_max:
#                 apf_force: float = w * (gamma**(d_max - next_n_dist) / gamma**d_max)
#                 apfs_np[next_n.x, next_n.y, t] += apf_force
#                 # extend open
#                 for nei_n in next_n.neighbours_nodes:
#                     if nei_n.xy_name in closed_list:
#                         continue
#                     open_list.append(nei_n)
#     return apfs_np


# def update_apfs_map(
#         path: List[Node],
#         apfs_np: np.ndarray,
#         params: Dict
# ) -> np.ndarray | None:
#     """
#     apfs_np: [x, y, t] = float
#     """
#     if apfs_np is None:
#         return None
#     # w, d_max, gamma = get_apfs_params(params)
#     w, d_max, gamma = params['w'], params['d_max'], params['gamma']
#     for t, curr_node in enumerate(path):
#         open_list: Deque[Node] = deque([curr_node])
#         open_list_names: List[str] = [curr_node.xy_name]
#         closed_list: List[str] = []
#         while len(open_list) > 0:
#             next_n: Node = open_list.popleft()
#             heapq.heappush(closed_list, next_n.xy_name)
#
#             next_n_dist = manhattan_dist(curr_node, next_n)
#             if next_n_dist < d_max:
#                 apf_force: float = w * (gamma ** (d_max - next_n_dist) / gamma ** d_max)
#                 apfs_np[next_n.x, next_n.y, t] += apf_force
#
#             # extend open
#             for nei_n in next_n.neighbours_nodes:
#                 if nei_n.xy_name in closed_list:
#                     continue
#                 if nei_n.xy_name in closed_list:
#                     continue
#                 if nei_n.xy_name in open_list_names:
#                     continue
#                 dist_x = abs(nei_n.x - curr_node.x)
#                 if dist_x >= d_max:
#                     continue
#                 dist_y = abs(nei_n.y - curr_node.y)
#                 if dist_y >= d_max:
#                     continue
#                 if dist_x + dist_y >= d_max:
#                     continue
#                 # dist = manhattan_dist(curr_node, nei_n)
#                 # if dist >= d_max:
#                 #     continue
#                 open_list.append(nei_n)
#                 heapq.heappush(open_list_names, nei_n.xy_name)
#             heapq.heappush(closed_list, next_n.xy_name)
#             # closed_list.append(next_n)
#
#     return apfs_np
