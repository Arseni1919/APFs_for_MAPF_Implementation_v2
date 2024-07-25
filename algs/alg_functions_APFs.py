import heapq

from globals import *
from functions_general import *
from functions_plotting import *
from alg_sipps_functions import SIPPSNode


def get_apfs_params(params: dict) -> Tuple[float | None, int | None, int | None]:
    if 'w' not in params:
        return None, None, None
    w, d_max, gamma = params['w'], params['d_max'], params['gamma']
    return w, d_max, gamma


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


def get_k_apfs(
        path: List[Node],
        map_dim: Tuple[int, int],
        max_path_len: int,
        params: dict
):
    apfs_np = init_apfs_map(map_dim, max_path_len, params)
    return update_apfs_map(path, apfs_np, params)


# def get_apfs(
#         path: List[Node],
#         map_dim: Tuple[int, int],
#         max_path_len: int,
#         params: dict
# ):
#     apfs_np = init_apfs_map(map_dim, max_path_len, params)
#     return update_apfs_map(path, apfs_np, params)


def append_apfs(
        apfs_np: np.ndarray,
        agent: AgentAlg | Any,
        params: dict
):
    if 'w' not in params:
        return None
    apfs_np[:, :, :agent.k_apfs.shape[2]] += agent.k_apfs
    return apfs_np


def update_apfs_map(
        path: List[Node],
        apfs_np: np.ndarray,
        params: Dict
) -> np.ndarray | None:
    """
    apfs_np: [x, y, t] = float
    """
    if apfs_np is None:
        return None
    # w, d_max, gamma = get_apfs_params(params)
    w, d_max, gamma = params['w'], params['d_max'], params['gamma']
    for t, curr_node in enumerate(path):
        open_list: Deque[Node] = deque([curr_node])
        closed_list: List[str] = []
        while len(open_list) > 0:
            next_n: Node = open_list.popleft()
            heapq.heappush(closed_list, next_n.xy_name)

            next_n_dist = manhattan_dist(curr_node, next_n)
            if next_n_dist < d_max:
                apf_force: float = w * (gamma**(d_max - next_n_dist) / gamma**d_max)
                apfs_np[next_n.x, next_n.y, t] += apf_force
                # extend open
                for nei_n in next_n.neighbours_nodes:
                    if nei_n.xy_name in closed_list:
                        continue
                    open_list.append(nei_n)
    return apfs_np


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
