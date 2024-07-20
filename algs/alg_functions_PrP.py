import heapq
import random

from globals import *
from functions_general import *
from functions_plotting import *


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# CLASSES
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
class AgentPrP:
    def __init__(self, num: int, start_node: Node, goal_node: Node):
        self.num = num
        self.name = f'agent_{num}'
        self.start_node: Node = start_node
        self.curr_node: Node = start_node
        self.goal_node: Node = goal_node
        self.path: List[Node] = []
        self.k_path: List[Node] | None = None

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


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def create_prp_agents(
        start_nodes: List[Node], goal_nodes: List[Node]
) -> Tuple[List[AgentPrP], Dict[str, AgentPrP]]:
    agents: List[AgentPrP] = []
    agents_dict: Dict[str, AgentPrP] = {}
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentPrP(num, s_node, g_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent
    return agents, agents_dict


def create_hard_and_soft_constraints(
        h_priority_agents: List[AgentPrP],
        map_dim: Tuple[int, int],
        constr_type: str,
        flag_k_limit: bool = False,
):
    assert constr_type in ['hard', 'soft']
    if len(h_priority_agents) == 0:
        max_path_len = 1
        vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, max_path_len)
        vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, max_path_len)
        # vc_hard_np, ec_hard_np, pc_hard_np = None, None, None
        # vc_soft_np, ec_soft_np, pc_soft_np = None, None, None
        return vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np
    if flag_k_limit:
        max_path_len = max([len(a.k_path) for a in h_priority_agents])
        paths = [a.k_path for a in h_priority_agents]
    else:
        max_path_len = max([len(a.path) for a in h_priority_agents])
        paths = [a.path for a in h_priority_agents]
    if constr_type == 'hard':
        vc_hard_np, ec_hard_np, pc_hard_np = create_constraints(paths, map_dim, max_path_len)
        vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, max_path_len)
        # vc_soft_np, ec_soft_np, pc_soft_np = None, None, None
    elif constr_type == 'soft':
        vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, max_path_len)
        # vc_hard_np, ec_hard_np, pc_hard_np = None, None, None
        vc_soft_np, ec_soft_np, pc_soft_np = create_constraints(paths, map_dim, max_path_len)
    else:
        raise RuntimeError('nope')
    return vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np


def solution_is_found(agents: List[AgentPrP]):
    for agent in agents:
        if agent.path is None:
            return False
        if len(agent.path) == 0:
            return False
        if agent.path[-1] != agent.goal_node:
            return False
    return True


def get_shuffled_agents(agents: List[AgentPrP]) -> List[AgentPrP]:
    agents_copy = agents[:]
    random.shuffle(agents_copy)
    unfinished: List[AgentPrP] = [a for a in agents_copy if len(a.path) == 0 or a.path[-1] != a.goal_node]
    finished: List[AgentPrP] = [a for a in agents_copy if len(a.path) > 0 and a.path[-1] == a.goal_node]
    return [*unfinished, *finished]

