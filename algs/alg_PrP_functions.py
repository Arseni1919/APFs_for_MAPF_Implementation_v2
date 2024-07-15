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
        self.start_node_name: str = self.start_node.xy_name
        self.curr_node: Node = start_node
        self.curr_node_name: str = self.curr_node.xy_name
        self.goal_node: Node = goal_node
        self.goal_node_name: str = self.goal_node.xy_name
        self.path: List[Node] | None = None

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
def create_hard_and_soft_constraints(h_priority_agents: List[AgentPrP], map_dim: Tuple[int, int], constr_type: str):
    assert constr_type in ['hard', 'soft']
    if len(h_priority_agents) == 0:
        max_path_len = 1
        vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, max_path_len)
        vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, max_path_len)
        # vc_hard_np, ec_hard_np, pc_hard_np = None, None, None
        # vc_soft_np, ec_soft_np, pc_soft_np = None, None, None
        return vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np
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

