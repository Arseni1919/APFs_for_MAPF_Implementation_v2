from globals import *
from functions_general import *
from functions_plotting import *
from algs.alg_mapf_pibt import run_procedure_pibt
from algs.alg_functions_lacam import get_init_order, get_order, get_config_name


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# CLASSES
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
class LowLevelNodeStar:
    def __init__(self,
                 parent: Self | None,
                 who: AgentAlg | None,
                 where: Node | None,
                 depth: int):
        self.parent: Self | None = parent
        self.who: AgentAlg | None = who
        self.where: Node | None = where
        self.depth = depth
        self.who_list: List[AgentAlg] = []
        self.where_list: List[Node] = []

    def __str__(self):
        return f'~ depth={self.depth}, who_list={self.who_list}, where_list={self.where_list} ~'

    def __repr__(self):
        return f'~ depth={self.depth}, who_list={self.who_list}, where_list={self.where_list} ~'


class HighLevelNodeStar:
    def __init__(
            self,
            config: Dict[str, Node],
            tree: Deque[LowLevelNodeStar],
            order: List[AgentAlg],
            parent: Self | None,
            g: int = 0,
            h: int = 0,
            finished: int = 0,
    ):
        self.config: Dict[str, Node] = config
        self.tree: Deque[LowLevelNodeStar] = tree
        self.order: List[AgentAlg] = order
        self.parent: Self | None = parent
        self.finished: int = finished
        self.name = get_config_name(self.config)
        self.g: int = g
        self.h: int = h
        self.f: int = self.g + self.h
        self.neigh: Set[Self] = set()

    def __eq__(self, other: Self):
        return self.name == other.name

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

    def __hash__(self):
        return self.name.__hash__()

# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# HELP FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def get_C_init():
    return LowLevelNodeStar(None, None, None, 0)


def get_C_child(
        parent: LowLevelNodeStar,
        who: AgentAlg,
        where: Node
) -> LowLevelNodeStar:
    C_new = LowLevelNodeStar(parent=parent, who=who, where=where, depth=parent.depth + 1)
    C_new.who_list = parent.who_list + [who]
    C_new.where_list = parent.where_list + [where]
    return C_new


def backtrack(N: HighLevelNodeStar) -> Dict[str, List[Node]]:
    paths_deque_dict: Dict[str, Deque[Node]] = {k: deque([v]) for k, v in N.config.items()}
    parent: HighLevelNodeStar = N.parent
    while parent is not None:
        for k, v in parent.config.items():
            paths_deque_dict[k].appendleft(v)
        parent = parent.parent

    paths_dict: Dict[str, List[Node]] = {}
    for k, v in paths_deque_dict.items():
        paths_dict[k] = list(v)

    return paths_dict


def get_new_config(
        N: HighLevelNodeStar,
        C: LowLevelNodeStar,
        agents_dict: Dict[str, AgentAlg],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
) -> Dict[str, Node] | None:
    # setup next configuration
    config_from: Dict[str, Node] = N.config
    occupied_from: Dict[str, AgentAlg] = {v.xy_name: agents_dict[k] for k, v in N.config.items()}
    config_to: Dict[str, Node] = {}
    occupied_to: Dict[str, AgentAlg] = {}
    for k in range(C.depth):
        agent = C.who_list[k]
        node = C.where_list[k]
        config_to[agent.name] = node
        # vc
        if node.xy_name in occupied_to:
            return None
        occupied_to[node.xy_name] = agent
        # ec
        if node.xy_name in occupied_from:
            other_agent = occupied_from[node.xy_name]
            if other_agent != agent and other_agent.name in config_to and config_to[other_agent.name] == config_from[agent.name]:
                return None

    # apply PIBT
    for agent in N.order:
        if agent.name not in config_to:
            success = run_procedure_pibt(
                agent,
                config_from, occupied_from,
                config_to, occupied_to,
                agents_dict, nodes_dict, h_dict, [])
            if not success:
                return None
    return config_to


def get_edge_cost(
        agents: List[AgentAlg],
        config_from: Dict[str, Node],
        config_to: Dict[str, Node],
):
    # e.g., \sum_i | not (Q_from[i] == Q_to[k] == g_i) |
    cost = 0
    for agent in agents:
        if not (agent.goal_node == config_from[agent.name] == config_to[agent.name]):
            cost += 1
    return cost


def get_h_value(
        config: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        agents: List[AgentAlg]
) -> int:
    # e.g., \sum_i dist(Q[i], g_i)
    cost = 0
    for agent in agents:
        goal_np = h_dict[agent.goal_node_name]
        curr_n = config[agent.name]
        c: float = float(goal_np[curr_n.x, curr_n.y])
        cost += c
    return cost



