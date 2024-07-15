from globals import *
from functions_general import *
from functions_plotting import *
from algs.alg_pibt import run_procedure_pibt


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# CLASSES
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
class LowLevelNode:
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


class HighLevelNode:
    def __init__(
            self,
            config: Dict[str, Node],
            tree: Deque[LowLevelNode],
            order: List[AgentAlg],
            parent: Self | None,
            finished: int = 0,
            i: int = -1,
    ):
        self.config: Dict[str, Node] = config
        self.tree: Deque[LowLevelNode] = tree
        self.order: List[AgentAlg] = order
        self.parent: Self | None = parent
        self.finished: int = finished
        self.i: int = i
        self.name = get_config_name(self.config)

    def __eq__(self, other: Self):
        return self.name == other.name

    def __str__(self):
        return f'i={self.i}, {self.name}'

    def __repr__(self):
        return f'i={self.i}, {self.name}'


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# HELP FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def get_C_init():
    return LowLevelNode(None, None, None, 0)


def get_C_child(
        parent: LowLevelNode,
        who: AgentAlg,
        where: Node
) -> LowLevelNode:
    C_new = LowLevelNode(parent=parent, who=who, where=where, depth=parent.depth + 1)
    C_new.who_list = parent.who_list + [who]
    C_new.where_list = parent.where_list + [where]
    return C_new


# def get_h_value(a: AgentAlg) -> float:
#     curr_node = config[a.name]
#     h_map = h_dict[a.goal_node.xy_name]
#     res: float = float(h_map[curr_node.x, curr_node.y])
#     return res


def get_init_order(agents: List[AgentAlg]) -> List[AgentAlg]:
    out_list: List[AgentAlg] = agents[:]
    out_list.sort(key=lambda a: a.priority, reverse=True)
    return out_list


def get_order(
        config_new: Dict[str, Node],
        N: HighLevelNode,
) -> Tuple[List[AgentAlg], int]:
    out_list: List[AgentAlg] = N.order[:]
    finished = 0
    for i in out_list:
        if config_new[i.name] == i.goal_node:
            i.priority = i.init_priority
            finished += 1
        else:
            i.priority += 1
    out_list.sort(key=lambda a: a.priority, reverse=True)
    return out_list, finished


def get_config_name(config: Dict[str, Node]):
    assert len(config) > 0
    k_list = list(config.keys())
    k_list.sort()
    name = ''
    for k in k_list:
        v = config[k]
        name += v.xy_name + '-'
    return name[:-1]


def backtrack(N: HighLevelNode) -> Dict[str, List[Node]]:
    paths_deque_dict: Dict[str, Deque[Node]] = {k: deque([v]) for k, v in N.config.items()}
    parent: HighLevelNode = N.parent
    while parent is not None:
        for k, v in parent.config.items():
            paths_deque_dict[k].appendleft(v)
        parent = parent.parent

    paths_dict: Dict[str, List[Node]] = {}
    for k, v in paths_deque_dict.items():
        paths_dict[k] = list(v)

    return paths_dict


def get_new_config(
        N: HighLevelNode,
        C: LowLevelNode,
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


























