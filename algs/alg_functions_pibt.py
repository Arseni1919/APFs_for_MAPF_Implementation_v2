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


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def get_sorted_nei_nodes(
        agent: AgentAlg,
        config_from: Dict[str, Node],
        # nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
):
    h_goal_np: np.ndarray = h_dict[agent.goal_node.xy_name]
    # sort C in ascending order of dist(u, gi) where u ∈ C
    # nei_nodes: List[Node] = [nodes_dict[n_name] for n_name in config_from[agent.name].neighbours]
    nei_nodes: List[Node] = config_from[agent.name].neighbours_nodes[:]
    random.shuffle(nei_nodes)

    def get_nei_v(n: Node) -> float:
        return float(h_goal_np[n.x, n.y])

    nei_nodes.sort(key=get_nei_v)
    return nei_nodes


def there_is_vc(
        nei_node: Node,
        config_to: Dict[str, Node],
) -> bool:
    for name, n in config_to.items():
        if nei_node == n:
            return True
    return False


def get_agent_k(
        nei_node: Node,
        occupied_from: Dict[str, AgentAlg],
        config_to: Dict[str, Node],
) -> AgentAlg | None:
    if nei_node.xy_name in occupied_from:
        other_agent = occupied_from[nei_node.xy_name]
        if other_agent.name not in config_to:
            return other_agent
    return None
    # for a_f_name, n_f_node in config_from.items():
    #     if n_f_node == nei_node and a_f_name not in config_to:
    #         return agents_dict[a_f_name]
    # return None


def there_is_ec(
        agent_i: AgentAlg,
        node_to: Node,
        config_from: Dict[str, Node],
        config_to: Dict[str, Node],
) -> bool:
    node_from = config_from[agent_i.name]
    for other_name, other_node_from in config_from.items():
        if other_name == agent_i.name or other_name not in config_to:
            continue
        other_node_to = config_to[other_name]
        if other_node_from == node_to and other_node_to == node_from:
            return True
    return False


def get_next_node(node: Node, blocked: List[Node]) -> Node | None:
    nei_nodes = node.neighbours_nodes[:]
    nei_nodes.remove(node)
    for n in blocked:
        nei_nodes.remove(n)
    if len(nei_nodes) == 0:
        return None
    return random.choice(nei_nodes)


def check_if_swap_required(
        agent_i: AgentAlg,
        agent_j: AgentAlg,
        config_from: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
) -> bool:
    """
    This is done by continuously moving i to j’s location while moving j to another vertex not equal to i’s location,
    ignoring the other agents.
    The emulation stops in two cases:
    (i) The swap is not required when j’s location has a degree of more than two.
    (ii) The swap is required when
        (1) j’s location has a degree of one,
        or,
        (2) when i reaches gi while j’s nearest neighboring vertex toward its goal is gi.
    """
    prev_node_i = config_from[agent_i.name]
    prev_node_j = config_from[agent_j.name]
    while True:

        next_node_i = prev_node_j
        next_node_j = get_next_node(prev_node_j, blocked=[prev_node_i])

        if next_node_j is None:
            return True

        if len(next_node_j.neighbours) > 3:
            return False

        if next_node_i == agent_i.goal_node:
            nei_nodes_j = get_sorted_nei_nodes(agent_j, config_from, h_dict)
            nearest_nei_to_goal_j = nei_nodes_j[0]
            if nearest_nei_to_goal_j == agent_i.goal_node:
                return True

        prev_node_i = next_node_i
        prev_node_j = next_node_j


def check_if_swap_possible(
        agent_i: AgentAlg,
        agent_j: AgentAlg,
        config_from: Dict[str, Node],
) -> bool:
    """
    This is done by reversing the emulation direction; that is,
    continuously moving j to i’s location while moving i to another vertex.
    It stops in two cases:
        (i) The swap is possible when i’s location has a degree of more than two.
        (ii) The swap is impossible when i is on a vertex with degree of one.
    :return:
    """
    prev_node_i = config_from[agent_i.name]
    prev_node_j = config_from[agent_j.name]
    while True:

        next_node_j = prev_node_i
        next_node_i = get_next_node(prev_node_i, blocked=[prev_node_j])

        if next_node_i is None:
            return False

        if len(next_node_i.neighbours) > 3:
            return True

        prev_node_i = next_node_i
        prev_node_j = next_node_j


def swap_required_and_possible(
        agent_i: AgentAlg,
        first_node: Node,
        config_from: Dict[str, Node],
        occupied_from: Dict[str, AgentAlg],
        h_dict: Dict[str, np.ndarray],
        with_swap: bool,
) -> AgentAlg | None:
    if not with_swap:
        return None
    if len(first_node.neighbours) - 1 <= 2 and first_node.xy_name in occupied_from:
        agent_j: AgentAlg = occupied_from[first_node.xy_name]
        if agent_j == agent_i:
            return None
        # necessity of the swap
        is_required = check_if_swap_required(agent_i, agent_j, config_from, h_dict)
        if not is_required:
            return None
        # possibility of the swap
        is_possible = check_if_swap_possible(agent_i, agent_j, config_from)
        if not is_possible:
            return None
        return agent_j
    return None


def run_procedure_pibt(
        agent_i: AgentAlg,
        config_from: Dict[str, Node],
        occupied_from: Dict[str, AgentAlg],
        config_to: Dict[str, Node],
        occupied_to: Dict[str, AgentAlg],
        agents_dict: Dict[str, AgentAlg],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        blocked_nodes: List[Node],
        with_swap: bool = True
        # with_swap: bool = False
) -> bool:  # valid or invalid

    # nei_nodes = get_sorted_nei_nodes(agent_i, config_from, nodes_dict, h_dict)
    nei_nodes = get_sorted_nei_nodes(agent_i, config_from, h_dict)

    #  j ← swap_required_and_possible
    agent_j = swap_required_and_possible(agent_i, nei_nodes[0], config_from, occupied_from, h_dict, with_swap)
    if agent_j is not None:
        nei_nodes.reverse()

    for j, nei_node in enumerate(nei_nodes):

        if nei_node.xy_name in occupied_to:
            continue

        node_from = config_from[agent_i.name]
        if node_from.xy_name in occupied_to:
            other_agent = occupied_to[node_from.xy_name]
            if other_agent != agent_i and config_from[other_agent.name] == nei_node:
                continue

        if nei_node in blocked_nodes:
            continue

        config_to[agent_i.name] = nei_node
        occupied_to[nei_node.xy_name] = agent_i
        agent_k = get_agent_k(nei_node, occupied_from, config_to)
        if agent_k is not None:
            valid = run_procedure_pibt(
                agent_k,
                config_from, occupied_from,
                config_to, occupied_to,
                agents_dict, nodes_dict, h_dict, blocked_nodes
            )
            if not valid:
                continue
        if with_swap and nei_node == nei_nodes[0] and agent_j is not None and agent_j.name not in config_to:
            i_node_from = config_from[agent_i.name]
            config_to[agent_j.name] = i_node_from
            occupied_to[i_node_from.xy_name] = agent_j
        return True
    node_from = config_from[agent_i.name]
    config_to[agent_i.name] = node_from
    occupied_to[node_from.xy_name] = agent_i
    return False












