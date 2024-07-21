import numpy as np

from globals import *


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# HELP FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def set_seed(random_seed_bool, seed=1):
    if random_seed_bool:
        seed = random.randint(0, 10000)
    random.seed(seed)
    np.random.seed(seed)
    print(f'[SEED]: --- {seed} ---')


def get_dims_from_pic(img_dir: str, path: str = 'maps') -> Tuple[int, int]:
    with open(f'{path}/{img_dir}') as f:
        lines = f.readlines()
        height = int(re.search(r'\d+', lines[1]).group())
        width = int(re.search(r'\d+', lines[2]).group())
    return height, width


def get_np_from_dot_map(img_dir: str, path: str = 'maps') -> Tuple[np.ndarray, Tuple[int, int]]:
    with open(f'{path}/{img_dir}') as f:
        lines = f.readlines()
        height, width = get_dims_from_pic(img_dir, path)
        img_np = np.zeros((height, width))
        for height_index, line in enumerate(lines[4:]):
            for width_index, curr_str in enumerate(line):
                if curr_str == '.':
                    img_np[height_index, width_index] = 1
        return img_np, (height, width)


def build_graph_from_np(img_np: np.ndarray, show_map: bool = False) -> Tuple[List[Node], Dict[str, Node]]:
    # 0 - wall, 1 - free space
    nodes = []
    nodes_dict = {}

    x_size, y_size = img_np.shape
    # CREATE NODES
    for i_x in range(x_size):
        for i_y in range(y_size):
            if img_np[i_x, i_y] == 1:
                node = Node(i_x, i_y)
                nodes.append(node)
                nodes_dict[node.xy_name] = node

    # CREATE NEIGHBOURS
    for node1, node2 in combinations(nodes, 2):
        if abs(node1.x - node2.x) > 1 or abs(node1.y - node2.y) > 1:
            continue
        if abs(node1.x - node2.x) == 1 and abs(node1.y - node2.y) == 1:
            continue
        node1.neighbours.append(node2.xy_name)
        node2.neighbours.append(node1.xy_name)
        # dist = distance_nodes(node1, node2)
        # if dist == 1:

    for node in nodes:
        node.neighbours.append(node.xy_name)
        heapq.heapify(node.neighbours)

    for node in nodes:
        for nei in node.neighbours:
            node.neighbours_nodes.append(nodes_dict[nei])

    if show_map:
        plt.imshow(img_np, cmap='gray', origin='lower')
        plt.show()
        # plt.pause(1)
        # plt.close()

    return nodes, nodes_dict


def exctract_h_dict(img_dir, path) -> Dict[str, np.ndarray]:
    # print(f'Started to build heuristic for {kwargs['img_dir'][:-4]}...')
    possible_dir = f"{path}/h_dict_of_{img_dir[:-4]}.json"

    # if there is one
    if os.path.exists(possible_dir):
        # Opening JSON file
        with open(possible_dir, 'r') as openfile:
            # Reading from json file
            h_dict = json.load(openfile)
            for k, v in h_dict.items():
                h_dict[k] = np.array(v)
            return h_dict

    raise RuntimeError('nu nu')


def create_constraints(
        paths: List[List[Node]], map_dim: Tuple[int, int], max_path_len: int
) -> Tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """
    vc_np: vertex constraints [x, y, t] = bool
    ec_np: edge constraints [x, y, x, y, t] = bool
    pc_np: permanent constraints [x, y] = int or -1
    """
    if len(paths) == 0:
        return None, None, None
    if max_path_len == 0:
        return None, None, None
    vc_np = np.zeros((map_dim[0], map_dim[1], max_path_len))
    ec_np = np.zeros((map_dim[0], map_dim[1], map_dim[0], map_dim[1], max_path_len))
    pc_np = np.ones((map_dim[0], map_dim[1])) * -1
    for path in paths:
        update_constraints(path, vc_np, ec_np, pc_np)
    return vc_np, ec_np, pc_np


def init_constraints(
        map_dim: Tuple[int, int], max_path_len: int
) -> Tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """
    vc_np: vertex constraints [x, y, t] = bool
    ec_np: edge constraints [x, y, x, y, t] = bool
    pc_np: permanent constraints [x, y] = int or -1
    """
    if max_path_len == 0:
        return None, None, None
    vc_np = np.zeros((map_dim[0], map_dim[1], max_path_len))
    ec_np = np.zeros((map_dim[0], map_dim[1], map_dim[0], map_dim[1], max_path_len))
    pc_np = np.ones((map_dim[0], map_dim[1])) * -1
    return vc_np, ec_np, pc_np


def init_ec_table(
        map_dim: Tuple[int, int], max_path_len: int
) -> np.ndarray | None:
    """
    ec_np: edge constraints [x, y, x, y, t] = bool
    """
    if max_path_len == 0:
        return None, None, None
    ec_np = np.zeros((map_dim[0], map_dim[1], map_dim[0], map_dim[1], max_path_len))
    return ec_np


def update_ec_table(
        path: List[Node], ec_np: np.ndarray
) -> np.ndarray | None:
    """
    ec_np: edge constraints [x, y, x, y, t] = bool
    """
    prev_n = path[0]
    for t, n in enumerate(path):
        # ec
        ec_np[prev_n.x, prev_n.y, n.x, n.y, t] = 1
        prev_n = n
    return ec_np


def update_constraints(
        path: List[Node], vc_np: np.ndarray, ec_np: np.ndarray, pc_np: np.ndarray
) -> Tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
    """
    vc_np: vertex constraints [x, y, t] = bool
    ec_np: edge constraints [x, y, x, y, t] = bool
    pc_np: permanent constraints [x, y] = int or -1
    """
    # pc
    last_node = path[-1]
    last_time = len(path) - 1
    pc_np[last_node.x, last_node.y] = max(int(pc_np[last_node.x, last_node.y]), last_time)
    prev_n = path[0]
    for t, n in enumerate(path):
        # vc
        vc_np[n.x, n.y, t] = 1
        # ec
        ec_np[prev_n.x, prev_n.y, n.x, n.y, t] = 1
        prev_n = n
    return vc_np, ec_np, pc_np


def align_all_paths(agents: List, flag_k_limit: bool = False) -> int:
    if len(agents) == 0:
        return 1
    if flag_k_limit:
        max_len = max([len(a.k_path) for a in agents])
        for a in agents:
            while len(a.k_path) < max_len:
                a.k_path.append(a.k_path[-1])
        return max_len
    max_len = max([len(a.path) for a in agents])
    for a in agents:
        while len(a.path) < max_len:
            a.path.append(a.path[-1])
    return max_len


def shorten_back_all_paths(agents: List) -> None:
    for a in agents:
        if len(a.path) <= 2:
            continue
        minus_1_node = a.path[-1]
        minus_2_node = a.path[-2]
        while minus_1_node == minus_2_node:
            a.path = a.path[:-1]
            if len(a.path) <= 2:
                break
            minus_1_node = a.path[-1]
            minus_2_node = a.path[-2]


def check_one_vc_ec_neic_iter(path: List[Node], agent_name, other_paths: Dict[str, List[Node]], iteration: int) -> None:
    collisions: int = 0
    for agent_other_name, other_path in other_paths.items():
        # vertex conf
        assert path[iteration] != other_path[iteration], f'[i: {iteration}] vertex conf: {path[iteration].xy_name}'
        # edge conf
        prev_node1 = path[max(0, iteration - 1)]
        curr_node1 = path[iteration]
        prev_node2 = other_path[max(0, iteration - 1)]
        curr_node2 = other_path[iteration]
        edge1 = (prev_node1.x, prev_node1.y, curr_node1.x, curr_node1.y)
        edge2 = (curr_node2.x, curr_node2.y, prev_node2.x, prev_node2.y)
        # nei conf
        assert path[iteration].xy_name in path[max(0, iteration - 1)].neighbours, f'[i: {iteration}] wow wow wow! Not nei pos!'
        assert edge1 != edge2, f'[i: {iteration}] edge collision: {edge1}'
    assert path[iteration].xy_name in path[max(0, iteration - 1)].neighbours, f'[i: {iteration}] wow wow wow! Not nei pos!'


def check_vc_ec_neic_iter(agents: list | Deque, iteration: int, to_count: bool = False) -> int:
    collisions: int = 0
    for a1, a2 in combinations(agents, 2):
        # vertex conf
        if not to_count:
            assert a1.path[iteration] != a2.path[iteration], f'[i: {iteration}] vertex conf: {a1.name}-{a2.name} in {a1.path[iteration].xy_name}'
        else:
            if a1.path[iteration] == a2.path[iteration]:
                collisions += 1
        # edge conf
        prev_node1 = a1.path[max(0, iteration - 1)]
        curr_node1 = a1.path[iteration]
        prev_node2 = a2.path[max(0, iteration - 1)]
        curr_node2 = a2.path[iteration]
        edge1 = (prev_node1.x, prev_node1.y, curr_node1.x, curr_node1.y)
        edge2 = (curr_node2.x, curr_node2.y, prev_node2.x, prev_node2.y)
        # nei conf
        assert a1.path[iteration].xy_name in a1.path[max(0, iteration - 1)].neighbours, f'[i: {iteration}] wow wow wow! Not nei pos!'
        if not to_count:
            assert edge1 != edge2, f'[i: {iteration}] edge collision: {a1.name}-{a2.name} in {edge1}'
        else:
            if edge1 == edge2:
                collisions += 1
    assert agents[-1].path[iteration].xy_name in agents[-1].path[max(0, iteration - 1)].neighbours, f'[i: {iteration}] wow wow wow! Not nei pos!'
    return collisions


def check_configs(
        agents: List[AgentAlg],
        config_from: Dict[str, Node],
        config_to: Dict[str, Node],
        final_check: bool = True
) -> None:
    if final_check:
        for agent in agents:
            assert agent.name in config_from
            assert agent.name in config_to
    for a1, a2 in combinations(agents, 2):
        if a1.name not in config_to or a2.name not in config_to:
            continue
        # vertex conf
        from_node_1: Node = config_from[a1.name]
        to_node_1: Node = config_to[a1.name]
        from_node_2: Node = config_from[a2.name]
        to_node_2: Node = config_to[a2.name]

        # vc
        assert from_node_1 != from_node_2, f' vc: {a1.name}-{a2.name} in {from_node_1.xy_name}'
        assert to_node_1 != to_node_2, f' vc: {a1.name}-{a2.name} in {to_node_2.xy_name}'

        # edge conf
        edge1 = (from_node_1.x, from_node_1.y, to_node_1.x, to_node_1.y)
        edge2 = (to_node_2.x, to_node_2.y, from_node_2.x, from_node_2.y)
        assert edge1 != edge2, f'ec: {a1.name}-{a2.name} in {edge1}'

        # nei conf
        assert from_node_1.xy_name in to_node_1.neighbours, f'neic {a1.name}: {from_node_1.xy_name} not nei of {to_node_1.xy_name}'
        assert from_node_2.xy_name in to_node_2.neighbours, f'neic {a2.name}: {from_node_2.xy_name} not nei of {to_node_2.xy_name}'


def ranges_intersect(range1, range2):
    start1, end1 = range1
    start2, end2 = range2
    return start1 <= end2 and start2 <= end1


def use_profiler(save_dir):
    def decorator(func):
        def inner1(*args, **kwargs):
            profiler = cProfile.Profile()
            profiler.enable()
            # getting the returned value
            returned_value = func(*args, **kwargs)
            profiler.disable()
            stats = pstats.Stats(profiler).sort_stats('cumtime')
            stats.dump_stats(save_dir)
            # returning the value to the original frame
            return returned_value
        return inner1
    return decorator


def two_plans_have_no_confs(path1: List[Node], path2: List[Node]):

    min_len = min(len(path1), len(path2))
    # assert len(path1) == len(path2)
    prev1 = None
    prev2 = None
    bigger_path = path1 if len(path1) >= len(path2) else path2
    smaller_path = path1 if len(path1) < len(path2) else path2
    if smaller_path[-1] in bigger_path[len(smaller_path) - 1:]:
        return False
    for i, (vertex1, vertex2) in enumerate(zip(path1[:min_len], path2[:min_len])):
        if vertex1.x == vertex2.x and vertex1.y == vertex2.y:
            return False
        if i > 0:
            # edge1 = (prev1.xy_name, vertex1.xy_name)
            # edge2 = (vertex2.xy_name, prev2.xy_name)
            # if (prev1.x, prev1.y, vertex1.x, vertex1.y) == (vertex2.x, vertex2.y, prev2.x, prev2.y):
            if prev1.x == vertex2.x and prev1.y == vertex2.y and vertex1.x == prev2.x and vertex1.y == prev2.y:
                return False
        prev1 = vertex1
        prev2 = vertex2
    return True


def two_equal_paths_have_confs(path1: List[Node], path2: List[Node]):
    assert len(path1) == len(path2)
    from1 = None
    from2 = None
    for i, (to1, to2) in enumerate(zip(path1, path2)):
        if to1.x == to2.x and to1.y == to2.y:
            return True
        if i > 0:
            if from1.x == to2.x and from1.y == to2.y and to1.x == from2.x and to1.y == from2.y:
                return True
        from1 = to1
        from2 = to2
    return False


def create_agents(
        start_nodes: List[Node], goal_nodes: List[Node]
) -> Tuple[List[AgentAlg], Dict[str, AgentAlg]]:
    agents: List[AgentAlg] = []
    agents_dict: Dict[str, AgentAlg] = {}
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentAlg(num, s_node, g_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent
    return agents, agents_dict


def time_is_good(start_time: int | float, max_time: int) -> bool:
    runtime = time.time() - start_time
    return runtime < max_time


def align_path(new_path: List[Node], k_limit: int) -> List[Node]:
    while len(new_path) < k_limit:
        new_path.append(new_path[-1])
    return new_path[:k_limit]


def manhattan_dist(n1: Node, n2: Node):
    """
    The Manhattan Distance between two points (X1, Y1) and (X2, Y2) is given by |X1 – X2| + |Y1 – Y2|.
    :return:
    """
    return np.abs(n1.x - n2.x) + np.abs(n1.y - n2.y)


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# LIFELONG / k-LIMITED FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #


def add_k_paths_to_agents(agents: List[AgentAlg] | list) -> None:
    for agent in agents:
        if len(agent.path) == 0:
            agent.path.append(agent.start_node)
        agent.path.extend(agent.k_path[1:])


def update_goal_nodes(agents: List[AgentAlg] | list, nodes: List[Node]) -> int:
    finished_goals = 0
    goal_names: List[str] = [a.goal_node.xy_name for a in agents]
    heapq.heapify(goal_names)
    free_nodes: List[Node] = [n for n in nodes if n.xy_name not in goal_names]
    for agent in agents:
        if agent.curr_node == agent.goal_node:
            finished_goals += 1
            next_goal_node = random.choice(free_nodes)
            while next_goal_node.xy_name in goal_names:
                next_goal_node = random.choice(free_nodes)
            agent.goal_node = next_goal_node
            heapq.heappush(goal_names, next_goal_node.xy_name)
    return finished_goals


def stay_k_path_agent(agent: AgentAlg | Any, n: Node, k_limit: int) -> None:
    agent.k_path = [n]
    while len(agent.k_path) < k_limit:
        agent.k_path.append(n)
    agent.k_path = agent.k_path[:k_limit]


def exceeds_k_dist(n1: Node, n2: Node, k_limit: int) -> bool:
    dist_x = abs(n1.x - n2.x)
    if dist_x > k_limit:
        return True
    dist_y = abs(n1.y - n2.y)
    if dist_y > k_limit:
        return True
    if dist_x + dist_y > k_limit:
        return True
    return False


def repair_agents_k_paths(agents: List[AgentAlg] | list, k_limit: int) -> None:
    standby_agents_dict: Dict[str, bool] = {}
    for agent in agents:
        if agent.k_path is None or len(agent.k_path) == 0:
            stay_k_path_agent(agent, agent.curr_node, k_limit + 1)
            standby_agents_dict[agent.name] = True
        else:
            standby_agents_dict[agent.name] = False

    all_good = False
    while not all_good:
        all_good = True
        for a1, a2 in combinations(agents, 2):
            if standby_agents_dict[a1.name] and standby_agents_dict[a2.name]:
                continue
            if exceeds_k_dist(a1.curr_node, a2.curr_node, k_limit + 1):
                continue
            if two_equal_paths_have_confs(a1.k_path, a2.k_path):
                stay_k_path_agent(a1, a1.curr_node, k_limit + 1)
                stay_k_path_agent(a2, a2.curr_node, k_limit + 1)
                standby_agents_dict[a1.name] = True
                standby_agents_dict[a2.name] = True
                all_good = False
                break
    print(' | repaired')
    return
