from globals import *
from functions_general import *
from functions_plotting import *
from algs.alg_sipps import run_sipps


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# CLASSES
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
class AgentLNS2:
    def __init__(self, num: int, start_node: Node, goal_node: Node):
        self.num = num
        self.name = f'agent_{num}'
        self.start_node: Node = start_node
        self.start_node_name: str = self.start_node.xy_name
        self.curr_node: Node = start_node
        self.curr_node_name: str = self.curr_node.xy_name
        self.goal_node: Node = goal_node
        self.goal_node_name: str = self.goal_node.xy_name
        self.path: List[Node] | None = []
        self.k_path: List[Node] | None = None

    def __str__(self):
        return f'{self.name}'

    @property
    def path_names(self):
        return [n.xy_name for n in self.path]

    def update_curr_node(self, i_time):
        if i_time >= len(self.path):
            self.curr_node = self.path[-1]
            return
        self.curr_node = self.path[i_time]

    def __lt__(self, other):
        return self.num < other.num

    def __hash__(self):
        return hash(self.num)

    def __eq__(self, other):
        return self.num == other.num


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def create_lns_agents(
        start_nodes: List[Node], goal_nodes: List[Node]
) -> Tuple[List[AgentLNS2], Dict[str, AgentLNS2]]:
    agents: List[AgentLNS2] = []
    agents_dict: Dict[str, AgentLNS2] = {}
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentLNS2(num, s_node, g_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent
    return agents, agents_dict


def solution_is_found(agents: List[AgentLNS2]):
    for agent in agents:
        if agent.path is None:
            return False
        if len(agent.path) == 0:
            return False
        if agent.path[-1] != agent.goal_node:
            return False
    return True


def get_shuffled_agents(agents: List[AgentLNS2]) -> List[AgentLNS2]:
    agents_copy = agents[:]
    random.shuffle(agents_copy)
    unfinished: List[AgentLNS2] = [a for a in agents_copy if len(a.path) == 0 or a.path[-1] != a.goal_node]
    finished: List[AgentLNS2] = [a for a in agents_copy if len(a.path) > 0 and a.path[-1] == a.goal_node]
    return [*unfinished, *finished]


def solve_subset_with_prp(
        agents_subset: List[AgentLNS2],
        outer_agents: List[AgentLNS2],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        start_time: int | float,
        # constr_type: str = 'hard',
        constr_type: str = 'soft',
        agents: List[AgentLNS2] | None = None
) -> None:
    c_sum: int = 0
    h_priority_agents: List[AgentLNS2] = outer_agents[:]
    random.shuffle(agents_subset)
    for agent in agents_subset:
        align_all_paths(h_priority_agents)
        (vc_hard_np, ec_hard_np, pc_hard_np,
         vc_soft_np, ec_soft_np, pc_soft_np) = create_hard_and_soft_constraints(h_priority_agents, map_dim,
                                                                                constr_type)
        new_path, sipps_info = run_sipps(
            agent.start_node, agent.goal_node, nodes, nodes_dict, h_dict,
            vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np, agent=agent
        )
        shorten_back_all_paths(h_priority_agents)
        if new_path is None:
            agent.path = None
            break
        agent.path = new_path[:]
        h_priority_agents.append(agent)
        c_sum += sipps_info['c']

        # checks
        runtime = time.time() - start_time
        assert len(agents_subset) + len(outer_agents) == len(agents)
        print(
            f'\r[nei calc] | agents: {len(h_priority_agents): <3} / {len(agents_subset) + len(outer_agents)} | {runtime= : .2f} s.',
            end='')  # , end=''
        # collisions: int = 0
        # align_all_paths(h_priority_agents)
        # for i in range(len(h_priority_agents[0].path)):
        #     to_count = False if constr_type == 'hard' else True
        #     collisions += check_vc_ec_neic_iter(h_priority_agents, i, to_count)
        # if c_sum > 0:
        #     print(f'{c_sum=}')


def create_init_solution(
        agents: List[AgentLNS2],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        constr_type: str,
        start_time: int | float
):
    c_sum: int = 0
    h_priority_agents: List[AgentLNS2] = []
    for agent in agents:
        align_all_paths(h_priority_agents)
        (vc_hard_np, ec_hard_np, pc_hard_np,
         vc_soft_np, ec_soft_np, pc_soft_np) = create_hard_and_soft_constraints(h_priority_agents, map_dim,
                                                                                constr_type)
        new_path, sipps_info = run_sipps(
            agent.start_node, agent.goal_node, nodes, nodes_dict, h_dict,
            vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np, agent=agent
        )
        shorten_back_all_paths(h_priority_agents)
        if new_path is None:
            agent.path = None
            break
        agent.path = new_path[:]
        h_priority_agents.append(agent)
        c_sum += sipps_info['c']

        # checks
        runtime = time.time() - start_time
        print(f'\r[LNS2 - init] | agents: {len(h_priority_agents): <3} / {len(agents)} | {runtime= : .2f} s.', end='\n')  # , end=''


def create_hard_and_soft_constraints(h_priority_agents: List[AgentLNS2], map_dim: Tuple[int, int], constr_type: str):
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


def get_cp_graph(
        agents: List[AgentLNS2],
        other_agents: List[AgentLNS2] | None = None,
        prev_cp_graph: Dict[str, List[AgentLNS2]] | None = None,
) -> Tuple[Dict[str, List[AgentLNS2]], Dict[str, List[str]]]:
    if other_agents is None:
        other_agents = []
    # align_all_paths(agents)
    cp_graph: Dict[str, List[AgentLNS2]] = {}
    for a1, a2 in combinations(agents, 2):
        if not two_plans_have_no_confs(a1.path, a2.path):
            if a1.name not in cp_graph:
                cp_graph[a1.name] = []
            if a2.name not in cp_graph:
                cp_graph[a2.name] = []
            cp_graph[a1.name].append(a2)
            cp_graph[a2.name].append(a1)
    for other_a in other_agents:
        if other_a.name in prev_cp_graph:
            if other_a.name not in cp_graph:
                cp_graph[other_a.name] = []
            for nei in prev_cp_graph[other_a.name]:
                if nei not in agents:
                    cp_graph[other_a.name].append(nei)
        for a in agents:
            if not two_plans_have_no_confs(other_a.path, a.path):
                if other_a.name not in cp_graph:
                    cp_graph[other_a.name] = []
                if a.name not in cp_graph:
                    cp_graph[a.name] = []
                cp_graph[other_a.name].append(a)
                cp_graph[a.name].append(other_a)
    return cp_graph, {}


def get_outer_agent_via_random_walk(
        rand_agent: AgentLNS2,
        agents_s: List[AgentLNS2],
        occupied_from: Dict[str, AgentLNS2]
) -> AgentLNS2:
    next_node: Node = random.choice(rand_agent.path)
    while True:
        if next_node.xy_name in occupied_from and occupied_from[next_node.xy_name] not in agents_s:
            return occupied_from[next_node.xy_name]
        next_node = random.choice(next_node.neighbours_nodes)


def get_agent_s_from_random_walk(
        curr_agent: AgentLNS2,
        cp_graph: Dict[str, List[AgentLNS2]],
        n_neighbourhood: int,
) -> List[AgentLNS2]:
    out_list: List[AgentLNS2] = []
    next_nei: AgentLNS2 = curr_agent
    while len(out_list) < n_neighbourhood:
        nei_agents = cp_graph[next_nei.name]
        next_nei = random.choice(nei_agents)
        if next_nei not in out_list:
            out_list.append(next_nei)
    return out_list


def get_agents_subset(
        cp_graph: Dict[str, List[AgentLNS2]],
        cp_graph_names: Dict[str, List[str]],
        n_neighbourhood: int,
        agents: List[AgentLNS2],
        occupied_from: Dict[str, AgentLNS2],
        h_dict: Dict[str, np.ndarray],
) -> List[AgentLNS2]:
    agents_with_cp: List[AgentLNS2] = [a for a in agents if a.name in cp_graph]
    curr_agent: AgentLNS2 = random.choice(agents_with_cp)

    # find largest connected component
    lcc: List[AgentLNS2] = []
    l_open = deque([curr_agent])
    i = 0
    while len(l_open) > 0:
        i += 1
        next_a = l_open.pop()
        heapq.heappush(lcc, next_a)
        random.shuffle(cp_graph[next_a.name])
        for nei_a in cp_graph[next_a.name]:
            if nei_a not in lcc and nei_a not in l_open:
                l_open.append(nei_a)

    agents_s: List[AgentLNS2] = []
    if len(lcc) <= n_neighbourhood:
        agents_s.extend(lcc)
        while len(agents_s) < n_neighbourhood:
            rand_agent = random.choice(agents_s)
            outer_agent = get_outer_agent_via_random_walk(rand_agent, agents_s, occupied_from)
            agents_s.append(outer_agent)
        return agents_s
    else:
        # add until N agents
        agents_s = get_agent_s_from_random_walk(curr_agent, cp_graph, n_neighbourhood)
        return agents_s


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# K LIMITED Functions
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
def create_k_limit_init_solution(
        agents: List[AgentLNS2],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        constr_type: str,
        k_limit: int,
        start_time: int | float
):
    c_sum: int = 0
    h_priority_agents: List[AgentLNS2] = []
    vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, k_limit + 1)
    vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, k_limit + 1)
    for agent in agents:
        new_path, sipps_info = run_sipps(
            agent.curr_node, agent.goal_node, nodes, nodes_dict, h_dict,
            vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np,
            flag_k_limit=True, k_limit=k_limit, agent=agent
        )
        new_path = align_path(new_path, k_limit + 1)
        agent.k_path = new_path[:]
        h_priority_agents.append(agent)
        update_constraints(new_path, vc_soft_np, ec_soft_np, pc_soft_np)
        c_sum += sipps_info['c']

        # checks
        # runtime = time.time() - start_time
        # print(f'\r[LNS2 - init] | agents: {len(h_priority_agents): <3} / {len(agents)} | {c_sum=: <3} | {runtime=: .2f} s.', end='\n')  # , end=''


def get_k_limit_cp_graph(
        agents: List[AgentLNS2],
        other_agents: List[AgentLNS2] | None = None,
        prev_cp_graph: Dict[str, List[AgentLNS2]] | None = None,
) -> Tuple[Dict[str, List[AgentLNS2]], Dict[str, List[str]]]:
    if other_agents is None:
        other_agents = []
    # align_all_paths(agents)
    cp_graph: Dict[str, List[AgentLNS2]] = {}
    for a1, a2 in combinations(agents, 2):
        if not two_plans_have_no_confs(a1.k_path, a2.k_path):
            if a1.name not in cp_graph:
                cp_graph[a1.name] = []
            if a2.name not in cp_graph:
                cp_graph[a2.name] = []
            cp_graph[a1.name].append(a2)
            cp_graph[a2.name].append(a1)
    for other_a in other_agents:
        if other_a.name in prev_cp_graph:
            if other_a.name not in cp_graph:
                cp_graph[other_a.name] = []
            for nei in prev_cp_graph[other_a.name]:
                if nei not in agents:
                    cp_graph[other_a.name].append(nei)
        for a in agents:
            if not two_plans_have_no_confs(other_a.k_path, a.k_path):
                if other_a.name not in cp_graph:
                    cp_graph[other_a.name] = []
                if a.name not in cp_graph:
                    cp_graph[a.name] = []
                cp_graph[other_a.name].append(a)
                cp_graph[a.name].append(other_a)
    return cp_graph, {}


def get_k_limit_outer_agent_via_random_walk(
        rand_agent: AgentLNS2,
        agents_s: List[AgentLNS2],
        occupied_from: Dict[str, AgentLNS2]
) -> AgentLNS2:
    next_node: Node = random.choice(rand_agent.k_path)
    while True:
        if next_node.xy_name in occupied_from and occupied_from[next_node.xy_name] not in agents_s:
            return occupied_from[next_node.xy_name]
        next_node = random.choice(next_node.neighbours_nodes)


def get_k_limit_agents_subset(
        cp_graph: Dict[str, List[AgentLNS2]],
        cp_graph_names: Dict[str, List[str]],
        n_neighbourhood: int,
        agents: List[AgentLNS2],
        occupied_from: Dict[str, AgentLNS2],
        h_dict: Dict[str, np.ndarray],
) -> List[AgentLNS2]:
    agents_with_cp: List[AgentLNS2] = [a for a in agents if a.name in cp_graph]
    curr_agent: AgentLNS2 = random.choice(agents_with_cp)

    # find largest connected component
    lcc: List[AgentLNS2] = []
    l_open = deque([curr_agent])
    i = 0
    while len(l_open) > 0:
        i += 1
        next_a = l_open.pop()
        heapq.heappush(lcc, next_a)
        random.shuffle(cp_graph[next_a.name])
        for nei_a in cp_graph[next_a.name]:
            if nei_a not in lcc and nei_a not in l_open:
                l_open.append(nei_a)

    agents_s: List[AgentLNS2] = []
    if len(lcc) <= n_neighbourhood:
        agents_s.extend(lcc)
        while len(agents_s) < n_neighbourhood:
            rand_agent = random.choice(agents_s)
            outer_agent = get_k_limit_outer_agent_via_random_walk(rand_agent, agents_s, occupied_from)
            agents_s.append(outer_agent)
        return agents_s
    else:
        # add until N agents
        agents_s = get_agent_s_from_random_walk(curr_agent, cp_graph, n_neighbourhood)
        return agents_s


def solve_k_limit_subset_with_prp(
        agents_subset: List[AgentLNS2],
        outer_agents: List[AgentLNS2],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        start_time: int | float,
        # constr_type: str = 'hard',
        constr_type: str = 'soft',
        k_limit: int = int(1e10),
        agents: List[AgentLNS2] | None = None
) -> None:
    c_sum: int = 0

    h_priority_agents: List[AgentLNS2] = outer_agents[:]
    vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, k_limit + 1)
    vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, k_limit + 1)
    for h_agent in h_priority_agents:
        update_constraints(h_agent.k_path, vc_soft_np, ec_soft_np, pc_soft_np)

    random.shuffle(agents_subset)

    for agent in agents_subset:
        new_path, sipps_info = run_sipps(
            agent.curr_node, agent.goal_node, nodes, nodes_dict, h_dict,
            vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np,
            flag_k_limit=True, k_limit=k_limit, agent=agent
        )
        new_path = align_path(new_path, k_limit + 1)
        agent.k_path = new_path[:]
        h_priority_agents.append(agent)
        update_constraints(new_path, vc_soft_np, ec_soft_np, pc_soft_np)
        c_sum += sipps_info['c']

        # checks
        # runtime = time.time() - start_time
        # assert len(agents_subset) + len(outer_agents) == len(agents)
        # print(f'\r[LNS neigh calc] | agents: {len(h_priority_agents): <3} / {len(agents_subset) + len(outer_agents)} | {runtime=: .2f} s.',end='')  # , end=''
        # collisions: int = 0
        # align_all_paths(h_priority_agents)
        # for i in range(len(h_priority_agents[0].path)):
        #     to_count = False if constr_type == 'hard' else True
        #     collisions += check_vc_ec_neic_iter(h_priority_agents, i, to_count)
        # if c_sum > 0:
        #     print(f'{c_sum=}')
