from algs.alg_functions_lacam import *
from run_single_MAPF_func import run_mapf_alg


def solve_k_lacam(
        agents: List[AgentAlg],
        agents_dict: Dict[str, AgentAlg],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict
) -> Tuple[None, Dict] | Tuple[Dict[str, List[Node]], Dict]:

    k_limit: int = params['k_limit']
    max_iter_time: int = params['max_iter_time']
    # max_time = params['max_time']
    # alg_name = params['alg_name']
    # to_render: bool = params['to_render']
    # img_np: np.ndarray = params['img_np']

    # if to_render:
    #     fig, ax = plt.subplots(1, 2, figsize=(14, 7))

    start_time = time.time()

    # create agents
    n_agents = len(agents_dict)

    config_start: Dict[str, Node] = {a.name: a.curr_node for a in agents}
    config_goal: Dict[str, Node] = {a.name: a.goal_node for a in agents}
    config_goal_name: str = get_config_name(config_goal)

    open_list: Deque[HighLevelNode] = deque()  # stack
    explored_dict: Dict[str, HighLevelNode] = {}   # stack

    init_order = get_init_order(agents)
    N_init: HighLevelNode = HighLevelNode(
        config=config_start, tree=deque([get_C_init()]), order=init_order, parent=None
    )
    open_list.appendleft(N_init)
    explored_dict[N_init.name] = N_init

    iteration = 0
    while len(open_list) > 0:
        N: HighLevelNode = open_list[0]

        if N.name == config_goal_name or backtrack_N_number(N) >= k_limit or not time_is_good(start_time, max_iter_time):

            paths_dict = backtrack(N)
            for a_name, path in paths_dict.items():
                new_path = align_path(path, k_limit + 1)
                agents_dict[a_name].k_path = new_path
            # checks
            # for i in range(len(agents[0].path)):
            #     check_vc_ec_neic_iter(agents, i, to_count=False)
            runtime = time.time() - start_time
            makespan: int = max([len(a.path) for a in agents])
            return paths_dict, {'agents': agents, 'time': runtime, 'makespan': makespan}

        # low-level search end
        if len(N.tree) == 0:
            open_list.popleft()
            continue

        # low-level search
        C: LowLevelNode = N.tree.popleft()  # constraints
        if C.depth < n_agents:
            i_agent = N.order[C.depth]
            v = N.config[i_agent.name]
            neighbours = v.neighbours[:]
            random.shuffle(neighbours)
            for nei_name in neighbours:
                C_new = get_C_child(parent=C, who=i_agent, where=nodes_dict[nei_name])
                N.tree.append(C_new)

        config_new = get_new_config(N, C, agents_dict, nodes_dict, h_dict, map_dim, params)
        # check_configs(N.order, N.config, config_new)
        if config_new is None:
            continue

        config_new_name = get_config_name(config_new)
        if config_new_name in explored_dict:
            N_known = explored_dict[config_new_name]
            open_list.appendleft(N_known)  # typically helpful
            continue

        # check_configs(N.order, N.config, config_new)

        order, finished = get_order(config_new, N)
        N_new: HighLevelNode = HighLevelNode(
            config=config_new,
            tree=deque([get_C_init()]),
            order=order,
            parent=N,
            finished=finished,
            i=iteration
        )
        open_list.appendleft(N_new)
        explored_dict[N_new.name] = N_new

        # print + render
        runtime = time.time() - start_time
        # print(
        #     f'\r{'*' * 10} | '
        #     f'[{alg_name}] {iteration=: <3} | '
        #     f'finished: {N_new.finished}/{n_agents: <3} | '
        #     f'runtime: {runtime: .2f} seconds | '
        #     f'{len(open_list)=} | '
        #     f'{len(explored_dict)=} | '
        #     f'{len(N.tree)=} | '
        #     f'{'*' * 10}',
        #     end='')
        iteration += 1
        # if to_render and iteration >= 0:
        #     # update curr nodes
        #     for a in N.order:
        #         a.curr_node = N.config[a.name]
        #     # plot the iteration
        #     i_agent = agents[0]
        #     plot_info = {
        #         'img_np': img_np,
        #         'agents': agents,
        #         'i_agent': i_agent,
        #     }
        #     plot_step_in_env(ax[0], plot_info)
        #     plt.pause(0.001)
        #     # plt.pause(1)

    return None, {'agents': agents}


def run_lifelong_lacam(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict,
) -> Tuple[Dict[str, List[Node]] | None, dict]:
    """
    MAPF:
    - stop condition: all agents at their locations or time is up
    - behaviour, when agent is at its goal: the goal remains the same
    - output: success, time, makespan, soc
    -> LMAPF:
    - stop condition: the end of n iterations where every iteration has a time limit
    - behaviour, when agent is at its goal: agent receives a new goal
    - output: throughput
    """
    n_steps: int = params['n_steps']
    k_limit: int = params['k_limit']
    alg_name: bool = params['alg_name']
    to_render: bool = params['to_render']
    img_np: np.ndarray = params['img_np']

    # stats
    global_start_time = time.time()
    throughput: int = 0

    if to_render:
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))

    # create agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    n_agents = len(agents_dict)

    # main loop
    path_len = 0
    for step_iter in range(n_steps):

        if step_iter > 0 and (step_iter - 1) % k_limit == 0:
            # update goal and throughput
            throughput += update_goal_nodes(agents, nodes)

        if step_iter == path_len:

            for agent in agents:
                agent.k_path = []
            # create k paths
            # agents = get_shuffled_agents(agents)
            random.shuffle(agents)
            solve_k_lacam(
                agents, agents_dict, nodes, nodes_dict, h_dict, map_dim, params
            )
            # append paths
            add_k_paths_to_agents(agents)
            path_len = len(agents[0].path)


        # update curr nodes
        for agent in agents:
            agent.curr_node = agent.path[step_iter]
        # check_vc_ec_neic_iter(agents, step_iter, to_count=False)

        # print
        global_runtime = time.time() - global_start_time
        # print(f'\r[{alg_name}] {n_agents=}, {step_iter=: <3} / {n_steps} | {global_runtime=: .2f} s. | {throughput=}')  # , end=''
        print(f'\r[{alg_name}] {n_agents=}, {step_iter=: <3} / {n_steps} | {global_runtime=: .2f} s. | {throughput=}', end='')  #
        # ------------------------------ #
        # ------------------------------ #
        # ------------------------------ #
        if to_render:
            # plot the iteration
            i_agent = agents_dict['agent_0']
            plot_info = {
                'img_np': img_np,
                'agents': agents,
                'i_agent': i_agent,
                'i': step_iter,
            }
            plot_step_in_env(ax[0], plot_info)
            plt.pause(0.001)
            # plt.pause(1)

    return {a.name: a.path for a in agents}, {'agents': agents, 'throughput': throughput}



@use_profiler(save_dir='../stats/alg_lifelong_lacam.pstat')
def main():

    # to_render = True
    to_render = False

    params = {
        'max_time': 1000,
        'k_limit': 5,
        'alg_name': 'L-LaCAM',
        'to_render': to_render,
        'max_iter_time': 5,  # seconds
        'n_steps': 50,
        # 'w': 0.5, 'd_max': 3, 'gamma': 2,
    }
    run_mapf_alg(alg=run_lifelong_lacam, params=params)


if __name__ == '__main__':
    main()
