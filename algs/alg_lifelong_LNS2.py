from algs.alg_functions_LNS2 import *
from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from run_single_MAPF_func import run_mapf_alg


def solve_k_LNS2(
        agents: List[AgentLNS2],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        vc_empty_np, ec_empty_np, pc_empty_np,
        params: dict,
) -> None:
    alg_name: bool = params['alg_name']
    k_limit: int = params['k_limit']
    n_neighbourhood: bool = params['n_neighbourhood']
    max_iter_time: int = params['max_iter_time']
    pf_alg = params['pf_alg']
    pf_alg_name: str = params['pf_alg_name']
    iter_start_time = time.time()

    # init solution
    create_k_limit_init_solution(
        agents, nodes, nodes_dict, h_dict, map_dim, pf_alg_name, pf_alg, k_limit, iter_start_time,
        vc_empty_np, ec_empty_np, pc_empty_np
    )
    cp_graph, cp_graph_names = get_k_limit_cp_graph(agents)
    cp_len = len(cp_graph)
    occupied_from: Dict[str, AgentLNS2] = {a.curr_node.xy_name: a for a in agents}

    # repairing procedure
    lns_iter = 0
    while cp_len > 0 and time.time() - iter_start_time <= max_iter_time:
        lns_iter += 1

        runtime = time.time() - iter_start_time
        print(f'\r[{alg_name}] {lns_iter=}, {cp_len=}, {runtime=: .2f} s. ', end='')

        agents_subset: List[AgentLNS2] = get_k_limit_agents_subset(
            cp_graph, cp_graph_names, n_neighbourhood, agents, occupied_from, h_dict
        )
        old_paths: Dict[str, List[Node]] = {a.name: a.k_path[:] for a in agents_subset}
        agents_outer: List[AgentLNS2] = [a for a in agents if a not in agents_subset]

        solve_k_limit_subset_with_prp(
            agents_subset, agents_outer, nodes, nodes_dict, h_dict, map_dim, iter_start_time,
            pf_alg_name, pf_alg, vc_empty_np, ec_empty_np, pc_empty_np, k_limit, agents,
        )

        old_cp_graph, old_cp_graph_names = cp_graph, cp_graph_names
        cp_graph, cp_graph_names = get_k_limit_cp_graph(agents_subset, agents_outer, cp_graph)
        if len(cp_graph) > cp_len:
            for agent in agents_subset:
                agent.k_path = old_paths[agent.name]
            cp_graph, cp_graph_names = old_cp_graph, old_cp_graph_names
            continue
        cp_len = len(cp_graph)

    if cp_len > 0:
        # time is up -> repair policy
        repair_agents_k_paths(agents, k_limit)
    return


def run_lifelong_LNS2(
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
    agents, agents_dict = create_lns_agents(start_nodes, goal_nodes)
    vc_empty_np, ec_empty_np, pc_empty_np = init_constraints(map_dim, k_limit + 1)

    # main loop
    path_len = 0
    for step_iter in range(n_steps):

        if step_iter == path_len:
            for agent in agents:
                agent.k_path = []
            # create k paths
            solve_k_LNS2(
                agents, nodes, nodes_dict, h_dict, map_dim, vc_empty_np, ec_empty_np, pc_empty_np, params
            )
            # append paths
            add_k_paths_to_agents(agents)
            path_len = len(agents[0].path)

        # update curr nodes
        for agent in agents:
            agent.curr_node = agent.path[step_iter]
        # check
        # check_vc_ec_neic_iter(agents, step_iter, to_count=False)

        # update goal and throughput
        throughput += update_goal_nodes(agents, nodes)

        # print
        global_runtime = time.time() - global_start_time
        print(f'\r[{alg_name}] {step_iter=: <3} / {n_steps} | {global_runtime=: .2f} s. | {throughput=}')  # , end=''
        # ------------------------------ #
        # ------------------------------ #
        # ------------------------------ #
        if to_render:
            # plot the iteration
            i_agent = agents[0]
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


@use_profiler(save_dir='../stats/alg_lifelong_LNS2.pstat')
def main():
    # to_render = True
    to_render = False

    n_neighbourhood: int = 5
    # n_neighbourhood: int = 10

    k_limit: int = 5
    # k_limit: int = 20

    # --------------------------------------------------------------------- #
    # Lifelong-LNS - A*
    # --------------------------------------------------------------------- #
    params_lifelong_lns_a_star = {
        'max_iter_time': 5,  # seconds
        'n_steps': 50,
        'alg_name': f'Lifelong-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': k_limit,
        'n_neighbourhood': n_neighbourhood,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': to_render,
    }
    run_mapf_alg(alg=run_lifelong_LNS2, params=params_lifelong_lns_a_star)
    # --------------------------------------------------------------------- #

    # --------------------------------------------------------------------- #
    # Lifelong-LNS - SIPPS
    # --------------------------------------------------------------------- #
    params_lifelong_lns_sipps = {
        'max_iter_time': 5,  # seconds
        'n_steps': 50,
        'alg_name': f'Lifelong-LNS2-SIPPS',
        'constr_type': 'soft',
        'k_limit': k_limit,
        'n_neighbourhood': n_neighbourhood,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': to_render,
    }
    run_mapf_alg(alg=run_lifelong_LNS2, params=params_lifelong_lns_sipps)
    # --------------------------------------------------------------------- #


if __name__ == '__main__':
    main()







