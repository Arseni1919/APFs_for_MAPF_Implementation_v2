from algs.alg_functions_LNS2 import *
from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from run_single_MAPF_func import run_mapf_alg


def run_lns2(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict,
) -> Tuple[Dict[str, List[Node]] | None, dict]:
    """
    To begin with,
    - MAPF-LNS2 calls a MAPF algorithm to solve the instance and obtains a (partial or complete) plan
    from the MAPF algorithm.
    - For each agent that does not yet have a path, MAPF-LNS2 plans a path for it that
    minimizes the number of collisions with the existing paths (Section 4).
    - MAPF-LNS2 then repeats a repairing procedure until the plan P becomes
    feasible.
    - At each iteration, MAPF-LNS2 selects a subset of agents As ⊆ A by a neighborhood selection method (see
    Section 5). We denote the paths of the agents in As as P−.
    - It then calls a modiﬁed MAPF algorithm to replan the paths of the agents in As to minimize
    the number of collisions with each other and with the paths in P \\ P−.
    Speciﬁcally, MAPF-LNS2 uses a modiﬁcation of Prioritized Planning (PP) as the modiﬁed MAPF algorithm.
    PP assigns a random priority ordering to the agents in As and replans their paths one at a time according
    to the ordering. Each time, it calls a single-agent pathﬁnding algorithm (see Section 4) to ﬁnd a path for
    an agent that minimizes the number of collisions with the new paths of the higher-priority agents in As and
    the paths in P \\ P−. We denote the new paths of the agents in As as P+.
    - Finally, MAPF-LNS2 replaces the old plan P with the new plan (P \\ P−) ∪ P+ iff the number of colliding pairs
    (CP) of the paths in the new plan is no larger than that of the old plan.
    """
    alg_name: str = params['alg_name']
    constr_type: str = params['constr_type']
    n_neighbourhood: bool = params['n_neighbourhood']
    to_render: bool = params['to_render']
    max_time: bool = params['max_time']

    start_time = time.time()
    # create agents
    agents, agents_dict = create_lns_agents(start_nodes, goal_nodes)

    # init solution
    create_init_solution(agents, nodes, nodes_dict, h_dict, map_dim, constr_type, start_time)
    cp_graph, cp_graph_names = get_cp_graph(agents)
    cp_len = len(cp_graph)
    occupied_from: Dict[str, AgentLNS2] = {a.start_node.xy_name: a for a in agents}

    # repairing procedure
    while cp_len > 0:
        if time.time() - start_time >= max_time:
            return None, {'agents': agents}
        print(f'\n[{alg_name}] {cp_len=}')
        agents_subset: List[AgentLNS2] = get_agents_subset(cp_graph, cp_graph_names, n_neighbourhood, agents, occupied_from, h_dict)
        old_paths: Dict[str, List[Node]] = {a.name: a.path[:] for a in agents_subset}
        agents_outer: List[AgentLNS2] = [a for a in agents if a not in agents_subset]

        # assert len(set(agents_outer)) == len(agents_outer)
        # assert len(set(agents_subset)) == len(agents_subset)
        # assert len(set(agents)) == len(agents)
        # assert len(agents_subset) + len(agents_outer) == len(agents)

        solve_subset_with_prp(agents_subset, agents_outer, nodes, nodes_dict, h_dict, map_dim, start_time, constr_type, agents)

        old_cp_graph, old_cp_graph_names = cp_graph, cp_graph_names
        # cp_graph, cp_graph_names = get_cp_graph(agents)
        cp_graph, cp_graph_names = get_cp_graph(agents_subset, agents_outer, cp_graph)
        if len(cp_graph) > cp_len:
            for agent in agents_subset:
                agent.path = old_paths[agent.name]
            cp_graph, cp_graph_names = old_cp_graph, old_cp_graph_names
            continue
        cp_len = len(cp_graph)
    # align_all_paths(agents)
    # for i in range(len(agents[0].path)):
    #     check_vc_ec_neic_iter(agents, i)
    runtime = time.time() - start_time
    makespan: int = max([len(a.path) for a in agents])
    return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': makespan}


def run_k_lns2(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict,
) -> Tuple[Dict[str, List[Node]] | None, dict]:
    """
    -> MAPF:
    - stop condition: all agents at their locations or time is up
    - behaviour, when agent is at its goal: the goal remains the same
    - output: success, time, makespan, soc
    LMAPF:
    - stop condition: the end of n iterations where every iteration has a time limit
    - behaviour, when agent is at its goal: agent receives a new goal
    - output: throughput
    """
    alg_name: str = params['alg_name']
    pf_alg_name: str = params['pf_alg_name']
    pf_alg: str = params['pf_alg']
    k_limit: bool = params['k_limit']
    n_neighbourhood: bool = params['n_neighbourhood']
    to_render: bool = params['to_render']
    img_np: np.ndarray = params['img_np']
    max_time: bool = params['max_time']

    if to_render:
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))

    start_time = time.time()
    # create agents
    agents, agents_dict = create_lns_agents(start_nodes, goal_nodes)
    vc_empty_np, ec_empty_np, pc_empty_np = init_constraints(map_dim, k_limit + 1)

    k_iter: int = 0
    while True:
        k_iter += 1

        # ------------------------------ #
        # Solve k steps
        # ------------------------------ #

        # init solution
        create_k_limit_init_solution(
            agents, nodes, nodes_dict, h_dict, map_dim, pf_alg_name, pf_alg, k_limit, start_time,
            vc_empty_np, ec_empty_np, pc_empty_np
        )
        cp_graph, cp_graph_names = get_k_limit_cp_graph(agents)
        cp_len = len(cp_graph)
        occupied_from: Dict[str, AgentLNS2] = {a.curr_node.xy_name: a for a in agents}

        # repairing procedure
        lns_iter = 0
        while cp_len > 0:
            lns_iter += 1
            if time.time() - start_time >= max_time:
                return None, {'agents': agents}

            print(f'\r[{alg_name}] {lns_iter=}, {cp_len=}', end='')
            agents_subset: List[AgentLNS2] = get_k_limit_agents_subset(
                cp_graph, cp_graph_names, n_neighbourhood, agents, occupied_from, h_dict
            )
            old_paths: Dict[str, List[Node]] = {a.name: a.k_path[:] for a in agents_subset}
            agents_outer: List[AgentLNS2] = [a for a in agents if a not in agents_subset]

            solve_k_limit_subset_with_prp(
                agents_subset, agents_outer, nodes, nodes_dict, h_dict, map_dim, start_time,
                pf_alg_name, pf_alg, vc_empty_np, ec_empty_np, pc_empty_np, k_limit, agents
            )

            old_cp_graph, old_cp_graph_names = cp_graph, cp_graph_names
            cp_graph, cp_graph_names = get_k_limit_cp_graph(agents_subset, agents_outer, cp_graph)
            if len(cp_graph) > cp_len:
                for agent in agents_subset:
                    agent.k_path = old_paths[agent.name]
                cp_graph, cp_graph_names = old_cp_graph, old_cp_graph_names
                continue
            cp_len = len(cp_graph)

        # ------------------------------ #
        # ------------------------------ #
        # ------------------------------ #
        if to_render:
            for i in range(k_limit):
                for a in agents:
                    a.curr_node = a.k_path[i]
                # plot the iteration
                i_agent = agents[0]
                plot_info = {
                    'img_np': img_np,
                    'agents': agents,
                    'i_agent': i_agent,
                }
                plot_step_in_env(ax[0], plot_info)
                plt.pause(0.001)
                # plt.pause(1)

        # align_all_paths(agents)
        # for i in range(len(agents[0].path)):
        #     check_vc_ec_neic_iter(agents, i)

        # append paths
        for agent in agents:
            if len(agent.path) == 0:
                agent.path.append(agent.start_node)
            agent.path.extend(agent.k_path[1:])
            if len(agent.path) > 0:
                agent.curr_node = agent.path[-1]

        # print
        runtime = time.time() - start_time
        finished: List[AgentLNS2] = [a for a in agents if len(a.path) > 0 and a.path[-1] == a.goal_node]
        print(f'\r[{alg_name}] {k_iter=: <3} | agents: {len(finished): <3} / {len(agents)} | {runtime=: .2f} s.')  # , end=''

        # return check
        if solution_is_found(agents):
            runtime = time.time() - start_time
            makespan: int = max([len(a.path) for a in agents])
            return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': makespan}

        # reshuffle
        # agents = get_shuffled_agents(agents)
        for agent in agents:
            agent.k_path = []


def run_lifelong_lns2():
    pass


@use_profiler(save_dir='../stats/alg_lns2.pstat')
def main():
    to_render = True
    # to_render = False

    n_neighbourhood: int = 10

    k_limit: int = 5
    # k_limit: int = 20

    params_lns2 = {
        'max_time': 1000,
        'alg_name': 'LNS2',
        'constr_type': 'soft',
        'n_neighbourhood': n_neighbourhood,
        'to_render': to_render,
    }
    # run_mapf_alg(alg=run_lns2, params=params_lns2)

    params_k_lns2_sipps = {
        'max_time': 1000,
        'alg_name': 'k-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'k_limit': k_limit,
        'n_neighbourhood': n_neighbourhood,
        'to_render': to_render,
    }
    run_mapf_alg(alg=run_k_lns2, params=params_k_lns2_sipps)

    params_k_lns2_a_star = {
        'max_time': 1000,
        'alg_name': 'k-LNS2-A*',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'k_limit': k_limit,
        'n_neighbourhood': n_neighbourhood,
        'to_render': to_render,
    }
    # run_mapf_alg(alg=run_k_lns2, params=params_k_lns2_a_star)


if __name__ == '__main__':
    main()
