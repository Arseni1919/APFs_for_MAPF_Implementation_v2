from algs.alg_LNS2_functions import *
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
    constr_type: bool = params['constr_type']
    n_neighbourhood: bool = params['n_neighbourhood']
    to_render: bool = params['to_render']
    max_time: bool = params['max_time']

    start_time = time.time()
    # create agents
    agents: List[AgentLNS2] = []
    agents_dict: Dict[str, AgentLNS2] = {}
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentLNS2(num, s_node, g_node)
        agents.append(new_agent)
        agents_dict[new_agent.name] = new_agent

    # init solution
    create_init_solution(agents, nodes, nodes_dict, h_dict, map_dim, constr_type, start_time)
    cp_graph, cp_graph_names = get_cp_graph(agents)
    cp_len = len(cp_graph)
    occupied_from: Dict[str, AgentLNS2] = {a.start_node.xy_name: a for a in agents}

    # repairing procedure
    while cp_len > 0 and time.time() - start_time < max_time:
        print(f'\n{cp_len=}')
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
    return {a.name: a.path for a in agents}, {'agents': agents}


@use_profiler(save_dir='../stats/alg_lns2.pstat')
def main():
    # to_render = True
    to_render = False

    # constr_type: str = 'hard'
    constr_type: str = 'soft'

    n_neighbourhood: int = 5

    params = {
        'max_time': 1000,
        'alg_name': 'LNS2',
        'constr_type': constr_type,
        'n_neighbourhood': n_neighbourhood,
        'to_render': to_render,
    }
    run_mapf_alg(alg=run_lns2, params=params)


if __name__ == '__main__':
    main()
