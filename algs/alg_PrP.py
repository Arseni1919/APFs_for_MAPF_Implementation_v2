from algs.alg_PrP_functions import *
from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from run_single_MAPF_func import run_mapf_alg


def run_prp(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict,
) -> Tuple[Dict[str, List[Node]] | None, dict]:

    constr_type: str = params['constr_type']
    alg_name: bool = params['alg_name']
    pf_alg = params['pf_alg']
    to_render: bool = params['to_render']
    max_time: bool = params['max_time']

    start_time = time.time()

    # create agents
    agents = []
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentPrP(num, s_node, g_node)
        agents.append(new_agent)

    r_iter = 0
    while time.time() - start_time < max_time:
        # calc paths
        h_priority_agents: List[AgentPrP] = []
        for agent in agents:
            (vc_hard_np, ec_hard_np, pc_hard_np,
             vc_soft_np, ec_soft_np, pc_soft_np) = create_hard_and_soft_constraints(h_priority_agents, map_dim, constr_type)
            new_path, alg_info = pf_alg(
                agent.start_node, agent.goal_node, nodes, nodes_dict, h_dict,
                vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np, agent=agent
            )
            if new_path is None:
                agent.path = None
                break
            agent.path = new_path[:]
            h_priority_agents.append(agent)
            align_all_paths(h_priority_agents)

            # checks
            runtime = time.time() - start_time
            print(f'\r[{alg_name}] {r_iter=: <3} | agents: {len(h_priority_agents): <3} / {len(agents)} | {runtime= : .2f} s.')  # , end=''
            # collisions: int = 0
            # for i in range(len(h_priority_agents[0].path)):
            #     to_count = False if constr_type == 'hard' else True
            #     # collisions += check_vc_ec_neic_iter(h_priority_agents, i, to_count)
            # if collisions > 0:
            #     print(f'{collisions=} | {alg_info['c']=}')

        # return check
        if solution_is_found(agents):
            runtime = time.time() - start_time
            makespan: int = max([len(a.path) for a in agents])
            return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': makespan}

        # reshuffle
        r_iter += 1
        random.shuffle(agents)
        for agent in agents:
            agent.path = []

    return None, {}


def run_k_prp(
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
    # constr_type: str = params['constr_type']
    k_limit: int = params['k_limit']
    alg_name: bool = params['alg_name']
    pf_alg = params['pf_alg']
    to_render: bool = params['to_render']
    max_time: bool = params['max_time']

    start_time = time.time()

    # create agents
    agents: List[AgentPrP] = []
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentPrP(num, s_node, g_node)
        agents.append(new_agent)
    vc_soft_np, ec_soft_np, pc_soft_np = init_constraints(map_dim, k_limit + 1)

    # main loop
    k_iter = 0
    while time.time() - start_time < max_time:

        # calc k paths
        all_good: bool = True
        h_priority_agents: List[AgentPrP] = []
        vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, k_limit + 1)
        for agent in agents:
            new_path, alg_info = pf_alg(
                agent.curr_node, agent.goal_node, nodes, nodes_dict, h_dict,
                vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np,
                flag_k_limit=True, k_limit=k_limit, agent=agent
            )
            if new_path is None:
                all_good = False
                break
            new_path = align_path(new_path, k_limit + 1)
            agent.k_path = new_path[:]
            h_priority_agents.append(agent)
            update_constraints(new_path, vc_hard_np, ec_hard_np, pc_hard_np)


            # checks
            # for i in range(len(h_priority_agents[0].path)):
            #     check_vc_ec_neic_iter(h_priority_agents, i, to_count=False)

        # reset k paths if not good
        if not all_good:
            for agent in agents:
                agent.k_path = []
        else:
            for agent in agents:
                assert len(agent.k_path) == k_limit + 1

        # append paths
        for agent in agents:
            agent.path.extend(agent.k_path[1:])
            if len(agent.path) > 0:
                agent.curr_node = agent.path[-1]

        # print
        runtime = time.time() - start_time
        finished: List[AgentPrP] = [a for a in agents if len(a.path) > 0 and a.path[-1] == a.goal_node]
        print(f'\r[{alg_name}] {k_iter=: <3} | agents: {len(finished): <3} / {len(agents)} | {runtime=: .2f} s.')  # , end=''

        # return check
        if solution_is_found(agents):
            runtime = time.time() - start_time
            makespan: int = max([len(a.path) for a in agents])
            return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': makespan}

        # reshuffle
        k_iter += 1
        agents = get_shuffled_agents(agents)
        for agent in agents:
            agent.k_path = []

    return None, {}


def run_lifelong_prp():
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
    pass


@use_profiler(save_dir='../stats/alg_prp.pstat')
def main():
    to_render = True
    # to_render = False

    # PrP-A*
    params_prp_a_star = {
        'max_time': 1000,
        'alg_name': f'PrP-A*',
        'constr_type': 'hard',
        'pf_alg': run_temporal_a_star,
        'to_render': to_render,
    }

    # PrP-SIPPS
    params_prp_sipps = {
        'max_time': 1000,
        'alg_name': f'PrP-SIPPS',
        'constr_type': 'hard',
        'pf_alg': run_sipps,
        'to_render': to_render,
    }

    # run_mapf_alg(alg=run_prp, params=params_prp_a_star)
    # run_mapf_alg(alg=run_prp, params=params_prp_sipps)

    # k-PrP - A*
    params_k_prp_a_star = {
        'max_time': 1000,
        'alg_name': f'k-PrP-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'pf_alg': run_temporal_a_star,
        'to_render': to_render,
    }

    # k-PrP - SIPPS
    params_k_prp_sipps = {
        'max_time': 1000,
        'alg_name': f'k-PrP-SIPPS',
        'constr_type': 'hard',
        'k_limit': 5,
        'pf_alg': run_sipps,
        'to_render': to_render,
    }

    run_mapf_alg(alg=run_k_prp, params=params_k_prp_sipps)
    # run_mapf_alg(alg=run_k_prp, params=params_k_prp_a_star)


if __name__ == '__main__':
    main()







