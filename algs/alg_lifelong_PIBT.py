from algs.alg_functions_pibt import *
from run_single_MAPF_func import run_mapf_alg


def run_lifelong_pibt(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict
) -> Tuple[None, Dict] | Tuple[Dict[str, List[Node]], Dict]:
    """
    -> LMAPF:
    - stop condition: the end of n iterations where every iteration has a time limit
    - behaviour, when agent is at its goal: agent receives a new goal
    - output: throughput
    """
    max_iter_time: int | float = params['max_iter_time']
    n_steps: int = params['n_steps']
    alg_name: bool = params['alg_name']
    to_render: bool = params['to_render']
    k_limit: bool = params['k_limit']

    start_time = time.time()
    throughput: int = 0

    # create agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    n_agents = len(agents_dict)
    agents.sort(key=lambda a: a.priority, reverse=True)

    for step_iter in range(n_steps):

        if step_iter > 0 and (step_iter - 1) % k_limit == 0:
            # update goal and throughput
            throughput += update_goal_nodes(agents, nodes)

        config_from: Dict[str, Node] = {a.name: a.path[-1] for a in agents}
        occupied_from: Dict[str, AgentAlg] = {a.path[-1].xy_name: a for a in agents}
        config_to: Dict[str, Node] = {}
        occupied_to: Dict[str, AgentAlg] = {}
        pibt_apfs: np.ndarray = init_pibt_apfs_map(map_dim, params)
        # pibt_apfs = init_apfs_map(map_dim, k_limit + 1, params)  # !!!

        # calc the step
        for agent in agents:
            if agent.name not in config_to:
                _ = run_procedure_pibt(
                    agent,
                    config_from, occupied_from,
                    config_to, occupied_to,
                    pibt_apfs, params,
                    agents_dict, nodes_dict, h_dict, [])

        # execute the step + check the termination condition
        agents_finished, agents_unfinished = [], []
        for agent in agents:
            next_node = config_to[agent.name]
            agent.path.append(next_node)
            agent.prev_node = agent.curr_node
            agent.curr_node = next_node
            if agent.curr_node != agent.goal_node:
                agent.priority += 1
                agents_unfinished.append(agent)
            else:
                agent.priority = agent.init_priority
                agents_finished.append(agent)

        # unfinished first
        agents.sort(key=lambda a: a.priority, reverse=True)
        # agents_unfinished.sort(key=lambda a: a.priority, reverse=True)
        # agents = [*agents_finished, *agents_unfinished]

        # throughput += update_goal_nodes(agents, nodes)
        # throughput += get_finished_goals(agents)

        # print + render
        runtime = time.time() - start_time
        print(f'\r[{alg_name}] {step_iter=: <3} | runtime: {runtime: .2f} s. | {throughput=}', end='')

    # checks
    # for i in range(len(agents[0].path)):
    #     check_vc_ec_neic_iter(agents, i, to_count=False)
    return {a.name: a.path for a in agents}, {'agents': agents, 'throughput': throughput}


@use_profiler(save_dir='../stats/alg_lifelong_pibt.pstat')
def main():

    to_render = True
    # to_render = False

    params = {
        'max_iter_time': 5,  # seconds
        'n_steps': 100,
        'alg_name': f'Lifelong-PIBT',
        'to_render': to_render,
        'k_limit': 10,
        # 'w': 0.5, 'd_max': 3, 'gamma': 2,
    }
    run_mapf_alg(alg=run_lifelong_pibt, params=params)


if __name__ == '__main__':
    main()

