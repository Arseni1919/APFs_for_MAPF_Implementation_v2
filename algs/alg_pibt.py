from algs.alg_pibt_funcitons import *
from run_single_MAPF_func import run_mapf_alg


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


def run_pibt(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict
) -> Tuple[None, Dict] | Tuple[Dict[str, List[Node]], Dict]:

    max_time = params['max_time']

    start_time = time.time()

    # create agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    n_agents = len(agents_dict)
    agents.sort(key=lambda a: a.priority, reverse=True)

    iteration = 0
    finished = False
    while not finished:

        config_from: Dict[str, Node] = {a.name: a.path[-1] for a in agents}
        occupied_from: Dict[str, AgentAlg] = {a.path[-1].xy_name: a for a in agents}
        config_to: Dict[str, Node] = {}
        occupied_to: Dict[str, AgentAlg] = {}


        # calc the step
        for agent in agents:
            if agent.name not in config_to:
                _ = run_procedure_pibt(
                    agent,
                    config_from, occupied_from,
                    config_to, occupied_to,
                    agents_dict, nodes_dict, h_dict, [])

        # execute the step + check the termination condition
        finished = True
        agents_finished = []
        for agent in agents:
            next_node = config_to[agent.name]
            agent.path.append(next_node)
            agent.prev_node = agent.curr_node
            agent.curr_node = next_node
            if agent.curr_node != agent.goal_node:
                finished = False
                agent.priority += 1
            else:
                agent.priority = agent.init_priority
                agents_finished.append(agent)

        # unfinished first
        agents.sort(key=lambda a: a.priority, reverse=True)

        # print + render
        runtime = time.time() - start_time
        print(f'\r{'*' * 10} | [PIBT] {iteration=: <3} | finished: {len(agents_finished)}/{n_agents: <3} | runtime: {runtime: .2f} seconds | {'*' * 10}', end='')
        iteration += 1

        if runtime > max_time:
            return None, {}

    # checks
    # for i in range(len(agents[0].path)):
    #     check_vc_ec_neic_iter(agents, i, to_count=False)
    runtime = time.time() - start_time
    return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': iteration}


def run_lifelong_pibt():
    pass


@use_profiler(save_dir='../stats/alg_pibt.pstat')
def main():
    params = {'max_time': 100}
    run_mapf_alg(alg=run_pibt, params=params)


if __name__ == '__main__':
    main()

