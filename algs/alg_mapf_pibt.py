import numpy as np

from algs.alg_functions_pibt import *
from run_single_MAPF_func import run_mapf_alg


def run_pibt(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict
) -> Tuple[None, Dict] | Tuple[Dict[str, List[Node]], Dict]:

    max_time: int | float = params['max_time']
    alg_name = params['alg_name']
    to_render: bool = params['to_render']
    img_np: np.ndarray = params['img_np']

    if to_render:
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))

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
        pibt_apfs: np.ndarray = init_pibt_apfs_map(map_dim, params)


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
        print(f'\r{'*' * 10} | [{alg_name}] {iteration=: <3} | finished: {len(agents_finished)}/{n_agents: <3} | runtime: {runtime: .2f} seconds | {'*' * 10}', end='')
        iteration += 1
        if to_render and iteration >= 0:
            # update curr nodes
            for a in agents:
                a.curr_node = config_to[a.name]
            # plot the iteration
            i_agent = agents_dict['agent_0']
            plot_info = {
                'img_np': img_np,
                'agents': agents,
                'i_agent': i_agent,
            }
            plot_step_in_env(ax[0], plot_info)
            plt.pause(0.001)
            # plt.pause(1)

        if runtime > max_time:
            return None, {}

    # checks
    # for i in range(len(agents[0].path)):
    #     check_vc_ec_neic_iter(agents, i, to_count=False)
    runtime = time.time() - start_time
    return {a.name: a.path for a in agents}, {'agents': agents, 'time': runtime, 'makespan': iteration}


@use_profiler(save_dir='../stats/alg_pibt.pstat')
def main():

    # to_render = True
    to_render = False

    params = {
        'max_time': 100,
        'alg_name': 'PIBT',
        'to_render': to_render,
        'w': 0.5, 'd_max': 3, 'gamma': 2,
    }
    run_mapf_alg(alg=run_pibt, params=params)


if __name__ == '__main__':
    main()

