from algs.alg_functions_PrP import *
from algs.alg_sipps import run_sipps
from algs.alg_sipps_functions import init_si_table, update_si_table_soft, update_si_table_hard
from algs.alg_temporal_a_star import run_temporal_a_star
from algs.alg_functions_APFs import *
from run_single_MAPF_func import run_mapf_alg


def solve_k_prp(
        agents: List[AgentAlg],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        vc_empty_np, ec_empty_np, pc_empty_np,
        params: dict,
) -> None:
    k_limit: int = params['k_limit']
    max_iter_time: int = params['max_iter_time']
    pf_alg = params['pf_alg']
    pf_alg_name = params['pf_alg_name']
    iter_start_time = time.time()

    # main loop
    r_iter = 0
    while time.time() - iter_start_time <= max_iter_time:
        r_iter += 1

        si_table: Dict[str, List[Tuple[int, int, str]]] = init_si_table(nodes)
        apfs_np = init_apfs_map(map_dim, k_limit + 1, params)
        if pf_alg_name == 'sipps':
            vc_hard_np, ec_hard_np, pc_hard_np = vc_empty_np, ec_empty_np, pc_empty_np
            vc_soft_np, pc_soft_np = vc_empty_np, pc_empty_np
            ec_soft_np = init_ec_table(map_dim, k_limit + 1)
        elif pf_alg_name == 'a_star':
            vc_hard_np, ec_hard_np, pc_hard_np = init_constraints(map_dim, k_limit + 1)
            vc_soft_np, ec_soft_np, pc_soft_np = vc_empty_np, ec_empty_np, pc_empty_np
        else:
            raise RuntimeError('nono')

        # calc k paths
        all_good: bool = True
        h_priority_agents: List[AgentPrP] = []

        for agent in agents:
            new_path, alg_info = pf_alg(
                agent.curr_node, agent.goal_node, nodes, nodes_dict, h_dict,
                vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np,
                flag_k_limit=True, k_limit=k_limit, agent=agent, apfs_np=apfs_np, si_table=si_table
            )
            if new_path is None or len(new_path) == 0:
                # all_good = False
                # break
                new_path = [agent.curr_node]
            new_path = align_path(new_path, k_limit + 1)
            agent.k_path = new_path[:]
            agent.k_apfs = get_k_apfs(new_path, map_dim, k_limit + 1, params)
            h_priority_agents.append(agent)

            # update_apfs_map(new_path, apfs_np, params)
            append_apfs(apfs_np, agent, params)
            if pf_alg_name == 'sipps':
                update_ec_table(new_path, ec_soft_np)
                si_table = update_si_table_hard(new_path, si_table, consider_pc=False)
            elif pf_alg_name == 'a_star':
                update_constraints(new_path, vc_hard_np, ec_hard_np, pc_hard_np)
            else:
                raise RuntimeError('nono')

        # runtime = time.time() - iter_start_time
        # print(f' | {r_iter=}, {runtime=: .2f} s. ', end='')

        repair_agents_k_paths(agents, k_limit)
        return

        # if time is up - save the found solution
        if not all_good and time.time() - iter_start_time > max_iter_time:
            break

        if all_good:
            return

        # reset k paths if not good
        random.shuffle(agents)
        for agent in agents:
            agent.k_path = []

    # repair policy
    repair_agents_k_paths(agents, k_limit)

    # for i in range(k_limit + 1):
    #     check_vc_ec_neic_iter(agents, i, to_count=False)
    return


def run_lifelong_prp(
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
    agents, agents_dict = create_prp_agents(start_nodes, goal_nodes)
    vc_empty_np, ec_empty_np, pc_empty_np = init_constraints(map_dim, k_limit + 1)
    n_agents = len(agents)

    # main loop
    path_len = 0
    for step_iter in range(n_steps):

        if step_iter == path_len:
            for agent in agents:
                agent.k_path = []
            # create k paths
            # agents = get_shuffled_agents(agents)
            random.shuffle(agents)
            solve_k_prp(
                agents, nodes, nodes_dict, h_dict, map_dim, vc_empty_np, ec_empty_np, pc_empty_np, params
            )
            # append paths
            add_k_paths_to_agents(agents)
            path_len = len(agents[0].path)


        # update curr nodes
        for agent in agents:
            agent.curr_node = agent.path[step_iter]
        # check_vc_ec_neic_iter(agents, step_iter, to_count=False)
        # update goal and throughput
        throughput += update_goal_nodes(agents, nodes)

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


@use_profiler(save_dir='../stats/alg_lifelong_prp.pstat')
def main():
    # to_render = True
    to_render = False

    # --------------------------------------------------------------------- #
    # Lifelong-PrP - A*
    # --------------------------------------------------------------------- #
    params_lifelong_prp_a_star = {
        'max_iter_time': 5,  # seconds
        'n_steps': 50,
        'alg_name': f'Lifelong-PrP-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'pf_alg': run_temporal_a_star,
        'pf_alg_name': 'a_star',
        'to_render': to_render,
        # 'w': 0.5, 'd_max': 4, 'gamma': 2,
        # 'w': 2, 'd_max': 5, 'gamma': 2,
    }
    run_mapf_alg(alg=run_lifelong_prp, params=params_lifelong_prp_a_star)
    # --------------------------------------------------------------------- #

    # --------------------------------------------------------------------- #
    # Lifelong-PrP - SIPPS
    # --------------------------------------------------------------------- #
    # params_lifelong_prp_sipps = {
    #     'max_iter_time': 5,  # seconds
    #     'n_steps': 50,
    #     'alg_name': f'Lifelong-PrP-SIPPS',
    #     'constr_type': 'hard',
    #     'k_limit': 5,
    #     'pf_alg': run_sipps,
    #     'pf_alg_name': 'sipps',
    #     'to_render': to_render,
    #     'w': 1, 'd_max': 5, 'gamma': 2,
    # }
    # run_mapf_alg(alg=run_lifelong_prp, params=params_lifelong_prp_sipps)
    # --------------------------------------------------------------------- #


if __name__ == '__main__':
    main()







