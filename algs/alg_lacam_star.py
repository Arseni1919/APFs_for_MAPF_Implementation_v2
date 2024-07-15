import random

from algs.alg_lacam_star_functions import *
from run_single_MAPF_func import run_mapf_alg


def run_lacam_star(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict
) -> Tuple[None, Dict] | Tuple[Dict[str, List[Node]], Dict]:

    max_time: int = params['max_time']
    alg_name: str = params['alg_name']
    flag_star: bool = params['flag_star']
    to_render: bool = params['to_render']
    img_np: np.ndarray = params['img_np']

    if to_render:
        fig, ax = plt.subplots(1, 2, figsize=(14, 7))
        plot_rate = 0.001

    start_time = time.time()

    # create agents
    agents, agents_dict = create_agents(start_nodes, goal_nodes)
    n_agents = len(agents_dict)

    config_start: Dict[str, Node] = {a.name: a.start_node for a in agents}
    config_goal: Dict[str, Node] = {a.name: a.goal_node for a in agents}
    config_goal_name: str = get_config_name(config_goal)

    open_list: Deque[HighLevelNodeStar] = deque()  # stack
    explored_dict: Dict[str, HighLevelNodeStar] = {}   # stack
    N_goal: HighLevelNodeStar | None = None

    init_order = get_init_order(agents)
    N_init: HighLevelNodeStar = HighLevelNodeStar(
        config=config_start, tree=deque([get_C_init()]), order=init_order, parent=None
    )
    open_list.appendleft(N_init)
    explored_dict[N_init.name] = N_init

    iteration = 0
    while len(open_list) > 0 and time_is_good(start_time, max_time):
        N: HighLevelNodeStar = open_list[0]

        if N_goal is None and N.name == config_goal_name:

            N_goal = N
            print(f"\ninitial solution found, cost={N_goal.g}")
            if not flag_star:
                break

        # lower bound check
        if N_goal is not None and N_goal.g <= N.f:
            open_list.popleft()
            continue

        # low-level search end
        if len(N.tree) == 0:
            open_list.popleft()
            continue

        # low-level search
        C: LowLevelNodeStar = N.tree.popleft()  # constraints
        if C.depth < n_agents:
            i_agent = N.order[C.depth]
            v = N.config[i_agent.name]
            neighbours = v.neighbours[:]
            random.shuffle(neighbours)
            for nei_name in neighbours:
                C_new = get_C_child(parent=C, who=i_agent, where=nodes_dict[nei_name])
                N.tree.append(C_new)

        config_new = get_new_config(N, C, agents_dict, nodes_dict, h_dict)
        # check_configs(N.order, N.config, config_new)
        if config_new is None:
            continue

        config_new_name = get_config_name(config_new)
        if config_new_name in explored_dict:
            N_known = explored_dict[config_new_name]
            N.neigh.add(N_known)
            open_list.appendleft(N_known)  # typically helpful
            # rewrite, Dijkstra update
            D: Deque[HighLevelNodeStar] = deque([N])
            while len(D) > 0 and flag_star:
                N_from = D.popleft()
                for N_to in N_from.neigh:
                    g = N_from.g + get_edge_cost(agents, N_from.config, N_to.config)
                    if g < N_to.g:
                        if N_goal is not None and N_to is N_goal:
                            print(f"\ncost update: {N_goal.g:4d} -> {g:4d}")
                        N_to.g = g
                        N_to.f = N_to.g + N_to.h
                        N_to.parent = N_from
                        D.append(N_to)
                        if N_goal is not None and N_to.f < N_goal.g:
                            open_list.appendleft(N_to)
                            # possible improvement:
                            # if random.random() > 0.001:
                            #     open_list.appendleft(N_to)
                            # else:
                            #     open_list.appendleft(N_init)

        else:
            # new configuration
            order, finished = get_order(config_new, N)
            N_new: HighLevelNodeStar = HighLevelNodeStar(
                config=config_new,
                tree=deque([get_C_init()]),
                order=order,
                parent=N,
                g=N.g + get_edge_cost(agents, N.config, config_new),
                h=get_h_value(config_new, h_dict, agents),
                finished=finished
            )
            N.neigh.add(N_new)
            open_list.appendleft(N_new)
            explored_dict[N_new.name] = N_new

        iteration += 1

        # print + render
        runtime = time.time() - start_time
        print(
            f'\r{'*' * 10} | '
            f'[{alg_name}{'*' if flag_star else '-'}] {iteration=: <3} | '
            f'finished: {N.finished}/{n_agents: <3} | '
            f'runtime: {runtime: .2f} seconds | '
            f'{len(open_list)=} | '
            f'{len(explored_dict)=} | '
            f'{len(N.tree)=} | '
            f'{'*' * 10}',
            end='')
        if to_render and iteration > 0:
            # update curr nodes
            for a in N.order:
                a.curr_node = N.config[a.name]
            # plot the iteration
            i_agent = agents[0]
            plot_info = {
                'img_np': img_np,
                'agents': agents,
                'i_agent': i_agent,
            }
            plot_step_in_env(ax[0], plot_info)
            plt.pause(0.001)
            # plt.pause(5)


    if N_goal is not None and len(open_list) == 0:
        print(f"\nreach optimal solution, cost={N_goal.g}")
    elif N_goal is not None:
        print(f"\nsuboptimal solution, cost={N_goal.g}")
    elif len(open_list) == 0:
        print("\ndetected unsolvable instance")
        return None, {'agents': agents}
    else:
        print("\nfailure due to timeout")
        return None, {'agents': agents}

    paths_dict = backtrack(N_goal)
    for a_name, path in paths_dict.items():
        agents_dict[a_name].path = path
    # checks
    # for i in range(len(agents[0].path)):
    #     check_vc_ec_neic_iter(agents, i, to_count=False)
    return paths_dict, {'agents': agents}


@use_profiler(save_dir='../stats/alg_lacam_star.pstat')
def main():

    # flag_star: bool = True
    flag_star: bool = False

    # to_render = True
    to_render = False

    params = {'max_time': 60, 'alg_name': 'LaCAM', 'flag_star': flag_star, 'to_render': to_render}
    run_mapf_alg(alg=run_lacam_star, params=params)


if __name__ == '__main__':
    main()
