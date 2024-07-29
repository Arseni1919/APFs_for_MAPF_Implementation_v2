from globals import *
from functions_general import *
from functions_plotting import *

from experiments_LMAPF_lists import *
from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from algs.alg_lifelong_PrP import run_lifelong_prp
from algs.alg_lifelong_LNS2 import run_lifelong_LNS2
from algs.alg_lifelong_PIBT import run_lifelong_pibt


@use_profiler(save_dir='stats/experiments_lifelong_mapf.pstat')
def run_mapf_experiments():
    # ------------------------------------------------------------------------------------------------------------ #
    # General params
    # ------------------------------------------------------------------------------------------------------------ #
    # set_seed(random_seed_bool=False, seed=381)
    # set_seed(random_seed_bool=False, seed=9256)  # 500 - room
    set_seed(random_seed_bool=False, seed=1112)
    # set_seed(random_seed_bool=True)

    # ------------------------------------------------------------------------------------------------------------ #
    # LMAPF
    # ------------------------------------------------------------------------------------------------------------ #
    # ------------------------------------------------- #
    # img_dir = '10_10_my_rand.map'
    # img_dir = '15-15-two-rooms.map'
    # img_dir = '15-15-four-rooms.map'
    # img_dir = '15-15-six-rooms.map'
    # img_dir = '15-15-eight-rooms.map'

    # img_dir = 'empty-32-32.map'
    img_dir = 'random-32-32-10.map'
    # img_dir = 'random-32-32-20.map'
    # img_dir = 'maze-32-32-4.map'
    # img_dir = 'maze-32-32-2.map'
    # img_dir = 'room-32-32-4.map'
    # ------------------------------------------------- #

    # n_agents_list = [400]
    # n_agents_list = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
    # n_agents_list = [100, 200, 300, 400]
    n_agents_list = [50, 100, 150, 200, 250, 300, 350, 400, 450]
    # n_agents_list = [100, 200, 300, 400, 500]
    # n_agents_list = [200, 300, 400, 500, 600]
    # n_agents_list = [300, 400, 500, 600, 700]

    # ------------------------------------------------- #

    i_problems = 5

    # ------------------------------------------------- #

    # limits
    max_iter_time = 5
    # max_iter_time = 10

    n_steps = 50
    # n_steps = 100

    # ------------------------------------------------- #

    # alg_list = alg_list_general
    # alg_list = alg_list_a_star
    # alg_list = alg_list_sipps
    # alg_list = alg_list_pibt

    # SIPPS APFs params
    # alg_list = alg_list_sipps_params_w
    # alg_list = alg_list_sipps_params_d_max
    # alg_list = alg_list_sipps_params_gamma

    # PIBT APFs params
    alg_list = alg_list_pibt_params_w

    # ------------------------------------------------- #

    # debug
    # to_assert = True
    to_assert = False

    # rendering
    to_render = True
    # to_render = False

    # saving
    # to_save = False
    to_save = True

    logs_dict: Dict[str, Any] = {
        params['alg_name']: {
            f'{n_agents}': {
                'throughput': [],
            } for n_agents in n_agents_list
        } for alg, params in alg_list
    }
    logs_dict['alg_names'] = [params['alg_name'] for alg, params in alg_list]
    logs_dict['n_agents_list'] = n_agents_list
    logs_dict['i_problems'] = i_problems
    logs_dict['img_dir'] = img_dir
    logs_dict['max_iter_time'] = max_iter_time
    logs_dict['n_steps'] = n_steps
    logs_dict['expr_type'] = 'LMAPF'

    # ------------------------------------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------------------------------------ #

    fig, ax = plt.subplots(1, 1, figsize=(8, 8))

    path_to_maps: str = 'maps'
    path_to_heuristics: str = 'logs_for_heuristics'

    img_np, (height, width) = get_np_from_dot_map(img_dir, path_to_maps)
    map_dim = (height, width)
    nodes, nodes_dict = build_graph_from_np(img_np, show_map=False)
    h_dict = exctract_h_dict(img_dir, path_to_heuristics)

    for n_agents in n_agents_list:

        for i_problem in range(i_problems):

            start_nodes: List[Node] = random.sample(nodes, n_agents)
            goal_nodes: List[Node] = random.sample(nodes, n_agents)

            for alg, params in alg_list:

                # preps
                params['img_np'] = img_np
                params['max_iter_time'] = max_iter_time
                params['n_steps'] = n_steps
                alg_name = params['alg_name']

                # the run
                paths_dict, alg_info = alg(
                    start_nodes, goal_nodes, nodes, nodes_dict, h_dict, map_dim, params
                )

                logs_dict[alg_name][f'{n_agents}']['throughput'].append(alg_info['throughput'])
                print(f'\n{n_agents=}, {i_problem=}, {alg_name=}, throughput={alg_info['throughput']}')

            # plot
            plot_throughput(ax, info=logs_dict)
            plt.pause(0.001)

    if to_save:
        save_results(logs_dict)

    print('\n[INFO]: finished BIG Lifelong MAPF experiments')
    plt.show()

# if to_render:
#     fig, ax = plt.subplots(1, 2, figsize=(14, 7))


if __name__ == '__main__':
    run_mapf_experiments()



