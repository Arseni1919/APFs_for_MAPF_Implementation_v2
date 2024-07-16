from globals import *
from functions_general import *
from functions_plotting import *

from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from algs.alg_PrP import run_prp
from algs.alg_LNS2 import run_lns2
from algs.alg_pibt import run_pibt
from algs.alg_lacam import run_lacam
from algs.alg_lacam_star import run_lacam_star


@use_profiler(save_dir='stats/alg_mapf_experiments.pstat')
def run_mapf_experiments():
    # ------------------------------------------------------------------------------------------------------------ #
    # General params
    # ------------------------------------------------------------------------------------------------------------ #
    # set_seed(random_seed_bool=False, seed=381)
    # set_seed(random_seed_bool=False, seed=9256)  # 500 - room
    set_seed(random_seed_bool=False, seed=1112)
    # set_seed(random_seed_bool=True)

    # ------------------------------------------------------------------------------------------------------------ #
    # MAPF
    # ------------------------------------------------------------------------------------------------------------ #
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

    # n_agents_list = [400]
    n_agents_list = [50, 100, 150]
    # n_agents_list = [100, 200, 300, 400]
    # n_agents_list = [100, 200, 300, 400, 500]
    # n_agents_list = [200, 300, 400, 500, 600]
    # n_agents_list = [300, 400, 500, 600, 700]

    i_problems = 5

    alg_list = [
        # (AlgPIBT, {'name': 'PIBT'}),
        (run_prp, {
            'alg_name': f'PrP-SIPPS',
            'constr_type': 'hard',
            'pf_alg': run_sipps,
            'to_render': False,
        }),
        (run_prp, {
            'alg_name': f'PrP-A*',
            'constr_type': 'hard',
            'pf_alg': run_temporal_a_star,
            'to_render': False,
        }),
        (run_lns2, {
            'alg_name': f'LNS2',
            'constr_type': 'soft',
            'n_neighbourhood': 5,
            'to_render': False,
        }),
        (run_pibt, {
            'alg_name': f'PIBT',
            'to_render': False,
        }),
        (run_lacam, {
            'alg_name': f'LaCAM',
            'to_render': False,
        }),
        (run_lacam_star, {
            'alg_name': f'LaCAM*',
            'flag_star': False,
            'to_render': False,
        }),

        # (AlgCgar3Mapf, {
        #     'with_return_stage': False,
        #     'first_privilege': False,
        #     'name': 'CGA-no-R-no-F'
        # }),
        # (AlgCgar3Mapf, {
        #     'with_return_stage': True,
        #     'first_privilege': False,
        #     'name': 'CGA-R-no-F'
        # }),
    ]

    # limits
    # max_time = 1e7  # seconds
    # max_time = 60  # seconds
    max_time = 10  # seconds
    # debug
    # to_assert = True
    to_assert = False
    # rendering
    to_render = True
    # to_render = False

    logs_dict = {
        params['alg_name']: {
            f'{n_agents}': {
                'soc': [],
                'makespan': [],
                'sr': [],
                'time': [],
            } for n_agents in n_agents_list
        } for alg, params in alg_list
    }
    logs_dict['alg_names'] = [params['alg_name'] for alg, params in alg_list]
    logs_dict['n_agents_list'] = n_agents_list
    logs_dict['i_problems'] = i_problems
    logs_dict['img_dir'] = img_dir
    logs_dict['max_time'] = max_time

    # ------------------------------------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------------------------------------ #
    # ------------------------------------------------------------------------------------------------------------ #

    fig, ax = plt.subplots(2, 2, figsize=(8, 8))
    to_continue_dict = {params['alg_name']: True for alg, params in alg_list}

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
                params['max_time'] = max_time
                alg_name = params['alg_name']

                solved, alg_info = False, {}
                if to_continue_dict[alg_name]:
                    # the run
                    paths_dict, alg_info = alg(
                        start_nodes, goal_nodes, nodes, nodes_dict, h_dict, map_dim, params
                    )
                    solved = paths_dict is not None

                if solved:
                    logs_dict[alg_name][f'{n_agents}']['sr'].append(1)
                    logs_dict[alg_name][f'{n_agents}']['time'].append(alg_info['time'])
                    logs_dict[alg_name][f'{n_agents}']['makespan'].append(alg_info['makespan'])
                else:
                    logs_dict[alg_name][f'{n_agents}']['sr'].append(0)
                print(f'\n{n_agents=}, {i_problem=}, {alg_name=}')

            # plot
            plot_sr(ax[0, 0], info=logs_dict)
            plot_time_metric(ax[0, 1], info=logs_dict)
            plot_makespan(ax[1, 1], info=logs_dict)
            plt.pause(0.001)

        # check if solved all the prev problems
        for alg, params in alg_list:
            if sum(logs_dict[params['alg_name']][f'{n_agents}']['sr']) == 0:
                to_continue_dict[params['alg_name']] = False

    print('\n[INFO]: finished BIG MAPF experiments')
    plt.show()

# if to_render:
#     fig, ax = plt.subplots(1, 2, figsize=(14, 7))


if __name__ == '__main__':
    run_mapf_experiments()



