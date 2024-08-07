from globals import *
from functions_general import *
from functions_plotting import *

from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from algs.alg_lifelong_PrP import run_lifelong_prp
from algs.alg_lifelong_LNS2 import run_lifelong_LNS2
from algs.alg_lifelong_PIBT import run_lifelong_pibt


# ------------------------------------------------------------------------------------------------------------ #
# General
# ------------------------------------------------------------------------------------------------------------ #
"""
A*: w=0.5, d_max = 4, gamma = 2
SIPPS: w = 0.1, d_max = 3, gamma = 3
PIBT: w = 0.1, d_max = 2, gamma = 1.1
"""

alg_list_general = [
        (run_lifelong_prp, {
            'alg_name': f'L-PrP-SIPPS',
            'constr_type': 'hard',
            'pf_alg': run_sipps,
            'pf_alg_name': 'sipps',
            'k_limit': 5,
            'to_render': False,
        }),
        (run_lifelong_prp, {
            'alg_name': f'L-PrP-A*',
            'constr_type': 'hard',
            'pf_alg': run_temporal_a_star,
            'pf_alg_name': 'a_star',
            'k_limit': 5,
            'to_render': False,
        }),
        (run_lifelong_LNS2, {
            'alg_name': f'L-LNS2-SIPPS',
            'constr_type': 'soft',
            'k_limit': 5,
            'n_neighbourhood': 5,
            'pf_alg_name': 'sipps',
            'pf_alg': run_sipps,
            'to_render': False,
        }),
        (run_lifelong_LNS2, {
            'alg_name': f'L-LNS2-A*',
            'constr_type': 'hard',
            'k_limit': 5,
            'n_neighbourhood': 5,
            'pf_alg_name': 'a_star',
            'pf_alg': run_temporal_a_star,
            'to_render': False,
        }),
        (run_lifelong_pibt, {
            'alg_name': f'L-PIBT',
            'to_render': False,
        }),
    ]


# ------------------------------------------------------------------------------------------------------------ #
# APFs in A*
# ------------------------------------------------------------------------------------------------------------ #
"""
A*: w=0.5, d_max = 4, gamma = 2
"""

alg_list_a_star = [
    (run_lifelong_prp, {
        'alg_name': f'L-PrP-A*',
        'constr_type': 'hard',
        'pf_alg': run_temporal_a_star,
        'pf_alg_name': 'a_star',
        'k_limit': 5,
        'to_render': False,
    }),
    (run_lifelong_prp, {
        'alg_name': f'APF-L-PrP-A*',
        'constr_type': 'hard',
        'pf_alg': run_temporal_a_star,
        'pf_alg_name': 'a_star',
        'k_limit': 5,
        'to_render': False,
        'w': 0.5, 'd_max': 4, 'gamma': 2,
    }),
    (run_lifelong_LNS2, {
        'alg_name': f'L-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'alg_name': f'APF-L-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
        'w': 0.5, 'd_max': 4, 'gamma': 2,
    }),
]


alg_list_a_star_params_w = [
    (run_lifelong_LNS2, {
        'alg_name': f'L-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 1), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 2), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 5), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
]


alg_list_a_star_params_d_max = [
    (run_lifelong_LNS2, {
        'alg_name': f'L-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 1), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 2), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 5), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    # (run_lifelong_LNS2, {
    #         'k_limit': 5,
    #         'w': (w := 0.5), 'd_max': (d_max := 10), 'gamma': (gamma := 2),
    #         'alg_name': f'{w=},{d_max=},{gamma=}',
    #         'constr_type': 'hard',
    #         'n_neighbourhood': 5,
    #         'pf_alg_name': 'a_star',
    #         'pf_alg': run_temporal_a_star,
    #         'to_render': False,
    #     }),
]


alg_list_a_star_params_gamma = [
    (run_lifelong_LNS2, {
        'alg_name': f'L-LNS2-A*',
        'constr_type': 'hard',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 1),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 1.1),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 1.5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 2.5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
(run_lifelong_LNS2, {
        'k_limit': 5,
        'w': (w := 0.5), 'd_max': (d_max := 4), 'gamma': (gamma := 3),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'constr_type': 'hard',
        'n_neighbourhood': 5,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
]


# ------------------------------------------------------------------------------------------------------------ #
# APFs in SIPPS
# ------------------------------------------------------------------------------------------------------------ #
"""
SIPPS: w = 0.1, d_max = 3, gamma = 3
"""

alg_list_sipps = [
    (run_lifelong_prp, {
        'alg_name': f'L-PrP-SIPPS',
        'constr_type': 'hard',
        'pf_alg': run_sipps,
        'pf_alg_name': 'sipps',
        'k_limit': 5,
        'to_render': False,
    }),
    (run_lifelong_prp, {
        'alg_name': f'APF-L-PrP-SIPPS',
        'constr_type': 'hard',
        'pf_alg': run_sipps,
        'pf_alg_name': 'sipps',
        'k_limit': 5,
        'to_render': False,
        'w': 0.1, 'd_max': 3, 'gamma': 3,
    }),
    (run_lifelong_LNS2, {
        'alg_name': f'L-LNS2-SIPPS',
        'constr_type': 'soft',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
    }),
    (run_lifelong_LNS2, {
        'alg_name': f'APF-L-LNS2-SIPPS',
        'constr_type': 'soft',
        'k_limit': 5,
        'n_neighbourhood': 5,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
        'w': 0.1, 'd_max': 3, 'gamma': 3,
    }),
]


alg_list_sipps_params_w = [
    (run_lifelong_LNS2, {
        'k_limit': (k_limit := 5),
        'alg_name': f'L-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 7
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 1
    (run_lifelong_LNS2, {
        'w': (w := 0.5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 2
    (run_lifelong_LNS2, {
        'w': (w := 1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 3
    (run_lifelong_LNS2, {
        'w': (w := 3), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 4
    (run_lifelong_LNS2, {
        'w': (w := 5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 5
    (run_lifelong_LNS2, {
        'w': (w := 7), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 6
    (run_lifelong_LNS2, {
        'w': (w := 10), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
]


alg_list_sipps_params_d_max = [
    (run_lifelong_LNS2, {
        'k_limit': (k_limit := 5),
        'alg_name': f'L-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 1
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 1), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 2
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 3
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 4
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 5
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 5), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
]


alg_list_sipps_params_gamma = [
    (run_lifelong_LNS2, {
        'k_limit': (k_limit := 5),
        'alg_name': f'L-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 1
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 1),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 2
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 1.1),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 3
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 1.5),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 4
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 5
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 3),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 6
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 5),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 7
    (run_lifelong_LNS2, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 7),
        'k_limit': (k_limit := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
]


# ------------------------------------------------------------------------------------------------------------ #
# APFs in PIBT
# ------------------------------------------------------------------------------------------------------------ #
"""
PIBT: w = 0.1, d_max = 2, gamma = 1.1
"""

alg_list_pibt = [
    (run_lifelong_pibt, {
        'alg_name': f'L-PIBT',
        'to_render': False,
        'k_limit': 5,
    }),
    # (run_lifelong_pibt, {
    #     'alg_name': f'APF-L-PIBT',
    #     'to_render': False,
    #     'k_limit': 5,
    #     'w': 0.1, 'd_max': 2, 'gamma': 1.1,
    # }),
]


alg_list_pibt_params_w = [
    (run_lifelong_pibt, {
        'alg_name': f'L-PIBT',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 0.5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 2), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 10), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
]


alg_list_pibt_params_d_max = [
    (run_lifelong_pibt, {
        'alg_name': f'L-PIBT',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 1), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 4), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
(run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 5), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
]


alg_list_pibt_params_gamma = [
    (run_lifelong_pibt, {
        'alg_name': f'L-PIBT',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 1),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 1.1),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 1.5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 2),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 3),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'w': (w := 0.1), 'd_max': (d_max := 2), 'gamma': (gamma := 5),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'to_render': False,
    }),
]



