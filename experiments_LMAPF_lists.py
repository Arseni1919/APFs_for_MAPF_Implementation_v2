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


# ------------------------------------------------------------------------------------------------------------ #
# APFs in SIPPS
# ------------------------------------------------------------------------------------------------------------ #


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
        'w': 5, 'd_max': 3, 'gamma': 2,
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
        'w': 5, 'd_max': 3, 'gamma': 2,
    }),
]


# ------------------------------------------------------------------------------------------------------------ #
# APFs in PIBT
# ------------------------------------------------------------------------------------------------------------ #


alg_list_pibt = [
    (run_lifelong_pibt, {
        'alg_name': f'L-PIBT',
        'to_render': False,
    }),
    (run_lifelong_pibt, {
        'alg_name': f'APF-L-PIBT',
        'to_render': False,
        'w': 0.5, 'd_max': 3, 'gamma': 2,
    }),
]



