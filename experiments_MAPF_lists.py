from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from algs.alg_mapf_PrP import run_prp_sipps, run_prp_a_star, run_k_prp
from algs.alg_mapf_LNS2 import run_lns2, run_k_lns2
from algs.alg_mapf_pibt import run_pibt
from algs.alg_mapf_lacam import run_lacam
from algs.alg_mapf_lacam_star import run_lacam_star

# ------------------------------------------------------------------------------------------------------------ #
# General
# ------------------------------------------------------------------------------------------------------------ #

alg_list_general = [
    # ------------------------------------------------ #
    # PrP Family
    # ------------------------------------------------ #
    (run_prp_a_star, {
        'alg_name': f'PrP-A*',
        'constr_type': 'hard',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_prp_sipps, {
        'alg_name': f'PrP-SIPPS',
        'constr_type': 'hard',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
    }),
    (run_k_prp, {
        'alg_name': f'15-PrP-A*',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_k_prp, {
        'alg_name': f'15-PrP-SIPPS',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
    }),
    # ------------------------------------------------ #

    # ------------------------------------------------ #
    # LNS2 Family
    # ------------------------------------------------ #
    (run_lns2, {
        'alg_name': f'LNS2',
        'constr_type': 'soft',
        'n_neighbourhood': 5,
        'to_render': False,
    }),
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'{k_limit}-LNS2({k_limit})-A*',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'{k_limit}-LNS2({k_limit})-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),

    # ------------------------------------------------ #
    # PIBT, LaCAM Family
    # ------------------------------------------------ #
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
]

# ------------------------------------------------------------------------------------------------------------ #
# APFs in A*
# ------------------------------------------------------------------------------------------------------------ #


alg_list_a_star = [
    # ------------------------------------------------ #
    # PrP Family
    # ------------------------------------------------ #
    (run_prp_a_star, {
        'alg_name': f'PrP-A*',
        'constr_type': 'hard',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_prp_a_star, {
        'alg_name': f'APF-PrP-A*',
        'constr_type': 'hard',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
        'w': 0.5, 'd_max': 4, 'gamma': 2
    }),
    (run_k_prp, {
        'alg_name': f'k-PrP-A*',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
    }),
    (run_k_prp, {
        'alg_name': f'APF-k-PrP-A*',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'to_render': False,
        'w': 0.5, 'd_max': 4, 'gamma': 2
    }),
    # ------------------------------------------------ #

    # ------------------------------------------------ #
    # LNS2 Family
    # ------------------------------------------------ #
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'k-LNS2-A*',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'APF-k-LNS2-A*',
        'pf_alg_name': 'a_star',
        'pf_alg': run_temporal_a_star,
        'n_neighbourhood': k_limit,
        'to_render': False,
        'w': 0.5, 'd_max': 4, 'gamma': 2
    }),

]

# ------------------------------------------------------------------------------------------------------------ #
# APFs in SIPPS
# ------------------------------------------------------------------------------------------------------------ #


alg_list_sipps = [
    # ------------------------------------------------ #
    # PrP Family
    # ------------------------------------------------ #
    # (run_prp_sipps, {
    #     'alg_name': f'PrP-SIPPS',
    #     'constr_type': 'hard',
    #     'pf_alg_name': 'sipps',
    #     'pf_alg': run_sipps,
    #     'to_render': False,
    # }),
    # (run_prp_sipps, {
    #     'alg_name': f'APF-PrP-SIPPS',
    #     'constr_type': 'hard',
    #     'pf_alg_name': 'sipps',
    #     'pf_alg': run_sipps,
    #     'to_render': False,
    #     'w': 5, 'd_max': 3, 'gamma': 2,
    # }),
    (run_k_prp, {
        'alg_name': f'k-PrP-SIPPS',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
    }),
    (run_k_prp, {
        'alg_name': f'APF-k-PrP-SIPPS',
        'constr_type': 'hard',
        'k_limit': 15,
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'to_render': False,
        'w': 5, 'd_max': 3, 'gamma': 2,
    }),
    # ------------------------------------------------ #

    # ------------------------------------------------ #
    # LNS2 Family
    # ------------------------------------------------ #
    # (run_lns2, {
    #     'alg_name': f'LNS2',
    #     'constr_type': 'soft',
    #     'n_neighbourhood': 5,
    #     'to_render': False,
    # }),
    # (run_lns2, {
    #     'alg_name': f'APF-LNS2',
    #     'constr_type': 'soft',
    #     'n_neighbourhood': 5,
    #     'to_render': False,
    #     'w': 5, 'd_max': 3, 'gamma': 2,
    # }),
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'k-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'APF-k-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
        'w': 5, 'd_max': 3, 'gamma': 2,
    }),
]

alg_list_sipps_params_w = [
    (run_k_lns2, {
        'k_limit': (k_limit := 15),
        'alg_name': f'k-LNS2-SIPPS',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 1
    (run_k_lns2, {
        'w': (w := 0.5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 2
    (run_k_lns2, {
        'w': (w := 1), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 3
    (run_k_lns2, {
        'w': (w := 3), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 4
    (run_k_lns2, {
        'w': (w := 5), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 5
    (run_k_lns2, {
        'w': (w := 7), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 6
    (run_k_lns2, {
        'w': (w := 10), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
        'alg_name': f'{w=},{d_max=},{gamma=}',
        'pf_alg_name': 'sipps',
        'pf_alg': run_sipps,
        'n_neighbourhood': k_limit,
        'to_render': False,
    }),
    # 7
    (run_k_lns2, {
        'w': (w := 15), 'd_max': (d_max := 3), 'gamma': (gamma := 2),
        'k_limit': (k_limit := 15),
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


alg_list_pibt = [
    # ------------------------------------------------ #
    # PIBT, LaCAM Family
    # ------------------------------------------------ #
    (run_pibt, {
        'alg_name': f'PIBT',
        'to_render': False,
    }),
    (run_pibt, {
        'alg_name': f'APF-PIBT',
        'to_render': False,
        'w': 0.5, 'd_max': 3, 'gamma': 2,
    }),
    (run_lacam, {
        'alg_name': f'LaCAM',
        'to_render': False,
    }),
    (run_lacam, {
        'alg_name': f'APF-LaCAM',
        'to_render': False,
        'w': 0.5, 'd_max': 3, 'gamma': 2,
    }),
    (run_lacam_star, {
        'alg_name': f'LaCAM*',
        'flag_star': False,
        'to_render': False,
    }),
    (run_lacam_star, {
        'alg_name': f'APF-LaCAM*',
        'flag_star': False,
        'to_render': False,
        'w': 0.5, 'd_max': 3, 'gamma': 2,
    }),
]
