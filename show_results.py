import matplotlib.pyplot as plt

from globals import *
from functions_plotting import *


def show_results(file_dir):
    plt.close()
    with open(f'{file_dir}', 'r') as openfile:
        # Reading from json file
        logs_dict = json.load(openfile)
        expr_type = logs_dict['expr_type']

        if expr_type == 'MAPF':
            fig, ax = plt.subplots(2, 2, figsize=(8, 8))

            plot_sr(ax[0, 0], info=logs_dict)
            # plot_time_metric(ax[0, 1], info=logs_dict)
            plot_time_metric_cactus(ax[0, 1], info=logs_dict)
            plot_makespan(ax[1, 0], info=logs_dict)
            # plot_makespan_cactus(ax[1, 0], info=logs_dict)
            plot_soc(ax[1, 1], info=logs_dict)
            # plot_soc_cactus(ax[1, 1], info=logs_dict)

        if expr_type == 'LMAPF':
            logs_dict['alg_names'] = [
                "L-LNS2-A*",
                "APF-L-LNS2-A*",
                "L-LNS2-SIPPS",
                "APF-L-LNS2-SIPPS",
                "L-PIBT",
                "L-PrP-A*",
                "APF-L-PrP-A*",
                "L-PrP-SIPPS",
                "APF-L-PrP-SIPPS",
                # "APF-L-PIBT",
                # "L-LaCAM"
            ]
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))

            plot_throughput(ax, info=logs_dict)

        plt.tight_layout()
        plt.show()


def main():
    # file_dir = '2023-11-24--16-29_ALGS-4_RUNS-15_MAP-empty-32-32.json'
    # show_results(file_dir=f'logs_for_plots/{file_dir}')

    # LMAPF
    # file_dir = 'LMAPF_ALL_ALGS-10_RUNS-15_MAP-empty-32-32.json'
    # file_dir = 'LMAPF_ALL_ALGS-10_RUNS-15_MAP-random-32-32-10.json'
    # file_dir = 'LMAPF_ALL_ALGS-10_RUNS-15_MAP-random-32-32-20.json'
    # file_dir = 'LMAPF_ALL_ALGS-10_RUNS-15_MAP-room-32-32-4.json'

    # MAPF
    # file_dir = 'MAPF_ALL_K_ALGS_RUNS-15_MAP-empty-32-32.json'
    # file_dir = 'MAPF_ALL_K_ALGS_RUNS-15_MAP-random-32-32-10.json'
    # file_dir = 'MAPF_ALL_K_ALGS_RUNS-15_MAP-random-32-32-20.json'
    file_dir = 'MAPF_ALL_K_ALGS_RUNS-15_MAP-room-32-32-4.json'

    # parameters

    # show_results(file_dir=f'logs_for_experiments/{file_dir}')
    show_results(file_dir=f'final_logs/{file_dir}')


if __name__ == '__main__':
    main()
