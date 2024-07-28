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
            plot_time_metric(ax[0, 1], info=logs_dict)
            plot_makespan(ax[1, 1], info=logs_dict)

        if expr_type == 'LMAPF':

            fig, ax = plt.subplots(1, 1, figsize=(8, 8))

            plot_throughput(ax, info=logs_dict)

        plt.show()


def main():
    # file_dir = '2023-11-24--16-29_ALGS-4_RUNS-15_MAP-empty-32-32.json'
    # show_results(file_dir=f'logs_for_plots/{file_dir}')

    # LMAPF

    # MAPF
    file_dir = ''

    # parameters

    show_results(file_dir=f'logs_for_experiments/{file_dir}')


if __name__ == '__main__':
    main()


