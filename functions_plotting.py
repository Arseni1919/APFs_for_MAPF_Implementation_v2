from globals import *


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# PLOT FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #


def get_color(i):
    index_to_pick = i % len(color_names)
    return color_names[index_to_pick]


def set_plot_title(ax, title, size=9):
    ax.set_title(f'{title}', fontweight="bold", size=size)


def set_legend(ax, framealpha=None, size=9):
    to_put_legend = True
    # to_put_legend = False
    if to_put_legend:
        if not framealpha:
            framealpha = 0
        legend_properties = {'weight': 'bold', 'size': size}
        # legend_properties = {}
        if framealpha is not None:
            ax.legend(prop=legend_properties, framealpha=framealpha)
        else:
            ax.legend(prop=legend_properties)


def plot_step_in_env(ax, info):
    ax.cla()
    # nodes = info['nodes']
    # a_name = info['i_agent'].name if 'i_agent' in info else 'agent_0'
    img_np = info['img_np']
    agents = info['agents']

    field = img_np * -1
    ax.imshow(field, origin='lower', cmap='binary')

    others_y_list, others_x_list, others_cm_list, alpha_list = [], [], [], []
    for agent in agents:
        if 'i_agent' in info and info['i_agent'] == agent:
            continue
        curr_node = agent.curr_node
        others_y_list.append(curr_node.y)
        others_x_list.append(curr_node.x)
        others_cm_list.append(get_color(agent.num))
        if agent.curr_node == agent.goal_node:
            alpha_list.append(0.2)
        else:
            alpha_list.append(1)
    ax.scatter(others_y_list, others_x_list, s=100, c='k', alpha=alpha_list)
    ax.scatter(others_y_list, others_x_list, s=50, c=np.array(others_cm_list), alpha=alpha_list)
    # ax.scatter(others_y_list, others_x_list, s=50, c='yellow')

    if 'i_agent' in info:
        i_agent = info['i_agent']
        curr_node = i_agent.curr_node
        next_goal_node = i_agent.goal_node
        ax.scatter([curr_node.y], [curr_node.x], s=200, c='k')
        ax.scatter([curr_node.y], [curr_node.x], s=100, c='r')
        ax.scatter([next_goal_node.y], [next_goal_node.x], s=400, c='white', marker='X', alpha=0.4)
        ax.scatter([next_goal_node.y], [next_goal_node.x], s=200, c='red', marker='X', alpha=0.4)

    title_str = 'plot_step_in_env\n'
    if 'to_title' in info:
        to_title = info['to_title']
        title_str += f'{to_title}\n '
    if 'img_dir' in info:
        img_dir = info['img_dir']
        title_str += f'Map: {img_dir[:-4]}\n '
    if 'i' in info:
        i = info['i']
        title_str += f'(iteration: {i + 1})\n'
    title_str += f'{len(agents)} agents '
    ax.set_title(title_str)



def plot_sr(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']

    for i_alg in alg_names:
        sr_list = []
        x_list = []
        for n_a in n_agents_list:
            if len(info[i_alg][f'{n_a}']['sr']) > 0:
                sr_list.append(np.sum(info[i_alg][f'{n_a}']['sr']) / len(info[i_alg][f'{n_a}']['sr']))
                x_list.append(n_a)
        ax.plot(x_list, sr_list, markers_lines_dict[i_alg], color=colors_dict[i_alg],
                alpha=0.5, label=f'{i_alg}', linewidth=5, markersize=20)
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_ylim([0, 1 + 0.1])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('Success Rate', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    # set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.', size=11)
    set_plot_title(ax, f'{img_dir[:-4]} Map', size=11)
    set_legend(ax, size=12)
    plt.tight_layout()



def plot_time_metric(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    # x_list = n_agents_list[:4]
    x_list = n_agents_list
    for i_alg in alg_names:
        soc_list = []
        res_str = ''
        for n_a in x_list:
            soc_list.append(np.mean(info[i_alg][f'{n_a}']['time']))
            res_str += f'\t{n_a} - {soc_list[-1]: .2f}, '
        ax.plot(x_list, soc_list, markers_lines_dict[i_alg], color=colors_dict[i_alg],
                alpha=0.5, label=f'{i_alg}', linewidth=5, markersize=20)
        # print(f'{i_alg}\t\t\t: {res_str}')
    ax.set_xlim([min(x_list) - 20, max(x_list) + 20])
    ax.set_xticks(x_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('Runtime', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=11)
    set_legend(ax, size=12)


def plot_makespan(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    for i_alg in alg_names:
        makespan_list = []
        for n_a in n_agents_list:
            makespan_list.append(np.mean(info[i_alg][f'{n_a}']['makespan']))
        ax.plot(n_agents_list, makespan_list, '-^', label=f'{i_alg}')
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('Makespan', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=10)
    set_legend(ax, size=12)