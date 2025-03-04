from globals import *


# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# PLOT FUNCS
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #
# -------------------------------------------------------------------------------------------------------------------- #


def get_marker_line(alg_name: str):
    if alg_name not in mrc_dict or 'marker-line' not in mrc_dict[alg_name]:
        marker_line = ''
        if 'APF' in alg_name:
            marker_line += '--'
        else:
            marker_line += '-'
        marker_line += next(markers_iter)
        if alg_name not in mrc_dict:
            mrc_dict[alg_name] = {'marker-line': marker_line}
        else:
            mrc_dict[alg_name]['marker-line'] = marker_line
        return marker_line
    return mrc_dict[alg_name]['marker-line']


def get_alg_color(alg_name: str):
    if alg_name not in mrc_dict or 'color' not in mrc_dict[alg_name]:
        color = next(colors_iter)
        if alg_name not in mrc_dict:
            mrc_dict[alg_name] = {'color': color}
        else:
            mrc_dict[alg_name]['color'] = color
        return color
    return mrc_dict[alg_name]['color']


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


def plot_apfs(ax, info):
    ax.cla()
    if 'apfs_np' not in info or info['apfs_np'] is None:
        return
    # nodes = info['nodes']
    # a_name = info['i_agent'].name if 'i_agent' in info else 'agent_0'
    img_np = info['img_np']
    agents = info['agents']
    apfs_np = info['apfs_np']
    i = info['i']

    field = img_np * -1
    field -= apfs_np[:, :, i]
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

    title_str = 'plot step in apfs_np\n'
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

    for alg_name in alg_names:
        sr_list = []
        x_list = []
        for n_a in n_agents_list:
            if len(info[alg_name][f'{n_a}']['sr']) > 0:
                sr_list.append(np.sum(info[alg_name][f'{n_a}']['sr']) / len(info[alg_name][f'{n_a}']['sr']))
                x_list.append(n_a)
        label = '' if 'APF' in alg_name else f'{alg_name}'
        ax.plot(x_list, sr_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=label, linewidth=4, markersize=15)
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_ylim([0, 1 + 0.1])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=27)
    ax.set_ylabel('Success Rate', fontsize=27)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    # set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.', size=11)
    set_plot_title(ax, f'{img_dir[:-4]}',
                   size=30)
    labelsize = 20
    ax.xaxis.set_tick_params(labelsize=labelsize)
    ax.yaxis.set_tick_params(labelsize=labelsize)
    # set_legend(ax, size=23)
    plt.tight_layout()



def plot_time_metric(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    # x_list = n_agents_list[:4]
    x_list = n_agents_list
    for alg_name in alg_names:
        soc_list = []
        res_str = ''
        for n_a in x_list:
            soc_list.append(np.mean(info[alg_name][f'{n_a}']['time']))
            res_str += f'\t{n_a} - {soc_list[-1]: .2f}, '
        ax.plot(x_list, soc_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=f'{alg_name}', linewidth=4, markersize=15)
        # print(f'{alg_name}\t\t\t: {res_str}')
    ax.set_xlim([min(x_list) - 20, max(x_list) + 20])
    ax.set_xticks(x_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('Runtime', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=11)
    set_legend(ax, size=12)


def plot_time_metric_cactus(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    # x_list = n_agents_list[:4]
    x_list = n_agents_list
    for alg_name in alg_names:
        rt_list = []
        # res_str = ''
        for n_a in x_list:
            rt_list.extend(info[alg_name][f'{n_a}']['time'])
            # res_str += f'\t{n_a} - {rt_list[-1]: .2f}, '
        rt_list.sort()
        label = '' if 'APF' in alg_name else f'{alg_name}'
        ax.plot(rt_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=label, linewidth=4, markersize=15)
        # print(f'{i_alg}\t\t\t: {res_str}')
    # ax.set_xlim([min(x_list) - 20, max(x_list) + 20])
    # ax.set_xticks(x_list)
    ax.set_xlabel('Solved Instances', fontsize=27)
    ax.set_ylabel('Runtime', fontsize=27)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} | time limit: {max_time} sec.',
                   size=24)
    labelsize = 20
    ax.xaxis.set_tick_params(labelsize=labelsize)
    ax.yaxis.set_tick_params(labelsize=labelsize)
    # set_legend(ax, size=25)


def plot_makespan(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']
    i_problems = info['i_problems']

    for alg_name in alg_names:
        makespan_list = []
        x_list = []
        for n_a in n_agents_list:
            if len(info[alg_name][f'{n_a}']['soc']) < i_problems:
                break
            x_list.append(n_a)
            makespan_list.append(np.mean(info[alg_name][f'{n_a}']['makespan']))
        ax.plot(x_list, makespan_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=f'{alg_name}', linewidth=4, markersize=15)
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('Makespan', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=10)
    # set_legend(ax, size=12)


def plot_makespan_cactus(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    for alg_name in alg_names:
        y_list = []
        # res_str = ''
        for n_a in n_agents_list:
            y_list.extend(info[alg_name][f'{n_a}']['makespan'])
            # res_str += f'\t{n_a} - {y_list[-1]: .2f}, '
        y_list.sort()
        ax.plot(y_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=f'{alg_name}', linewidth=2, markersize=10)
        # print(f'{i_alg}\t\t\t: {res_str}')
    # ax.set_xlim([min(x_list) - 20, max(x_list) + 20])
    # ax.set_xticks(x_list)
    ax.set_xlabel('Solved Instances', fontsize=15)
    ax.set_ylabel('Makespan', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=11)
    # set_legend(ax, size=12)


def plot_soc(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']
    i_problems = info['i_problems']

    for alg_name in alg_names:
        soc_list = []
        x_list = []
        for n_a in n_agents_list:
            if len(info[alg_name][f'{n_a}']['soc']) < i_problems:
                break
            x_list.append(n_a)
            soc_list.append(np.mean(info[alg_name][f'{n_a}']['soc']))
        ax.plot(x_list, soc_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=f'{alg_name}', linewidth=4, markersize=15)
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=15)
    ax.set_ylabel('SoC', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=10)
    # set_legend(ax, size=12)


def plot_soc_cactus(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    for alg_name in alg_names:
        y_list = []
        # res_str = ''
        for n_a in n_agents_list:
            y_list.extend(info[alg_name][f'{n_a}']['soc'])
            # res_str += f'\t{n_a} - {y_list[-1]: .2f}, '
        y_list.sort()
        ax.plot(y_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=f'{alg_name}', linewidth=2, markersize=10)
        # print(f'{i_alg}\t\t\t: {res_str}')
    # ax.set_xlim([min(x_list) - 20, max(x_list) + 20])
    # ax.set_xticks(x_list)
    ax.set_xlabel('Solved Instances', fontsize=15)
    ax.set_ylabel('SoC', fontsize=15)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
                   size=11)
    # set_legend(ax, size=12)


def plot_throughput(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    n_steps = info['n_steps']

    for alg_name in alg_names:
        throughput_list = []
        for n_a in n_agents_list:
            throughput_list.append(np.mean(info[alg_name][f'{n_a}']['throughput']))
            # throughput_list.append(np.mean(info[alg_name][f'{n_a}']['n_closed_goals']))
        label = '' if 'APF' in alg_name else f'{alg_name}'
        ax.plot(n_agents_list, throughput_list, get_marker_line(alg_name), color=get_alg_color(alg_name),
                alpha=0.5, label=label, linewidth=4, markersize=15)
    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=27)
    ax.set_ylabel('Throughput', fontsize=27)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    # set_plot_title(ax, f'{img_dir[:-4]} | n_steps: {n_steps}',
    set_plot_title(ax, f'{img_dir[:-4]}',
                   size=27)
    labelsize = 20
    ax.xaxis.set_tick_params(labelsize=labelsize)
    ax.yaxis.set_tick_params(labelsize=labelsize)
    set_legend(ax, size=20)


def plot_rsoc(ax, info):
    ax.cla()
    alg_names = info['alg_names']
    n_agents_list = info['n_agents_list']
    img_dir = info['img_dir']
    max_time = info['max_time']

    for alg_without_apfs, alg_with_apfs in [
        ('k-PrP-A*', 'APF-k-PrP-A*'),
        ('k-PrP-SIPPS', 'APF-k-PrP-SIPPS'),
        ('k-LNS2-A*', 'APF-k-LNS2-A*'),
        ('k-LNS2-SIPPS', 'APF-k-LNS2-SIPPS'),
        ('PIBT', 'APF-PIBT'),
        ('LaCAM', 'APF-LaCAM'),
        ('LaCAM*', 'APF-LaCAM*'),
    ]:
        soc_with_apfs_list, soc_without_apfs_list = [], []
        x_list = []
        for n_a in n_agents_list:
            if len(info[alg_with_apfs][f'{n_a}']['soc']) > 3:
                x_list.append(n_a)
                soc_with_apfs_list.append(np.mean(info[alg_with_apfs][f'{n_a}']['soc']))
                soc_without_apfs_list.append(np.mean(info[alg_without_apfs][f'{n_a}']['soc']))
        soc_with_apfs_list = np.array(soc_with_apfs_list)
        soc_without_apfs_list = np.array(soc_without_apfs_list)
        y_list = soc_with_apfs_list / soc_without_apfs_list
        ax.plot(x_list, y_list, get_marker_line(alg_without_apfs), color=get_alg_color(alg_without_apfs),
                alpha=0.5, label=f'{alg_without_apfs}', linewidth=4, markersize=15)

        # print
        print(f'{alg_with_apfs}')
        for n_a, y_val in zip(n_agents_list, y_list):
            print(f'{n_a} -> {y_val: .2f}')

    ax.set_xlim([min(n_agents_list) - 20, max(n_agents_list) + 20])
    # ax.set_ylim([0, 1 + 0.1])
    ax.set_ylim([0, 1 + 0.2])
    ax.set_xticks(n_agents_list)
    ax.set_xlabel('N agents', fontsize=27)
    ax.set_ylabel('Average RSoC', fontsize=27)
    # ax.set_title(f'{img_dir[:-4]} Map | time limit: {time_to_think_limit} sec.')
    # set_plot_title(ax, f'{img_dir[:-4]} Map | time limit: {max_time} sec.',
    set_plot_title(ax, f'{img_dir[:-4]}',
                   size=30)
    labelsize = 20
    ax.xaxis.set_tick_params(labelsize=labelsize)
    ax.yaxis.set_tick_params(labelsize=labelsize)
    # set_legend(ax, size=21)
