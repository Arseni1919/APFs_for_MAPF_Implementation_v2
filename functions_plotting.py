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
        title_str += f'(iteration: {i + 1})'
    title_str += f'{len(agents)} agents '
    ax.set_title(title_str)