from algs.alg_temporal_a_star_functions import *


def run_temporal_a_star(
        start_node: Node,
        goal_node: Node,
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        vc_hard_np: np.ndarray | None,  # x, y, t -> bool (0/1)
        ec_hard_np: np.ndarray | None,  # x, y, x, y, t -> bool (0/1)
        pc_hard_np: np.ndarray | None,  # x, y -> time (int)
        vc_soft_np: np.ndarray | None,  # x, y, t -> bool (0/1)
        ec_soft_np: np.ndarray | None,  # x, y, x, y, t -> bool (0/1)
        pc_soft_np: np.ndarray | None,  # x, y -> time (int)
        max_final_time: int = int(1e10),
        flag_k_limit: bool = False,
        k_limit: int = int(1e10),
        agent=None,
        **kwargs,
) -> Tuple[List[Node] | None, dict]:
    start_time = time.time()
    goal_h_dict: np.ndarray = h_dict[goal_node.xy_name]
    initial_h = int(goal_h_dict[start_node.x, start_node.y])
    start_astr_node = AStarNode(start_node, 0, initial_h)
    open_list: List[AStarNode] = [start_astr_node]
    open_list_names: List[str] = [start_astr_node.xyt_name]
    closed_list_names: List[str] = []
    max_pc_time = np.max(pc_hard_np)
    if max_pc_time > 0:
        max_final_time = max_pc_time
    exploded_limit = max(32 * 32, max_pc_time * 150)
    # exploded_limit = max(1000, max_pc_time * 250)

    iteration: int = 0
    while len(open_list) > 0:
        iteration += 1
        # print(f'\r[{iteration}] {len(open_list)=}, {len(closed_list_names)=}', end='')
        print(f'\r[{iteration}] max_pc {max_pc_time}', end='')
        # exploded
        if max_pc_time > 0 and len(closed_list_names) > exploded_limit:
            runtime = time.time() - start_time
            return None, {'runtime': runtime, 'open_list': open_list, 'closed_list': closed_list_names}
        next_astr_node: AStarNode = heapq.heappop(open_list)
        open_list_names.remove(next_astr_node.xyt_name)
        if next_astr_node.n == goal_node or next_astr_node.t >= k_limit:
            latest_vc_on_node: int = get_latest_vc_on_node(next_astr_node, vc_hard_np)
            if next_astr_node.t > latest_vc_on_node or next_astr_node.t >= k_limit:
                path = reconstruct_path(next_astr_node)
                runtime = time.time() - start_time
                return path, {'runtime': runtime, 'open_list': open_list, 'closed_list': closed_list_names}

        for nei_node in next_astr_node.neighbours_nodes:
            new_t = next_astr_node.t + 1
            nei_astr_name = f'{nei_node.x}_{nei_node.y}_{new_t}'
            if next_astr_node.t >= max_final_time:
                new_t = max_final_time + 1
            if nei_astr_name == next_astr_node.xyt_name:
                continue
            if nei_astr_name in open_list_names:
                continue
            if nei_astr_name in closed_list_names:
                continue
            if new_t < vc_hard_np.shape[-1] and vc_hard_np[nei_node.x, nei_node.y, new_t]:
                continue
            if new_t < ec_hard_np.shape[-1] and ec_hard_np[nei_node.x, nei_node.y, next_astr_node.x, next_astr_node.y, new_t]:
                continue
            pc_value = pc_hard_np[nei_node.x, nei_node.y]
            if pc_value != -1 and new_t >= pc_value:
                continue
            new_h = int(goal_h_dict[nei_node.x, nei_node.y])
            nei_astr_node = AStarNode(nei_node, new_t, new_h, parent=next_astr_node)
            heapq.heappush(open_list, nei_astr_node)
            heapq.heappush(open_list_names, nei_astr_node.xyt_name)
        heapq.heappush(closed_list_names, next_astr_node.xyt_name)

    runtime = time.time() - start_time
    return None, {'runtime': runtime, 'open_list': open_list, 'closed_list': closed_list_names}


@use_profiler(save_dir='../stats/alg_temporal_a_star.pstat')
def main():
    # set_seed(random_seed_bool=False, seed=7310)
    # set_seed(random_seed_bool=False, seed=123)
    set_seed(random_seed_bool=True)

    # img_dir = '10_10_my_rand.map'
    img_dir = 'empty-32-32.map'
    # img_dir = 'random-32-32-10.map'
    # img_dir = 'random-32-32-20.map'
    # img_dir = 'room-32-32-4.map'
    # img_dir = 'maze-32-32-2.map'
    # img_dir = 'maze-32-32-4.map'

    to_render: bool = True
    # to_render: bool = False

    path_to_maps: str = '../maps'
    path_to_heuristics: str = '../logs_for_heuristics'

    img_np, (height, width) = get_np_from_dot_map(img_dir, path_to_maps)
    map_dim = (height, width)
    nodes, nodes_dict = build_graph_from_np(img_np, show_map=False)
    h_dict = exctract_h_dict(img_dir, path_to_heuristics)

    path1 = [
        nodes_dict['7_1'],
        nodes_dict['7_1'],
        nodes_dict['6_1'],
        nodes_dict['5_1'],
        nodes_dict['4_1'],
    ]
    path2 = [
        nodes_dict['6_0'],
        nodes_dict['6_0'],
        # nodes_dict['6_0'],
        # nodes_dict['6_0'],
        # nodes_dict['6_1'],
        # nodes_dict['6_1'],
        # nodes_dict['6_1'],
        # nodes_dict['6_0'],
    ]
    h_paths = [path1, path2]
    max_path_len = max(map(lambda x: len(x), h_paths))
    vc_hard_np, ec_hard_np, pc_hard_np = create_constraints(h_paths, map_dim, max_path_len)

    path = [
        nodes_dict['7_2'],
        nodes_dict['7_2'],
        nodes_dict['6_2'],
        nodes_dict['5_2'],
        nodes_dict['4_2'],
    ]
    s_paths = [path]
    vc_soft_np, ec_soft_np, pc_soft_np = create_constraints(s_paths, map_dim, max_path_len)

    start_node = nodes_dict['6_2']
    goal_node = nodes_dict['25_25']

    result, info = run_temporal_a_star(
        start_node, goal_node, nodes, nodes_dict, h_dict,
        vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np
    )

    # plot
    if to_render:
        print(f'{[n.xy_name for n in result]}')
        plot_np = np.ones(img_np.shape) * -2
        # nodes
        for n in nodes:
            plot_np[n.x, n.y] = 0
        # hard
        for h_path in h_paths:
            for n in h_path:
                plot_np[n.x, n.y] = -1
        # soft
        for s_path in s_paths:
            for n in s_path:
                plot_np[n.x, n.y] = -0.5
        # result
        for n in result:
            plot_np[n.x, n.y] = 1

        plt.imshow(plot_np, cmap='binary', origin='lower')
        plt.show()


if __name__ == '__main__':
    main()


