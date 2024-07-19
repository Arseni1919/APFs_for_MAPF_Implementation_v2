from algs.alg_sipps_functions import *


def run_sipps_insert_node(
        node: SIPPSNode,
        Q: List[SIPPSNode],
        P: List[SIPPSNode],
        ident_dict: DefaultDict[str, List[SIPPSNode]],
        goal_node: Node,
        goal_np: np.ndarray,
        T: int,
        T_tag: int,
        vc_soft_np: np.ndarray,  # x, y, t -> bool (0/1)
        ec_soft_np: np.ndarray,  # x, y, x, y, t -> bool (0/1)
        pc_soft_np: np.ndarray,  # x, y -> time (int)
        agent=None,
) -> None:
    compute_c_g_h_f_values(node, goal_node, goal_np, T, T_tag, vc_soft_np, ec_soft_np, pc_soft_np)
    identical_nodes = get_identical_nodes(node, Q, P, ident_dict)
    for q in identical_nodes:
        if q.low <= node.low and q.c <= node.c:
            return
        elif node.low <= q.low and node.c <= q.c:
            if q in Q:
                Q.remove(q)
                ident_dict[q.ident_str].remove(q)
            if q in P:
                P.remove(q)
                ident_dict[q.ident_str].remove(q)
        elif node.low < q.high and q.low < node.high:
            if node.low < q.low:
                node.set_high(q.low)
            else:
                q.set_high(node.low)
    heapq.heappush(Q, node)
    ident_dict[node.ident_str].append(node)
    return


def run_sipps_expand_node(
        node: SIPPSNode,
        nodes_dict: Dict[str, Node],
        Q: List[SIPPSNode],
        P: List[SIPPSNode],
        ident_dict: DefaultDict[str, List[SIPPSNode]],
        si_table: Dict[str, List[Tuple[int, int]]],
        goal_node: Node,
        goal_np: np.ndarray,
        T: int,
        T_tag: int,
        ec_hard_np: np.ndarray,  # x, y, x, y, t -> bool (0/1)
        vc_soft_np: np.ndarray,  # x, y, t -> bool (0/1)
        ec_soft_np: np.ndarray,  # x, y, x, y, t -> bool (0/1)
        pc_soft_np: np.ndarray,  # x, y -> time (int)
        agent=None
):
    I_group: List[Tuple[Node, int]] = get_I_group(node, nodes_dict, si_table, agent)
    # I_group_names = [(v.xy_name, i, si_table[v.xy_name][i]) for v, i in I_group]
    for v_node, si_id in I_group:
        init_low, init_high = si_table[v_node.xy_name][si_id]
        new_low = get_low_without_hard_ec(node, node.n, v_node, init_low, init_high, ec_hard_np, agent)
        if new_low is None:
            continue
        new_low_tag = get_low_without_hard_and_soft_ec(node, node.n, v_node, new_low, init_high, ec_hard_np, ec_soft_np)
        if new_low_tag is not None and new_low < new_low_tag < init_high:
            n_1 = SIPPSNode(v_node, (new_low, new_low_tag), si_id, False, parent=node)
            run_sipps_insert_node(n_1, Q, P, ident_dict, goal_node, goal_np, T, T_tag, vc_soft_np, ec_soft_np, pc_soft_np)
            n_2 = SIPPSNode(v_node, (new_low_tag, init_high), si_id, False, parent=node)
            run_sipps_insert_node(n_2, Q, P, ident_dict, goal_node, goal_np, T, T_tag, vc_soft_np, ec_soft_np, pc_soft_np)
        else:
            n_3 = SIPPSNode(v_node, (new_low, init_high), si_id, False, parent=node)
            run_sipps_insert_node(n_3, Q, P, ident_dict, goal_node, goal_np, T, T_tag, vc_soft_np, ec_soft_np, pc_soft_np)


def run_sipps(
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
        inf_num: int = int(1e10),
        max_final_time: int = int(1e10),
        flag_k_limit: bool = False,
        k_limit: int = int(1e10),
        agent=None,
        **kwargs,
) -> Tuple[List[Node] | None, dict]:

    if pc_hard_np is not None and pc_hard_np[goal_node.x, goal_node.y] == 1:
        return None, {}

    si_table: Dict[str, List[Tuple[int, int]]] = get_si_table(nodes, nodes_dict, vc_hard_np, pc_hard_np, vc_soft_np, pc_soft_np, inf_num)
    root = SIPPSNode(start_node, si_table[start_node.xy_name][0], 0, False)
    T = 0
    goal_vc_times_list = get_vc_list(goal_node, vc_hard_np)
    if len(goal_vc_times_list) > 0:
        T = max(goal_vc_times_list) + 1
    goal_vc_times_list = get_vc_list(goal_node, vc_soft_np)
    T_tag = T
    if len(goal_vc_times_list) > 0:
        T_tag = max(T, max(goal_vc_times_list) + 1)
    goal_np: np.ndarray = h_dict[goal_node.xy_name]
    compute_c_g_h_f_values(root, goal_node, goal_np, T, T_tag, vc_soft_np, ec_soft_np, pc_soft_np)

    Q: List[SIPPSNode] = []
    P: List[SIPPSNode] = []
    ident_dict: DefaultDict[str, List[SIPPSNode]] = defaultdict(lambda: [])
    heapq.heappush(Q, root)
    ident_dict[root.ident_str].append(root)

    while len(Q) > 0:
        # print(f'\r{len(Q)=}, {len(P)=}', end='')
        next_n: SIPPSNode = heapq.heappop(Q)
        if next_n.is_goal or next_n.low >= k_limit:
            nodes_path, sipps_path = extract_path(next_n, agent=agent)
            sipps_path_names = [n.to_print() for n in sipps_path]
            return nodes_path, {
                'T': T, 'T_tag': T_tag, 'Q': Q, 'P': P, 'si_table': si_table, 'r_type': 'is_goal',
                'sipps_path': sipps_path, 'sipps_path_names': sipps_path_names, 'c': next_n.c,
            }
        if next_n.n == goal_node and next_n.low >= T:
            c_future = get_c_future(goal_node, next_n.low, vc_soft_np, pc_soft_np)
            if c_future == 0:
                nodes_path, sipps_path = extract_path(next_n, agent=agent)
                # nodes_path_names = [n.xy_name for n in nodes_path]
                sipps_path_names = [n.to_print() for n in sipps_path]
                return nodes_path, {
                    'T': T, 'T_tag': T_tag, 'Q': Q, 'P': P, 'si_table': si_table, 'r_type': 'c_future=0',
                    'sipps_path': sipps_path, 'sipps_path_names': sipps_path_names, 'c': next_n.c,
                }
            n_tag = duplicate_sipps_node(next_n)
            n_tag.is_goal = True
            n_tag.c += c_future
            run_sipps_insert_node(n_tag, Q, P, ident_dict, goal_node, goal_np, T,  T_tag, vc_soft_np, ec_soft_np, pc_soft_np)
        run_sipps_expand_node(next_n, nodes_dict, Q, P, ident_dict, si_table, goal_node, goal_np, T,  T_tag,
                              ec_hard_np, vc_soft_np, ec_soft_np, pc_soft_np, agent)
        heapq.heappush(P, next_n)
        ident_dict[next_n.ident_str].append(next_n)
    return None, {}


@use_profiler(save_dir='../stats/alg_sipps.pstat')
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

    result, info = run_sipps(
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


# if vc_hard_np is not None and vc_soft_np is not None:
#     assert vc_hard_np.shape[2] == vc_soft_np.shape[2]
#     assert ec_hard_np.shape[4] == ec_soft_np.shape[4]
    # assert int(max(np.max(pc_hard_np), np.max(pc_soft_np))) + 1 == vc_hard_np.shape[2]
    # assert int(max(np.max(pc_hard_np), np.max(pc_soft_np))) + 1 == ec_hard_np.shape[4]
# if pc_hard_np is not None:
#     assert pc_hard_np[goal_node.x, goal_node.y] == -1

