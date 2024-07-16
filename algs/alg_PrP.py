from algs.alg_PrP_functions import *
from algs.alg_sipps import run_sipps
from algs.alg_temporal_a_star import run_temporal_a_star
from run_single_MAPF_func import run_mapf_alg


def run_prp(
        start_nodes: List[Node],
        goal_nodes: List[Node],
        nodes: List[Node],
        nodes_dict: Dict[str, Node],
        h_dict: Dict[str, np.ndarray],
        map_dim: Tuple[int, int],
        params: Dict,
) -> Tuple[Dict[str, List[Node]] | None, dict]:

    constr_type: bool = params['constr_type']
    alg_name: bool = params['alg_name']
    pf_alg = params['pf_alg']
    to_render: bool = params['to_render']
    max_time: bool = params['max_time']

    start_time = time.time()

    # create agents
    agents = []
    for num, (s_node, g_node) in enumerate(zip(start_nodes, goal_nodes)):
        new_agent = AgentPrP(num, s_node, g_node)
        agents.append(new_agent)

    r_iter = 0
    while time.time() - start_time < max_time:
        # calc paths
        h_priority_agents: List[AgentPrP] = []
        for agent in agents:
            (vc_hard_np, ec_hard_np, pc_hard_np,
             vc_soft_np, ec_soft_np, pc_soft_np) = create_hard_and_soft_constraints(h_priority_agents, map_dim, constr_type)
            new_path, sipps_info = pf_alg(
                agent.start_node, agent.goal_node, nodes, nodes_dict, h_dict,
                vc_hard_np, ec_hard_np, pc_hard_np, vc_soft_np, ec_soft_np, pc_soft_np, agent=agent
            )
            if new_path is None:
                agent.path = None
                break
            agent.path = new_path[:]
            h_priority_agents.append(agent)

            # checks
            runtime = time.time() - start_time
            print(f'\r[{alg_name}] {r_iter=: <3} | agents: {len(h_priority_agents): <3} / {len(agents)} | {runtime= : .2f} s.')  # , end=''
            collisions: int = 0
            align_all_paths(h_priority_agents)
            for i in range(len(h_priority_agents[0].path)):
                to_count = False if constr_type == 'hard' else True
                # collisions += check_vc_ec_neic_iter(h_priority_agents, i, to_count)
            if collisions > 0:
                print(f'{collisions=} | {sipps_info['c']=}')

        # return check
        to_return = True
        for agent in agents:
            if agent.path is None:
                to_return = False
                break
            if agent.path[-1] != agent.goal_node:
                to_return = False
                break
        if to_return:
            return {a.name: a.path for a in agents}, {'agents': agents}

        # reshuffle
        r_iter += 1
        random.shuffle(agents)
        for agent in agents:
            agent.path = []

    return None, {}


@use_profiler(save_dir='../stats/alg_prp.pstat')
def main():
    # to_render = True
    to_render = False

    pf_alg_tuple = ('SIPPS', run_sipps)
    # pf_alg_tuple = ('A*', run_temporal_a_star)

    params = {
        'max_time': 1000,
        'alg_name': f'PrP-{pf_alg_tuple[0]}',
        'constr_type': 'hard',
        'pf_alg': pf_alg_tuple[1],
        'to_render': to_render,

    }
    run_mapf_alg(alg=run_prp, params=params)


if __name__ == '__main__':
    main()







