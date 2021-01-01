from pyomo.environ import *
from pprint import pprint
import networkx as nx
import matplotlib.pyplot as plt

G=nx.Graph()

raw_map = [[17,29,4,16,30],[25,24,53,6,21],[12,23,48,22,31,26],[1,17,36,15,14],[32,33,20,17,45],[9,36,24,2,16],[10,11,37,29,30],[25,9,35,28],[6,8,36,15,47,35],[7,34,29,27,18],[7,38,32,57,37],[3,23,48,39,14,49],[40,21,42,24],[12,15,48,49,4,41],[4,9,14,36,47,49],[6,29,24,36,1],[1,4,30,26,5],[10,27,24,29,19,42],[18,54,27,42],[5,33,43,38,32,50],[13,53,58,40,2],[3,45,31,26,52],[3,12,39],[2,6,16,18,13],[2,8,53,28],[17,45,3,48,22],[10,18,19,34,54],[25,35,8,46],[1,10,16,18,7,30],[17,29,7,32,1],[3,22,51,52],[5,11,20,30,38],[5,20,44,50,43,45,55],[10,27,54,37,56],[8,9,28,47,46],[4,6,9,15,16],[7,11,34,57,56],[11,20,32,57,43],[12,23,49],[13,21,58,42],[47,49,14],[13,40,19,18,58],[20,33,50,38],[33,45,51,55],[22,26,44,5,33],[35,28,47],[9,15,35,41,46],[3,12,14,26],[14,39,41,12,15],[33,43,20,55],[31,44,55,52],[31,51,22],[2,21,25],[19,27,34,56],[44,50,51,33],[34,37,54],[11,37,38],[21,40,42]]
nodes_map = {}
nodes = []

model = ConcreteModel()

for node_idx in range(len(raw_map)):
	nodes_map[node_idx+1] = raw_map[node_idx]
	nodes.insert(0, node_idx)

	G.add_node(node_idx+1)
	for inner_node in raw_map[node_idx]:
		edge = (node_idx+1, inner_node)
		G.add_edge(*edge)

nodes.sort()

model.set_nodes = RangeSet(1, len(nodes))
model.set_nodes_higher_dim = RangeSet(1, len(nodes))

model.factory = Var(model.set_nodes, within=Binary, initialize = 0)
model.power_plant = Var(model.set_nodes, within=Binary, initialize = 0)
model.nodes_factory_power_plant_adj = Var(model.set_nodes, model.set_nodes, within=Binary, initialize = 0)

model.constraints = ConstraintList()

for node in model.set_nodes:
	print("proc node:", node)
	# ensure node is either factory or power_plant or nothing
	model.constraints.add(model.factory[node] + model.power_plant[node] <= 1)
	if node == len(raw_map):
		model.constraints.add(model.factory[node] == 1.0)
	for inner_nodes in nodes_map[node]:
		model.constraints.add(model.factory[inner_nodes] * model.power_plant[node] == model.nodes_factory_power_plant_adj[node, inner_nodes])
	for inner_nodes in model.set_nodes:
		if not (inner_nodes in nodes_map[node]):
			model.constraints.add(0 == model.nodes_factory_power_plant_adj[node, inner_nodes])


model.objective = Objective(expr = sum(model.factory[node] for node in model.set_nodes) + (2 * sum(sum(model.nodes_factory_power_plant_adj[node, inner_node] for node in model.set_nodes) for inner_node in model.set_nodes)),
    sense=maximize)	

#pprint(nodes_map)
#model.pprint()

SolverFactory('gurobi').solve(model, tee=True)

debug_mode = False

node_label_mapping = {}
node_color_mapping = []

for node in model.set_nodes:
	factory_val = model.factory[node]()
	power_plant_val = model.power_plant[node]()
	adj_count = 0

	if debug_mode:
		for inner_node in model.set_nodes:
			adj_count = adj_count + model.nodes_factory_power_plant_adj[node, inner_node]()
			if model.nodes_factory_power_plant_adj[node, inner_node]() > 0.9:
				print(node, inner_node)

		print("--Node--", node, factory_val, power_plant_val, adj_count)
	else:
		node_name = ""
		if factory_val > 0.9:
			node_name = "factory"
			node_color_mapping.append('brown')
		elif power_plant_val > 0.9:
			node_name = "pp"
			node_color_mapping.append('green')
		else:
			node_name = "none"
		node_label_mapping[node+1] = str(node+1) + "_" + node_name
		print(node_name)

G = nx.relabel_nodes(G, node_label_mapping)
G_pos = nx.kamada_kawai_layout(G)
nx.draw(G, pos=G_pos, node_color=node_color_mapping, with_labels=True)
plt.show()
