from pyomo.environ import *

model = ConcreteModel()

model.set_x_dim = RangeSet(1, 11)
model.set_y_dim = RangeSet(1, 11)

model.chairs = Var(model.set_x_dim, model.set_y_dim, within=Binary, initialize = 0)
model.tables = Var(model.set_x_dim, model.set_y_dim, within=Binary, initialize = 0)

model.constraints = ConstraintList()

for x in model.set_x_dim:
	for y in model.set_y_dim:
		up_index = y - 1
		down_index = y + 1
		left_index = x - 1
		right_index = x + 1

		up_valid = up_index in model.set_y_dim
		down_valid = down_index in model.set_y_dim
		left_valid = left_index in model.set_x_dim
		right_valid = right_index in model.set_x_dim

		model.constraints.add(model.chairs[x, y] + model.tables[x, y] <= 1) # Ensure that it is either a chair/table/nothing at a given coordinate

		if up_valid and down_valid and left_valid and right_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[x, down_index] + model.tables[left_index, y] + model.tables[right_index, y])
		# ----
		elif down_valid and left_valid and right_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, down_index] + model.tables[left_index, y] + model.tables[right_index, y])
		elif left_valid and up_valid and down_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[x, down_index] + model.tables[left_index, y])
		elif up_valid and left_valid and right_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[left_index, y] + model.tables[right_index, y])
		elif right_valid and up_valid and down_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[x, down_index] + model.tables[right_index, y])
		# ----
		elif down_valid and right_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, down_index] + model.tables[right_index, y])
		elif down_valid and left_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, down_index] + model.tables[left_index, y])
		elif up_valid and right_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[right_index, y])
		elif up_valid and left_valid:
			model.constraints.add(model.chairs[x, y] <= model.tables[x, up_index] + model.tables[left_index, y])



model.objective = Objective(expr = sum(sum(model.chairs[x, y] for y in model.set_y_dim) for x in model.set_x_dim),
    sense=maximize)		

SolverFactory('glpk').solve(model, tee=True)

model.display()

for y in model.set_y_dim:
	print()
	for x in model.set_x_dim:
		chair_value = model.chairs[x, y]()
		table_value = model.tables[x, y]()
		assert chair_value != table_value
		if chair_value == 1.0:
			print('x', end = '')
		elif table_value == 1.0:
			print('0', end = '')
		else:
			print('-', end = '')
