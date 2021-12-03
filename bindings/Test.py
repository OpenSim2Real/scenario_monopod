import scenario.bindings.monopod as monopod

world = monopod.World()
model = world.get_model("monopod")
print(model.joint_names())
print(model.joint_positions())
