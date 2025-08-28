import genesis as gs

# Initialize Genesis (use GPU if available, else fall back to CPU)
try:
    gs.init(backend=gs.gpu)
except Exception as e:
    print("GPU backend not available, using CPU:", e)
    gs.init(backend=gs.cpu)

# Create a simulation scene with a visualizer
scene = gs.Scene(show_viewer=True)
plane = scene.add_entity(gs.morphs.Plane())  # ground plane
franka = scene.add_entity(gs.morphs.MJCF(file='xml/franka_emika_panda/panda.xml'))  # robot arm

scene.build()  # finalize scene setup

for i in range(1000):
    scene.step()  # step simulation 1000 times (simulate a few seconds)
