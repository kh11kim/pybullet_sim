# pybullet_sim_panda

Pybullet simulation environment for Franka Emika Panda

### Dependency
pybullet, numpy

### Simple example (see sim_example.ipynb)
```python
uid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # for loading plane

# Simulation configuration
rate = 240.
p.setTimeStep(1/rate)
p.resetSimulation() #init
p.setGravity(0, 0, -9.8) #set gravity

# Load
plane_id = p.loadURDF("plane.urdf") # load plane
panda = PandaBullet(p) # load robot

time.sleep(3)

while(1):
    panda.set_control_mode(mode_str="position")
    panda.control_joint_positions([0,0,0,0,0,1,1])
    print(panda.get_body_jacobian())
    print()
    #panda.set_control_mode(mode_str="torque")
    #panda.control_joint_torques([0,0,0,0,0,1,100])

    p.stepSimulation()
    time.sleep(1/rate)

```
