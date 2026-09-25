import time 
import mujoco
import mujoco.viewer

model=mujoco.MjModel.from_xml_path("../scenes/flat_scene.xml")
data=mujoco.MjData(model)
data.ctrl[:]=0.0
with mujoco.viewer.launch_passive(model,data)as viewer:
    while viewer.is_running():
        step_start=time.time()
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        viewer.sync()

        time_left = model.opt.timestep - (time.time() - step_start)
        if time_left > 0:
            time.sleep(time_left)