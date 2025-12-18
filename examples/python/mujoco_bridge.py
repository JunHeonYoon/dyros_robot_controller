import time
import mujoco
import mujoco.viewer
import os
import importlib
from typing import Dict

def _precise_sleep(duration):
    """Busy-wait sleep function to achieve precise timing.
    Uses time.perf_counter() for high-resolution timing.
    """
    start = time.perf_counter()
    while True:
        now = time.perf_counter()
        if (now - start) >= duration:
            break
        
def load_controller(name: str):
    """Dynamically load a controller class by robot name.
    
    Example:
        name = "fr3"
        → imports module "fr3_controller.py"
        → gets class "FR3Controller"
    """
    mod = importlib.import_module(f"{name}_controller")
    # cls_name = f"{name.upper()}Controller"
    cls_core = "".join(part.upper() for part in name.split("_"))
    cls_name = f"{cls_core}Controller"
    return getattr(mod, cls_name)

class MujocoBridge():
    """Bridge between MuJoCo simulation and a dyros robot controller.
    
    Responsibilities:
    - Load MuJoCo model/data
    - Extract joint and actuator name-to-index mappings
    - Update joint states from simulation
    - Send control inputs into simulation
    - Run simulation loop while delegating control to the loaded controller
    """

    def __init__(self, robot_name: str):
        # Load MuJoCo model and data from XML (currently fixed to FR3 scene)
        mjcf_file_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robots", robot_name, "scene.xml")
        self.mujoco_model = mujoco.MjModel.from_xml_path(mjcf_file_path)
        self.mujoco_data = mujoco.MjData(self.mujoco_model)
        
        # Build dictionary: joint name → joint index
        self.joint_dict = {}
        for i in range(self.mujoco_model.njnt):
            name_adr = self.mujoco_model.name_jntadr[i]
            jname = self.mujoco_model.names[name_adr:].split(b'\x00', 1)[0].decode('utf-8')
            if not jname:
                continue
            self.joint_dict[jname] = i

        # Build dictionary: actuator name → actuator index
        self.ctrl_dict = {}
        for i in range(self.mujoco_model.nu):
            name_adr = self.mujoco_model.name_actuatoradr[i]
            aname = self.mujoco_model.names[name_adr:].split(b'\x00', 1)[0].decode('utf-8')
            self.ctrl_dict[aname] = i
            
        # Simulation timestep (control cycle)
        self.dt = self.mujoco_model.opt.timestep

        # Viewer update frequency
        self.viewer_fps = 60
        
        # Dynamically load the controller class based on robot_name
        CtrlClass = load_controller(robot_name)
        self.controller = CtrlClass(self.dt)
        
    def update_joint_state(self):
        """Read current joint positions/velocities from MuJoCo."""
        qpos_dict, qvel_dict = {}, {}

        for jname, jidx in self.joint_dict.items():
            qpos_adr = self.mujoco_model.jnt_qposadr[jidx]
            dof_adr  = self.mujoco_model.jnt_dofadr[jidx]
            jtype    = self.mujoco_model.jnt_type[jidx]

            if jtype == mujoco.mjtJoint.mjJNT_FREE:
                qpos_dict[jname] = self.mujoco_data.qpos[qpos_adr:qpos_adr+7].copy()
                qvel_dict[jname] = self.mujoco_data.qvel[dof_adr:dof_adr+6].copy()
            elif jtype == mujoco.mjtJoint.mjJNT_BALL:
                qpos_dict[jname] = self.mujoco_data.qpos[qpos_adr:qpos_adr+4].copy()
                qvel_dict[jname] = self.mujoco_data.qvel[dof_adr:dof_adr+3].copy()
            else:  # HINGE or SLIDE
                qpos_dict[jname] = float(self.mujoco_data.qpos[qpos_adr])
                qvel_dict[jname] = float(self.mujoco_data.qvel[dof_adr])

        return qpos_dict, qvel_dict

    
    def command_ctrl(self, ctrl_dict: Dict):
        """Write controller outputs into MuJoCo actuators."""
        for cname, cvalue in ctrl_dict.items():
            if cname in self.ctrl_dict.keys():
                self.mujoco_data.ctrl[self.ctrl_dict[cname]] = cvalue
            
    def run(self):
        """Main simulation loop.
        
        Workflow:
        - Step MuJoCo physics
        - Read joint states
        - Pass states to the controller (update_model)
        - Get control commands from controller (compute)
        - Apply commands to actuators
        - Sync viewer periodically
        - Keep precise real-time pacing
        """
        with mujoco.viewer.launch_passive(self.mujoco_model, self.mujoco_data,
                                          show_left_ui=False, show_right_ui=False) as viewer:
            last_viewr_update_time = 0
            while viewer.is_running():
                step_start = time.perf_counter()

                # Step physics forward one timestep
                mujoco.mj_step(self.mujoco_model, self.mujoco_data)

                # Read joint states
                qpos_dict, qvel_dict = self.update_joint_state()
                self.sim_time = self.mujoco_data.time

                # Update controller with current state
                self.controller.update_model(self.sim_time, qpos_dict, qvel_dict)

                # Compute control input
                ctrl_dict = self.controller.compute()

                # Send control input to simulation
                self.command_ctrl(ctrl_dict)

                # Update the viewer at fixed FPS
                if self.mujoco_data.time - last_viewr_update_time > 1/self.viewer_fps:
                    viewer.sync()
                    last_viewr_update_time = self.mujoco_data.time

                # Maintain real-time pacing
                time_until_next_step = self.dt - (time.perf_counter() - step_start)
                if time_until_next_step > 0:
                    _precise_sleep(time_until_next_step)
