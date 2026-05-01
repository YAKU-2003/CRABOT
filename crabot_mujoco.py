import crabbot_mujoco
import mujoco.viewer
import math, time, sys, os

# Robot geometry
L1 = 0.032   # hip to knee (m)
L2 = 0.112   # knee to foot (m)
STAND_HEIGHT = 0.068  # base height at rest

# Gait params
TROT_PERIOD = 0.8
CRAB_PERIOD = 1.0
STEP_HEIGHT = 0.025
YAW_REACH   = 0.30   # yaw joint swing angle (rad) ~17 degrees
PITCH_STAND = 0.55   # pitch angle at stand (rad) ~31 degrees

# Actuators
ACTUATORS = {
    "FL": ("act_hip_yaw_FL", "act_knee_pitch_FL"),
    "FR": ("act_hip_yaw_FR", "act_knee_pitch_FR"),
    "RL": ("act_hip_yaw_RL", "act_knee_pitch_RL"),
    "RR": ("act_hip_yaw_RR", "act_knee_pitch_RR"),
}

# Rear legs face backwards so yaw direction is flipped
REAR_LEGS = {"RL", "RR"}

def compute_pitch(dz):
    """Compute pitch angle to reach height dz below hip."""
    r = min(dz, L1 + L2 - 1e-4)
    cos_hip = max(-1.0, min(1.0, (L1*L1 + r*r - L2*L2) / (2*L1*r)))
    alpha = math.atan2(0, r)  # reach=0, straight down
    beta  = math.acos(cos_hip)
    return alpha - beta

def trot_targets(t):
    """
    Trot gait: yaw swings fore/aft for propulsion,
    pitch lifts foot during swing.
    Diagonal pairs: FL+RR swing together, FR+RL swing together.
    """
    period = TROT_PERIOD
    phase  = (t % period) / period
    targets = {}

    groups  = {"A": ["FL", "RR"], "B": ["FR", "RL"]}
    offsets = {"A": 0.0, "B": 0.5}

    for group, legs in groups.items():
        p = (phase + offsets[group]) % 1.0

        if p < 0.4:  # swing phase
            sw = p / 0.4  # 0->1
            # Yaw: swing from back to front
            yaw = YAW_REACH * (sw - 0.5)
            # Pitch: lift foot in middle of swing
            lift = STEP_HEIGHT * math.sin(math.pi * sw)
            dz = STAND_HEIGHT - lift
            pitch = compute_pitch(dz)
        else:  # stance phase
            st = (p - 0.4) / 0.6  # 0->1
            # Yaw: push back from front to back
            yaw = YAW_REACH * (0.5 - st)
            dz = STAND_HEIGHT
            pitch = compute_pitch(dz)

        for leg in legs:
            # Rear legs face backward so flip yaw
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * yaw, pitch)

    return targets

def crab_targets(t):
    """
    Crab walk: lateral motion using yaw.
    Left pair swings left, right pair swings right.
    """
    period = CRAB_PERIOD
    phase  = (t % period) / period
    targets = {}

    groups  = {"L": ["FL", "RL"], "R": ["FR", "RR"]}
    offsets = {"L": 0.0, "R": 0.5}

    for group, legs in groups.items():
        p = (phase + offsets[group]) % 1.0
        d = 1.0 if group == "L" else -1.0

        if p < 0.4:
            sw = p / 0.4
            yaw = d * YAW_REACH * (sw - 0.5)
            lift = STEP_HEIGHT * math.sin(math.pi * sw)
            dz = STAND_HEIGHT - lift
            pitch = compute_pitch(dz)
        else:
            st = (p - 0.4) / 0.6
            yaw = d * YAW_REACH * (0.5 - st)
            dz = STAND_HEIGHT
            pitch = compute_pitch(dz)

        for leg in legs:
            targets[leg] = (yaw, pitch)

    return targets

def apply_targets(model, data, targets):
    for leg, (yaw, pitch) in targets.items():
        for act_name, val in zip(ACTUATORS[leg], (yaw, pitch)):
            aid = crabbot_mujoco.mj_name2id(model, crabbot_mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid >= 0:
                data.ctrl[aid] = val

def main():
    xml_path = sys.argv[1] if len(sys.argv) > 1 else "crabbot.xml"
    if not os.path.exists(xml_path):
        print(f"ERROR: {xml_path} not found")
        return

    model = crabbot_mujoco.MjModel.from_xml_path(xml_path)
    data  = crabbot_mujoco.MjData(model)
    crabbot_mujoco.mj_resetData(model, data)

    # Set neutral standing pose
    neutral_pitch = compute_pitch(STAND_HEIGHT)
    for leg in ACTUATORS:
        for act_name, val in zip(ACTUATORS[leg], (0.0, neutral_pitch)):
            aid = crabbot_mujoco.mj_name2id(model, crabbot_mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            if aid >= 0:
                data.ctrl[aid] = val

    # Settle
    for _ in range(2000):
        crabbot_mujoco.mj_step(model, data)

    current_gait = ["trot"]
    print("Running!")
    print("  T = Trot (forward)    C = Crab (sideways)    Q = Quit")

    with crabbot_mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance  = 0.8
        viewer.cam.elevation = -35
        viewer.cam.azimuth   = 0
        viewer.opt.flags[crabbot_mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True

        t    = 0.0
        step = model.opt.timestep

        while viewer.is_running():
            if current_gait[0] == "trot":
                targets = trot_targets(t)
            else:
                targets = crab_targets(t)

            apply_targets(model, data, targets)
            crabbot_mujoco.mj_step(model, data)

            # Camera follows robot
            base_id = crabbot_mujoco.mj_name2id(model, crabbot_mujoco.mjtObj.mjOBJ_BODY, "base_link")
            if base_id >= 0:
                viewer.cam.lookat[:] = data.xpos[base_id]

            viewer.sync()
            t += step
            time.sleep(step * 0.5)

if __name__ == "__main__":
    main()