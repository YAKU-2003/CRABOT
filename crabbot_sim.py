"""
Crabbot PyBullet Simulation
Includes a "horse-trot" gait that mimics the bouncing gait
observed on the real hardware when pressing 's' for backward.

Gaits:
  H - Horse trot (mimics real hardware bounce)
  C - Crab walk sideways
  T - Smooth trot forward
  W - Wave crab
  S - Stand
"""

import pybullet as p
import pybullet_data
import math, time, os, sys


JOINT_NAMES = [
    "Revolute 48", "Revolute 45", "Revolute 36", "Revolute 39",
    "Revolute 44", "Revolute 47", "Revolute 38", "Revolute 41",
]

LEGS = {
    "FR": ("Revolute 48", "Revolute 44"),
    "FL": ("Revolute 45", "Revolute 47"),
    "RR": ("Revolute 36", "Revolute 38"),
    "RL": ("Revolute 39", "Revolute 41"),
}

REAR_LEGS = {"RL", "RR"}
PAIR_A = ["FL", "RR"]   # diagonal
PAIR_B = ["FR", "RL"]

YAW_LIMIT = math.radians(50)
PITCH_MIN = math.radians(-10)
PITCH_MAX = math.radians(90)
SIM_STEP = 1./240.


# ── Mimics the hardware 3-phase trot from crab_gait.py ──
def horse_trot(t, period, yaw_amp, lift_amp, stance_pitch, direction=1):
    """
    Mimics the real hardware gait that produced the horse-bounce motion.
    3 phases like the actual robot code:
      Phase 1 (33%): Pair A lifts+swings forward, Pair B stays
      Phase 2 (33%): Pair A plants forward, Pair B lifts+swings forward
      Phase 3 (33%): Both pairs swing back HALF-step (this causes the bounce)
    
    direction: +1 forward (full yaw), -1 backward (0.6× yaw scaling like the code)
    """
    # Match hardware: backward uses 0.6× yaw amplitude
    eff_yaw = yaw_amp if direction == 1 else yaw_amp * 0.6
    eff_yaw *= direction

    phase = (t % period) / period
    targets = {}

    if phase < 0.333:
        # Phase 1: A lifts and swings to +yaw, B stays at -yaw planted
        for leg in PAIR_A:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * eff_yaw, stance_pitch - lift_amp)
        for leg in PAIR_B:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * -eff_yaw, stance_pitch)

    elif phase < 0.666:
        # Phase 2: A plants forward, B lifts and swings to -yaw
        for leg in PAIR_A:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * eff_yaw, stance_pitch)
        for leg in PAIR_B:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * -eff_yaw, stance_pitch - lift_amp)

    else:
        # Phase 3: Both swing to half-step — THIS causes the bounce on hardware
        # (legs converge back to near-neutral simultaneously, body lifts briefly)
        for leg in PAIR_A:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * -eff_yaw * 0.5, stance_pitch)
        for leg in PAIR_B:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * eff_yaw * 0.5, stance_pitch)

    return targets


def smooth_trot(t, period, yaw_amp, lift_amp, stance_pitch):
    """Smooth continuous trot — no bounce."""
    phase = (t % period) / period
    targets = {}
    for group, legs, off in [("A", PAIR_A, 0.0), ("B", PAIR_B, 0.5)]:
        ph = (phase + off) % 1.0
        if ph < 0.4:
            sw = ph / 0.4
            yaw = yaw_amp * (sw - 0.5)
            pitch = stance_pitch - lift_amp * math.sin(math.pi * sw)
        else:
            st = (ph - 0.4) / 0.6
            yaw = yaw_amp * (0.5 - st)
            pitch = stance_pitch
        for leg in legs:
            sign = -1.0 if leg in REAR_LEGS else 1.0
            targets[leg] = (sign * yaw, pitch)
    return targets


def crab_walk(t, period, yaw_amp, lift_amp, stance_pitch):
    phase = (t % period) / period
    targets = {}
    for group, legs, off in [("L", ["FL", "RL"], 0.0),
                              ("R", ["FR", "RR"], 0.5)]:
        ph = (phase + off) % 1.0
        d = 1.0 if group == "L" else -1.0
        if ph < 0.4:
            sw = ph / 0.4
            yaw = d * yaw_amp * (sw - 0.5)
            pitch = stance_pitch - lift_amp * math.sin(math.pi * sw)
        else:
            st = (ph - 0.4) / 0.6
            yaw = d * yaw_amp * (0.5 - st)
            pitch = stance_pitch
        for leg in legs:
            targets[leg] = (yaw, pitch)
    return targets


def wave_crab(t, period, yaw_amp, lift_amp, stance_pitch):
    phase = (t % period) / period
    targets = {}
    leg_order = ["FL", "RL", "FR", "RR"]
    leg_dirs = {"FL": 1.0, "RL": 1.0, "FR": -1.0, "RR": -1.0}
    for i, leg in enumerate(leg_order):
        lp = (phase - i * 0.25) % 1.0
        d = leg_dirs[leg]
        if lp < 0.25:
            sw = lp / 0.25
            yaw = d * yaw_amp * (sw - 0.5)
            pitch = stance_pitch - lift_amp * math.sin(math.pi * sw)
        else:
            st = (lp - 0.25) / 0.75
            yaw = d * yaw_amp * (0.5 - st)
            pitch = stance_pitch
        targets[leg] = (yaw, pitch)
    return targets


def stand_targets(stance_pitch):
    return {leg: (0.0, stance_pitch) for leg in LEGS}


def apply_targets(robot_id, joint_map, targets, force=8.0, vel=2.5):
    for leg, (yaw, pitch) in targets.items():
        yaw_j, pitch_j = LEGS[leg]
        yaw = max(-YAW_LIMIT, min(YAW_LIMIT, yaw))
        pitch = max(PITCH_MIN, min(PITCH_MAX, pitch))
        if yaw_j in joint_map:
            p.setJointMotorControl2(robot_id, joint_map[yaw_j],
                p.POSITION_CONTROL, targetPosition=yaw,
                force=force, maxVelocity=vel)
        if pitch_j in joint_map:
            p.setJointMotorControl2(robot_id, joint_map[pitch_j],
                p.POSITION_CONTROL, targetPosition=pitch,
                force=force, maxVelocity=vel)


def load_robot(urdf_path):
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0.20],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False, flags=p.URDF_USE_INERTIA_FROM_FILE)
    joint_map = {}
    for i in range(p.getNumJoints(robot_id)):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode()
        joint_map[name] = i
        if info[2] == p.JOINT_REVOLUTE:
            p.setJointMotorControl2(robot_id, i,
                controlMode=p.VELOCITY_CONTROL, force=0)
    print(f"Loaded {len(joint_map)} joints")
    for name in JOINT_NAMES:
        mark = "✓" if name in joint_map else "✗"
        print(f"  {mark} {name}")
    return robot_id, joint_map


def main():
    urdf_path = sys.argv[1] if len(sys.argv) > 1 else "crabbot_fixed.urdf"
    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at '{urdf_path}'")
        return

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_STEP)
    p.loadURDF("plane.urdf")

    p.resetDebugVisualizerCamera(cameraDistance=0.6, cameraYaw=45,
        cameraPitch=-30, cameraTargetPosition=[0, 0, 0.05])

    print("\nLoading robot...")
    robot_id, joint_map = load_robot(urdf_path)

    speed_s  = p.addUserDebugParameter("Speed",         0.3, 2.0, 0.8)
    yaw_s    = p.addUserDebugParameter("Step Yaw",      0.05, 0.7, 0.26)
    lift_s   = p.addUserDebugParameter("Step Lift",     0.0, 0.6, 0.26)
    stance_s = p.addUserDebugParameter("Stance Pitch",  0.2, 1.2, 0.65)
    force_s  = p.addUserDebugParameter("Motor Force",   1.0, 20.0, 8.0)
    dir_s    = p.addUserDebugParameter("Direction (-1 back, +1 fwd)", -1, 1, 1)

    horse_btn = p.addUserDebugParameter(">> HORSE TROT (mimics real bot)", 1, 0, 1)
    crab_btn  = p.addUserDebugParameter(">> CRAB WALK",  1, 0, 0)
    smooth_btn= p.addUserDebugParameter(">> SMOOTH TROT", 1, 0, 0)
    wave_btn  = p.addUserDebugParameter(">> WAVE CRAB",  1, 0, 0)
    stand_btn = p.addUserDebugParameter(">> STAND",      1, 0, 0)

    text_id = p.addUserDebugText("HORSE TROT", [0, 0, 0.20],
        textColorRGB=[1.0, 0.4, 0.2], textSize=1.5)

    current_gait = "horse"
    prev = {"horse": 1, "crab": 0, "smooth": 0, "wave": 0, "stand": 0}

    print("\nSettling...")
    for _ in range(500):
        apply_targets(robot_id, joint_map, stand_targets(0.65), force=8.0)
        p.stepSimulation()

    print("Running!")
    print("  HORSE TROT mimics the bouncing gait you saw on hardware")
    print("  Use Direction slider: +1 forward, -1 backward (0.6× yaw)")

    t = 0.0
    try:
        while True:
            speed    = p.readUserDebugParameter(speed_s)
            yaw_amp  = p.readUserDebugParameter(yaw_s)
            lift_amp = p.readUserDebugParameter(lift_s)
            stance   = p.readUserDebugParameter(stance_s)
            force    = p.readUserDebugParameter(force_s)
            direction = 1 if p.readUserDebugParameter(dir_s) >= 0 else -1

            for name, btn in [("horse", horse_btn), ("crab", crab_btn),
                              ("smooth", smooth_btn), ("wave", wave_btn),
                              ("stand", stand_btn)]:
                v = p.readUserDebugParameter(btn)
                if v != prev[name]:
                    prev[name] = v
                    current_gait = name
                    labels = {"horse": "HORSE TROT", "crab": "CRAB WALK",
                              "smooth": "SMOOTH TROT", "wave": "WAVE CRAB",
                              "stand": "STAND"}
                    colors = {"horse": [1.0, 0.4, 0.2], "crab": [0.9, 0.5, 0.1],
                              "smooth": [0.2, 0.9, 0.4], "wave": [0.4, 0.6, 0.9],
                              "stand": [0.7, 0.7, 0.7]}
                    p.addUserDebugText(labels[name], [0, 0, 0.20],
                        textColorRGB=colors[name], textSize=1.5,
                        replaceItemUniqueId=text_id)

            period = 1.0 / max(speed, 0.1)

            if current_gait == "horse":
                targets = horse_trot(t, period, yaw_amp, lift_amp, stance, direction)
            elif current_gait == "crab":
                targets = crab_walk(t, period, yaw_amp, lift_amp, stance)
            elif current_gait == "smooth":
                targets = smooth_trot(t, period, yaw_amp, lift_amp, stance)
            elif current_gait == "wave":
                targets = wave_crab(t, period * 1.5, yaw_amp, lift_amp, stance)
            else:
                targets = stand_targets(stance)

            apply_targets(robot_id, joint_map, targets, force=force)
            p.stepSimulation()
            t += SIM_STEP
            time.sleep(SIM_STEP)

    except p.error:
        print("Window closed.")


if __name__ == "__main__":
    main()
