"""
Crabbot PyBullet Simulation
Quadruped with 2-DOF legs (yaw + pitch)
  - Yaw:   ±50 deg  (horizontal swing)
  - Pitch: -10 to +90 deg (vertical lift/push)

Leg geometry:
  hip-to-knee : 32 mm
  knee-to-foot: 112 mm
  stand height: ~72 mm from ground to base_link

Gaits implemented:
  [T] Trot      - diagonal pairs (FL+RR, FR+RL)
  [C] Crab walk - lateral motion, all legs in crab phase pattern

Controls (keyboard while sim window focused):
  T  - switch to Trot
  C  - switch to Crab walk
  R  - reset robot position
  ESC/Q - quit
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import math

# ── Robot geometry ──────────────────────────────────────────────────────────
L1 = 0.032   # hip-to-knee (m)
L2 = 0.112   # knee-to-foot (m)
STAND_HEIGHT = 0.072  # base_link height at rest (m)

# ── Joint name → index mapping (filled after URDF load) ─────────────────────
# Joints of interest (active revolute joints) - matches crabbot_fixed.urdf
JOINT_NAMES = [
    "Revolute 48",  # yaw  - FRONT-RIGHT
    "Revolute 45",  # yaw  - FRONT-LEFT
    "Revolute 36",  # yaw  - REAR-RIGHT
    "Revolute 39",  # yaw  - REAR-LEFT
    "Revolute 44",  # knee - foot_1 (FRONT-RIGHT)
    "Revolute 47",  # knee - foot_2 (FRONT-LEFT)
    "Revolute 38",  # knee - foot_3 (REAR-RIGHT)
    "Revolute 41",  # knee - foot_4 (REAR-LEFT)
]

LEGS = {
    "FR": ("Revolute 48", "Revolute 44"),
    "FL": ("Revolute 45", "Revolute 47"),
    "RR": ("Revolute 36", "Revolute 38"),
    "RL": ("Revolute 39", "Revolute 41"),
}

# ── Joint limits (rad) ───────────────────────────────────────────────────────
YAW_LIMIT   = math.radians(50)    # ±50 deg
PITCH_MIN   = math.radians(-10)   # -10 deg
PITCH_MAX   = math.radians(90)    #  90 deg

# ── Gait parameters ─────────────────────────────────────────────────────────
TROT_PERIOD   = 1.2   # seconds per full trot cycle
CRAB_PERIOD   = 1.4   # seconds per full crab cycle
STEP_HEIGHT   = 0.03  # how high foot lifts (m)
STEP_REACH    = 0.025 # forward/lateral reach per step (m)
SIM_STEP      = 1./480.


# ── IK: given desired foot offset from hip, return (yaw, pitch) angles ──────
def leg_ik(dx, dy, dz):
    """
    Simple 2-DOF IK for a leg in the hip frame.
    dx = forward, dy = lateral outward, dz = downward (positive = down)
    Returns (yaw_angle, pitch_angle) in radians, clamped to joint limits.
    """
    # Yaw from lateral/forward offset
    yaw = math.atan2(dy, dx) if (abs(dx) > 1e-6 or abs(dy) > 1e-6) else 0.0

    # Reach in the sagittal plane (after yaw rotation)
    reach = math.sqrt(dx*dx + dy*dy)

    # Total distance from hip to foot
    r = math.sqrt(reach*reach + dz*dz)
    r = min(r, L1 + L2 - 1e-4)  # clamp to reachable

    # Knee angle via cosine rule
    cos_knee = (L1*L1 + L2*L2 - r*r) / (2*L1*L2)
    cos_knee = max(-1.0, min(1.0, cos_knee))

    # Hip pitch angle
    alpha = math.atan2(reach, dz)
    cos_hip = (L1*L1 + r*r - L2*L2) / (2*L1*r)
    cos_hip = max(-1.0, min(1.0, cos_hip))
    beta = math.acos(cos_hip)
    pitch = alpha - beta  # pitch relative to vertical

    # Clamp to joint limits
    yaw   = max(-YAW_LIMIT,  min(YAW_LIMIT,  yaw))
    pitch = max(PITCH_MIN,   min(PITCH_MAX,  pitch))

    return yaw, pitch


# ── Stance foot position (neutral, relative to hip) ─────────────────────────
def neutral_foot(leg_name):
    """
    Returns (dx, dy, dz) neutral foot position in hip frame.
    dz is the downward distance = stand height (approx).
    """
    return (0.0, 0.0, STAND_HEIGHT)


# ── Gait generators ─────────────────────────────────────────────────────────

def trot_targets(t, speed=1.0, step_height=STEP_HEIGHT, step_reach=STEP_REACH):
    """
    Diagonal trot: FR+RL swing together, FL+RR swing together.
    Returns dict: leg_name -> (yaw, pitch)
    """
    period = TROT_PERIOD / max(speed, 0.1)
    phase = (t % period) / period  # 0..1

    targets = {}
    # Diagonal pairs: group A = FR+RL, group B = FL+RR
    groups = {
        "A": ["FR", "RL"],
        "B": ["FL", "RR"],
    }
    offsets = {"A": 0.0, "B": 0.5}  # B is half-cycle behind A

    for group, legs in groups.items():
        p_phase = (phase + offsets[group]) % 1.0

        if p_phase < 0.5:
            # Swing phase: lift and reach forward
            swing_t = p_phase / 0.5  # 0..1 within swing
            dx = step_reach * (swing_t - 0.5)  # -reach/2 to +reach/2
            dz = STAND_HEIGHT - step_height * math.sin(math.pi * swing_t)
        else:
            # Stance phase: push back
            stance_t = (p_phase - 0.5) / 0.5
            dx = step_reach * (0.5 - stance_t)
            dz = STAND_HEIGHT

        for leg in legs:
            yaw, pitch = leg_ik(dx, 0.0, dz)
            targets[leg] = (yaw, pitch)

    return targets


def crab_targets(t, speed=1.0, step_height=STEP_HEIGHT, step_reach=STEP_REACH):
    """
    Crab walk: lateral motion. Legs on same side move together.
    Phase pattern: left pair swings while right pair stances, then swap.
    """
    period = CRAB_PERIOD / max(speed, 0.1)
    phase = (t % period) / period

    targets = {}
    groups = {
        "L": ["FL", "RL"],
        "R": ["FR", "RR"],
    }
    offsets = {"L": 0.0, "R": 0.5}

    for group, legs in groups.items():
        p_phase = (phase + offsets[group]) % 1.0
        # Lateral direction: left group steps left (+y), right group steps right (-y)
        direction = 1.0 if group == "L" else -1.0

        if p_phase < 0.5:
            swing_t = p_phase / 0.5
            dy = direction * step_reach * (swing_t - 0.5)
            dz = STAND_HEIGHT - step_height * math.sin(math.pi * swing_t)
        else:
            stance_t = (p_phase - 0.5) / 0.5
            dy = direction * step_reach * (0.5 - stance_t)
            dz = STAND_HEIGHT

        for leg in legs:
            yaw, pitch = leg_ik(0.0, dy, dz)
            targets[leg] = (yaw, pitch)

    return targets


# ── PyBullet helpers ─────────────────────────────────────────────────────────

def load_robot(urdf_path):
    """Load the URDF and return (robot_id, joint_map)."""
    robot_id = p.loadURDF(
        urdf_path,
        basePosition=[0, 0, STAND_HEIGHT + 0.05],
        baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
        useFixedBase=False,
        flags=p.URDF_USE_INERTIA_FROM_FILE,
    )

    # Build joint name → index map
    joint_map = {}
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        name = info[1].decode("utf-8")
        joint_map[name] = i
        # Disable default velocity motor (causes chaos if left on)
        if info[2] == p.JOINT_REVOLUTE:
            p.setJointMotorControl2(robot_id, i,
                controlMode=p.VELOCITY_CONTROL, force=0)
        print(f"  Joint {i:2d}: {name}  type={info[2]}")

    return robot_id, joint_map


def set_joint(robot_id, joint_map, joint_name, angle):
    """Send position command to a joint."""
    if joint_name not in joint_map:
        return
    p.setJointMotorControl2(
        robot_id,
        joint_map[joint_name],
        controlMode=p.POSITION_CONTROL,
        targetPosition=angle,
        force=15.0,         # increased for stability
        maxVelocity=2.0,
    )


def apply_targets(robot_id, joint_map, targets):
    """Apply a dict of {leg_name: (yaw, pitch)} to the robot."""
    for leg, (yaw, pitch) in targets.items():
        yaw_joint, pitch_joint = LEGS[leg]
        set_joint(robot_id, joint_map, yaw_joint,   yaw)
        set_joint(robot_id, joint_map, pitch_joint, pitch)


# ── Main simulation loop ─────────────────────────────────────────────────────

def main():
    import sys
    import os

    # ── locate URDF ──
    if len(sys.argv) > 1:
        urdf_path = sys.argv[1]
    else:
        # default: same folder as this script
        urdf_path = os.path.join(os.path.dirname(__file__), "crabbot.urdf")

    if not os.path.exists(urdf_path):
        print(f"ERROR: URDF not found at '{urdf_path}'")
        print("Usage: python crabbot_sim.py path/to/crabbot.urdf")
        return

    # ── init PyBullet ──
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(SIM_STEP)

    # Load ground plane
    p.loadURDF("plane.urdf")

    # Nicer camera angle
    p.resetDebugVisualizerCamera(
        cameraDistance=0.6,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.05],
    )

    print("\nLoading crabbot URDF...")
    robot_id, joint_map = load_robot(urdf_path)
    print(f"Robot loaded. {len(joint_map)} joints found.\n")

    # ── GUI sliders for live tuning ──
    speed_slider   = p.addUserDebugParameter("Speed",        0.2, 2.0, 1.0)
    height_slider  = p.addUserDebugParameter("Step Height",  0.01, 0.06, STEP_HEIGHT)
    reach_slider   = p.addUserDebugParameter("Step Reach",   0.01, 0.05, STEP_REACH)

    # ── Gait selection buttons ──
    trot_btn = p.addUserDebugParameter(">> TROT",  1, 0, 1)
    crab_btn = p.addUserDebugParameter(">> CRAB",  1, 0, 0)

    # ── HUD text ──
    text_id = p.addUserDebugText(
        "Gait: TROT",
        [0, 0, 0.18],
        textColorRGB=[0.2, 1.0, 0.4],
        textSize=1.5,
    )

    current_gait   = "trot"
    prev_trot_val  = 1
    prev_crab_val  = 0

    print("Simulation running!")
    print("  Use GUI sliders to tune speed/step height/reach.")
    print("  Press >> TROT or >> CRAB buttons to switch gaits.")
    print("  Close the window to quit.\n")

    t = 0.0
    try:
        while True:
            # ── read sliders ──
            speed      = p.readUserDebugParameter(speed_slider)
            step_h     = p.readUserDebugParameter(height_slider)
            step_r     = p.readUserDebugParameter(reach_slider)

            # ── check gait buttons ──
            trot_val = p.readUserDebugParameter(trot_btn)
            crab_val = p.readUserDebugParameter(crab_btn)

            if trot_val != prev_trot_val:
                current_gait = "trot"
                prev_trot_val = trot_val
                p.addUserDebugText("Gait: TROT", [0, 0, 0.18],
                                   textColorRGB=[0.2, 1.0, 0.4],
                                   textSize=1.5, replaceItemUniqueId=text_id)

            if crab_val != prev_crab_val:
                current_gait = "crab"
                prev_crab_val = crab_val
                p.addUserDebugText("Gait: CRAB WALK", [0, 0, 0.18],
                                   textColorRGB=[1.0, 0.6, 0.1],
                                   textSize=1.5, replaceItemUniqueId=text_id)

            # ── compute gait targets with live params ──
            if current_gait == "trot":
                targets = trot_targets(t, speed, step_h, step_r)
            else:
                targets = crab_targets(t, speed, step_h, step_r)

            apply_targets(robot_id, joint_map, targets)

            p.stepSimulation()
            t += SIM_STEP
            time.sleep(SIM_STEP)

    except p.error:
        print("PyBullet window closed.")


if __name__ == "__main__":
    main()