"""Simulation 3D de l'hexapode (matplotlib).

    python -m sim.simulate                      # marche avant, fenêtre animée
    python -m sim.simulate --vx 0 --yaw 0.5     # rotation sur place
    python -m sim.simulate --save marche.gif    # export GIF (pillow requis)

Le robot est dessiné dans son propre repère : le sol défile sous lui.
"""

import argparse
import math

from hexapod import BodyPose, GaitCommand, Hexapod, default_config
from hexapod.body_ik import leg_to_body_frame, rotation_matrix
from hexapod.leg_ik import leg_joint_positions


def to_world(p, pose):
    r = rotation_matrix(pose.roll, pose.pitch, pose.yaw)
    return tuple(sum(r[i][j] * p[j] for j in range(3)) + (pose.x, pose.y, pose.z)[i] for i in range(3))


def skeleton(robot, angles):
    """Segments à dessiner : contour du corps et chaque patte, en repère monde."""
    c = robot.config
    body = [to_world((leg.mount_x, leg.mount_y, 0.0), robot.pose) for leg in c.legs]
    order = ["RF", "RM", "RR", "LR", "LM", "LF", "RF"]
    by_name = {leg.name: p for leg, p in zip(c.legs, body)}
    outline = [by_name[n] for n in order]
    legs = []
    for leg in c.legs:
        pts = leg_joint_positions(angles[leg.name], c.coxa_length, c.femur_length, c.tibia_length)
        legs.append([to_world(leg_to_body_frame(p, leg), robot.pose) for p in pts])
    return outline, legs


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--vx", type=float, default=80.0, help="vitesse avant (mm/s)")
    parser.add_argument("--vy", type=float, default=0.0, help="vitesse latérale (mm/s)")
    parser.add_argument("--yaw", type=float, default=0.0, help="vitesse de rotation (rad/s)")
    parser.add_argument("--sway", action="store_true", help="fait aussi osciller le corps (IK du corps)")
    parser.add_argument("--save", help="enregistre l'animation (ex. marche.gif)")
    args = parser.parse_args()

    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    robot = Hexapod(default_config())
    robot.command = GaitCommand(args.vx, args.vy, args.yaw)
    dt = 0.04

    fig = plt.figure(figsize=(7, 7))
    ax = fig.add_subplot(projection="3d")
    t = [0.0]

    def frame(_):
        t[0] += dt
        if args.sway:
            s = t[0]
            robot.pose = BodyPose(z=10 * math.sin(s), roll=0.12 * math.sin(1.3 * s), pitch=0.1 * math.cos(s))
        angles = robot.update(dt)
        outline, legs = skeleton(robot, angles)
        ax.cla()
        ax.set_xlim(-250, 250)
        ax.set_ylim(-250, 250)
        ax.set_zlim(-120, 120)
        ax.set_box_aspect((1, 1, 0.5))
        ax.set_xlabel("x (avant)")
        ax.set_ylabel("y (gauche)")
        ax.set_title(f"Démarche tripode, phase {robot.gait.phase:.2f}")
        ax.plot(*zip(*outline), color="black", linewidth=2)
        grounded = set(robot.gait.feet_on_ground())
        for leg, pts in zip(robot.config.legs, legs):
            color = "tab:blue" if leg.name in grounded else "tab:orange"
            ax.plot(*zip(*pts), marker="o", markersize=3, color=color)
        return []

    anim = FuncAnimation(fig, frame, frames=int(2 * robot.gait.period / dt) if args.save else None,
                         interval=dt * 1000, cache_frame_data=False)
    if args.save:
        anim.save(args.save, fps=int(1 / dt))
    else:
        plt.show()


if __name__ == "__main__":
    main()
