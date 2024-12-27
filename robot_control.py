# funktioniert so auch mit visualisierung
import argparse
import torch
import genesis as gs
import numpy as np


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--vis", action="store_true", default=False)
    args = parser.parse_args()

    ########################## init ##########################
    gs.init(backend=gs.cpu)

    ########################## create a scene ##########################

    scene = gs.Scene(
        sim_options=gs.options.SimOptions(),
        viewer_options=gs.options.ViewerOptions(
            camera_pos=(3.5, 0.0, 2.5),
            camera_lookat=(0.0, 0.0, 0.5),
            camera_fov=40,
        ),
        show_viewer=True,
        # show_viewer=args.vis,
        rigid_options=gs.options.RigidOptions(
            dt=0.01,
            gravity=(0.0, 0.0, -10.0),
        ),
    )

    ########################## entities ##########################
    plane = scene.add_entity(gs.morphs.Plane())
    franka = scene.add_entity(
        gs.morphs.MJCF(file="xml/franka_emika_panda/panda.xml"),
    )

    ########################## build ##########################
    scene.build()

    # Holt die jointnamen aus der MJCF/URDF Datei und mappt sie auf die DOFs
    jnt_names = [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "finger_joint1",
        "finger_joint2",
    ]
    dofs_idx = [franka.get_joint(name).dof_idx_local for name in jnt_names]

    # Optional: set control gains
    franka.set_dofs_kp(
        kp=np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100]),
        dofs_idx_local=dofs_idx,
    )
    franka.set_dofs_kv(
        kv=np.array([450, 450, 350, 350, 200, 200, 200, 10, 10]),
        dofs_idx_local=dofs_idx,
    )
    franka.set_dofs_force_range(
        lower=np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100]),
        upper=np.array([87, 87, 87, 87, 12, 12, 12, 100, 100]),
        dofs_idx_local=dofs_idx,
    )

    gs.tools.run_in_another_thread(fn=run_sim, args=(scene, franka, dofs_idx, args.vis))
    if args.vis:
        scene.viewer.start()


def run_sim(scene, franka, dofs_idx, enable_vis):
    from time import time

    t_prev = time()
    i = 0

    # Hard reset
    for _ in range(150):
        if i < 50:
            franka.set_dofs_position(
                np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]), dofs_idx
            )
        elif i < 100:
            franka.set_dofs_position(
                np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]), dofs_idx
            )
        else:
            franka.set_dofs_position(np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]), dofs_idx)

        scene.step()

    # PD control
    for i in range(1250):
        if i == 0:
            franka.control_dofs_position(
                np.array([1, 1, 0, 0, 0, 0, 0, 0.04, 0.04]),
                dofs_idx,
            )
        elif i == 250:
            franka.control_dofs_position(
                np.array([-1, 0.8, 1, -2, 1, 0.5, -0.5, 0.04, 0.04]),
                dofs_idx,
            )
        elif i == 500:
            franka.control_dofs_position(
                np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
                dofs_idx,
            )
        elif i == 750:
            franka.control_dofs_position(
                np.array([0, 0, 0, 0, 0, 0, 0, 0, 0])[1:],
                dofs_idx[1:],
            )
            franka.control_dofs_velocity(
                np.array([1.0, 0, 0, 0, 0, 0, 0, 0, 0])[:1],
                dofs_idx[:1],
            )
        elif i == 1000:
            franka.control_dofs_force(
                np.array([0, 0, 0, 0, 0, 0, 0, 0, 0]),
                dofs_idx,
            )

        print("control force:", franka.get_dofs_control_force(dofs_idx))
        print("internal force:", franka.get_dofs_force(dofs_idx))

        scene.step()

        t_now = time()
        print(1 / (t_now - t_prev), "FPS")
        t_prev = t_now

    if enable_vis:
        scene.viewer.stop()


if __name__ == "__main__":
    main()
