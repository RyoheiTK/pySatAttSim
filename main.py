import numpy as np
from math import *

# from quat import *

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *
#from core.FeedBackRWSetting import GainSetting

from core.Dynamics import Dynamics
from core.Satellite import Satellite
from core.GyroSetting import GyroSetting
from core.SttSetting import SttSetting
from core.Timer import Timer
from PIL import Image
from PIL import ImageOps

from core.SttSetting import SttSetting
from core.GyroSetting import GyroSetting
from core.RWSetting import RWSetting
from scipy.integrate import odeint
from core.Status import Status, StatusList, OutputStatusList
from core.Sensor import Gyro, STT
from core.Actuator import RWs
from core.Logic import FeedBackRW, EKFGyroSTT
from core.FeedBackRWSetting import FeedBackRWSetting
from core.EKFGyroSTTSetting import EKFGyroSTTSetting
from core.StatusVis import StatusVis
axis_verts = (
    (-7.5, 0.0, 0.0),
    (7.5, 0.0, 0.0),
    (0.0, -7.5, 0.0),
    (0.0, 7.5, 0.0),
    (0.0, 0.0, -7.5),
    (0.0, 0.0, 7.5),
)
axes = ((0, 1), (2, 3), (4, 5))
axis_colors = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)  # Red  # Green  # Blue


"""
       5____________6
       /           /|
      /           / |
    1/__________2/  |
    |           |   |
    |           |   |
    |           |   7
    |           |  /
    |           | /
    0___________3/
"""

cube_verts = (
    (-0.15, -0.2, 0.25),
    (-0.15, 0.2, 0.25),
    (0.15, 0.2, 0.25),
    (0.15, -0.2, 0.25),
    (-0.15, -0.2, -0.25),
    (-0.15, 0.2, -0.25),
    (0.15, 0.2, -0.25),
    (0.15, -0.2, -0.25),
    (-0.15, 0.7, 0.25),
    (0.15, 0.7, 0.25),
    (-0.15, -0.7, 0.25),
    (0.15, -0.7, 0.25),
)

cube_edges = (
    (0, 1),
    (0, 3),
    (0, 4),
    (2, 1),
    (2, 3),
    (2, 6),
    (5, 1),
    (5, 4),
    (5, 6),
    (7, 3),
    (7, 4),
    (7, 6),
    (8, 1),
    (8, 9),
    (9, 2),
    (10, 0),
    (10, 11),
    (11, 3),
)

cube_surfaces = (
    (0, 1, 2, 3),  # Front
    (3, 2, 6, 7),  # Right
    (7, 6, 5, 4),  # Left
    (4, 5, 1, 0),  # Back
    (1, 5, 6, 2),  # Top
    (4, 0, 3, 7),  # Bottom
    (1, 8, 9, 2),  # Sap1
    (0, 10, 11, 3),  # Sap2
)

cube_colors = (
    (1.0, 1.0, 1.0),  # White
    (1.0, 1.0, 1.0),  # White
    (1.0, 1.0, 1.0),  # White
    (0.769, 0.118, 0.227),  # Red
    (1.0, 1.0, 1.0),  # White
    (1.0, 1.0, 1.0),  # White
    (1.0, 1.0, 1.0),  # White
    (1.0, 1.0, 1.0),  # White
    (0.3, 0.835, 0.3),  # Yellow
    (0.3, 0.835, 0.3),  # Yellow
)


def Axis():
    glBegin(GL_LINES)
    for color, axis in zip(axis_colors, axes):
        glColor3fv(color)
        for point in axis:
            glVertex3fv(axis_verts[point])
    glEnd()


def Cube():
    glBegin(GL_QUADS)
    for color, surface in zip(cube_colors, cube_surfaces):
        glColor3fv(color)
        for vertex in surface:
            glVertex3fv(cube_verts[vertex])
    glEnd()

    glBegin(GL_LINES)
    glColor3fv((0.0, 0.0, 0.0))
    for edge in cube_edges:
        for vertex in edge:
            glVertex3fv(cube_verts[vertex])
    glEnd()


def main():
    step = -1
    intvl = 100
    imgs = []

    inertia = np.array([0.13, 0.26, 0.20])
    init_q = -np.array([1, 0.0, 0, 0])
    init_omega =  0.0*np.array([1, 1, 1]) * np.pi / 180  # 1 / 3.2])
    init_rw_vel = -0 * np.array([0.35, 0.35, 0.35]) / 0.000734
    k_omega = 0.5 *np.diag(inertia)
    kq = 0.07 * np.diag(inertia)
    kd_omega = 0.00 * np.diag(inertia)
    kqi = 0.001 * np.diag(inertia)
    #gainsetting = GainSetting(kq, kqi, k_omega, kd_omega)
    gyro_setting = GyroSetting(
        1.8/ 60 / 60 * 3 * np.random.randn(3) * 2 * np.pi / 180 * 1 / 10,
        0.8 / 60 / 60 * 3 * np.random.randn(3) * 2 * np.pi / 180,
        0.15 / 60 * 2 * np.pi / 180,
        1,
        0.05,
    )
    stt_setting = SttSetting(2 / 3600, 10 / 3600, 0.1)
    rw_setting = RWSetting(0.0007, 0.025, 6000 * 2 * np.pi / 60)
    rw_inertia = 0.0007
    dt = 0.0025
    feedbackrw_setting = FeedBackRWSetting(kq, kqi, k_omega, kd_omega, np.diag(inertia), rw_inertia, np.array([2**0.5/2,2**0.5/2,0,0]))
    ekfsetting = EKFGyroSTTSetting(2 / 3600, 10 / 3600, 0.005, 0.1, 1, inertia, rw_inertia, 0.005, 100)
    timer = Timer(dt)

    # Define Status
    q_t = Status("q_t", init_q)
    q_stt_obs = Status("q_stt_obs", init_q)
    w_t_rad_sec = Status("w_t_rad_sec", init_omega)
    w_bias_t_rad_sec = Status("w_bias_t_rad_sec", np.zeros(len(init_omega)))
    w_gyro_obs_rad_sec = Status("w_gyro_obs_rad_sec", init_omega)
    rw_vel_rad_sec = Status("rw_vel_rad_sec", init_rw_vel)
    torque_rw_dir_Nm = Status("torque_rw_dir_Nm", np.zeros(len(init_omega)))
    d_rw_vel_rad_sec = Status("d_rw_vel_rad_sec", np.zeros(len(init_omega)))
    torque_rw_t_Nm = Status("torque_rw_t_Nm", np.zeros(len(init_omega)))
    w_est_rad_sec = Status("w_est_rad_sec", np.zeros(len(init_omega)))
    w_bias_est_rad_sec = Status("w_bias_est_rad_sec", np.zeros(len(init_omega)))
    state_covariance = Status("state_covariance", np.ones([6, 6]) * 10)
    q_est = Status("q_est", init_q)
    evep = Status("evep", np.zeros(len(init_omega)))
    all_st = StatusList(
        [
            q_t,
            q_stt_obs,
            w_t_rad_sec,
            w_bias_t_rad_sec,
            w_gyro_obs_rad_sec,
            rw_vel_rad_sec,
            torque_rw_dir_Nm,
            d_rw_vel_rad_sec,
            torque_rw_t_Nm,
            w_est_rad_sec,
            w_bias_est_rad_sec,
            state_covariance,
            q_est,
            evep

        ]
    )
    # Define Components
    stt = STT(StatusList([q_t]), OutputStatusList([q_stt_obs]), stt_setting, timer)
    gyro = Gyro(
        StatusList([w_t_rad_sec]),
        OutputStatusList([w_bias_t_rad_sec, w_gyro_obs_rad_sec]),
        gyro_setting,
        timer,
    )
    rw = RWs(
        StatusList([torque_rw_dir_Nm]),
        OutputStatusList([rw_vel_rad_sec, d_rw_vel_rad_sec, torque_rw_t_Nm]),
        rw_setting,
        timer,
    )
    feedbackrw = FeedBackRW(
        StatusList([w_est_rad_sec, q_est, rw_vel_rad_sec]),
        OutputStatusList([torque_rw_dir_Nm, evep]),
        feedbackrw_setting,
        timer,
    )
    ekf = EKFGyroSTT(
        StatusList([torque_rw_t_Nm, rw_vel_rad_sec, q_stt_obs, w_gyro_obs_rad_sec]),
        OutputStatusList([q_est, w_est_rad_sec, w_bias_est_rad_sec, state_covariance]),
        ekfsetting,
        timer,
    )
    sat_output = OutputStatusList([q_t, w_t_rad_sec, torque_rw_t_Nm, rw_vel_rad_sec])

    sat1 = Satellite(inertia, sat_output, [stt, gyro, rw], [feedbackrw, ekf])
    time = 0
    calc_timer = 0
    dynamics = Dynamics(dt, sat1, rw_inertia)

    pygame.init()
    display = (1000, 818)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    # Using depth test to make sure closer colors are shown over further ones
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)

    # Default view
    glMatrixMode(GL_PROJECTION)
    gluPerspective(40, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(-0, -1, -8)
    glRotatef(90.0, 0.0, 1.0, 0.0)
    glRotatef(-90.0, 1.0, 0.0, 0.0)
    glRotatef(45.0, 0.0, 0.0, 1.0)

    time = 0

    while True:
        dynamics.update_time()
        sat1.tick()
        time = time + dt
        calc_timer += dt
        timer.update_time()
        step += 1

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                path_reserve = "./result"
                vis = StatusVis(all_st, dt, "test", path_reserve)
                vis.vis_history(["q_t", "q_stt_obs", "q_est"])
                vis.vis_history(["w_t_rad_sec", "w_gyro_obs_rad_sec", "w_est_rad_sec"])
                vis.vis_history(["torque_rw_t_Nm"])
                vis.vis_history(["rw_vel_rad_sec"])
                
                return imgs

            if event.type == pygame.KEYDOWN:
                # Rotating about the x axis
                if event.key == pygame.K_DOWN:
                    sat1.omega += np.array([0, 0.3, 0.3 / 3.2])
        q = sat1.status_list.get_status("q_t")
        x = q[1]  # -time*0.1
        y = q[2]  # time*0.1
        z = q[3]  # np.cos(time*0.1) #0.72-time*0.01
        w = q[0]  # +time*0.01

        mat4 = np.array(
            [
                [
                    1 - 2 * y * y - 2 * z * z,
                    2 * x * y - 2 * z * w,
                    2 * x * z + 2 * y * w,
                    0,
                ],
                [
                    2 * x * y + 2 * z * w,
                    1 - 2 * x * x - 2 * z * z,
                    2 * y * z - 2 * x * w,
                    0,
                ],
                [
                    2 * x * z - 2 * y * w,
                    2 * y * z + 2 * x * w,
                    1 - 2 * x * x - 2 * y * y,
                    0,
                ],
                [0, 0, 0, 1],
            ],
            "f",
        )

        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(mat4)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube()
        Axis()
        pygame.display.flip()
        if step % intvl != 0:
            continue
        else:
            pad_step = "{0:04d}".format(step)
            savepath = "img/tutorial3_" + pad_step + ".png"

            width = 1000  # glutGet(GLUT_WINDOW_WIDTH)
            height = 800  # glutGet(GLUT_WINDOW_HEIGHT)

            glReadBuffer(GL_FRONT)
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
            data = glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE)

            # image = Image.fromstring("RGBA", (width, height), data)
            image = Image.frombytes("RGBA", (width, height), data)
            image = ImageOps.flip(image)
            # image.save( savepath )
            imgs.append(image)
        # pygame.time.wait(10)


if __name__ == "__main__":
    imgs = main()
    imgs[0].save(
        "gif/no_momentum.gif",
        save_all=True,
        append_images=imgs[1:],
        optimize=False,
        duration=100,  # 40
        loop=0,
    )
    pygame.quit()
    quit()
