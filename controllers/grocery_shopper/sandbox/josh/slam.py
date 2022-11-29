import math
import matplotlib.pyplot as plt
import numpy as np


class EKF:
    """
    Extended Kalman Filter SLAM example
    author: Atsushi Sakai (@Atsushi_twi)

    https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html
    """

    def __init__(self):
        # EKF state covariance
        self.Cx = np.diag([0.5, 0.5, np.deg2rad(30.0)]) ** 2 # Change in covariance

        #  Simulation parameter
        self.Q_sim = np.diag([0.2, np.deg2rad(1.0)]) ** 2 # Sensor Noise
        self.R_sim = np.diag([1.0, np.deg2rad(10.0)]) ** 2 # Process Noise

        self.DT         = 0.1  # time tick [s]
        self.SIM_TIME   = 50.0 # simulation time [s]
        self.MAX_RANGE  = 20.0 # maximum observation range
        self.M_DIST_TH  = 2.0  # Threshold of Mahalanobis distance for data association.
        self.STATE_SIZE = 3    # State size [x,y,yaw]
        self.LM_SIZE    = 2    # LM state size [x,y]



        # RFID positions [x, y]
        self.RFID = np.array([[10.0, -2.0],
                        [15.0, 10.0],
                        [3.0, 15.0],
                        [-5.0, 20.0]])

        # State Vector [x y yaw v]
        self.xEst = np.zeros((self.STATE_SIZE, 1))
        self.xTrue = np.zeros((self.STATE_SIZE, 1))
        self.PEst = np.eye(self.STATE_SIZE)
        self.xDR = np.zeros((self.STATE_SIZE, 1))  # Dead reckoning

        # history
        self.hxEst  = self.xEst
        self.hxTrue = self.xTrue
        self.hxDR   = self.xTrue



    def update(self, show_animation=True):
        u = self.calc_input()

        self.xTrue, z, self.xDR, ud = self.observation(self.xTrue, self.xDR, u, self.RFID)
        self.xEst, self.PEst = self.ekf_slam(self.xEst, self.PEst, ud, z)
        x_state = self.xEst[0:self.STATE_SIZE]

        # store data history
        self.hxEst = np.hstack((self.hxEst, x_state))
        self.hxDR = np.hstack((self.hxDR, self.xDR))
        self.hxTrue = np.hstack((self.hxTrue, self.xTrue))

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

            plt.plot(self.RFID[:, 0], self.RFID[:, 1], "*k")
            plt.plot(self.xEst[0], self.xEst[1], ".r")

            # plot landmark
            for i in range(self.calc_n_lm(self.xEst)):
                plt.plot(self.xEst[self.STATE_SIZE + i * 2],
                        self.xEst[self.STATE_SIZE + i * 2 + 1], "xg")

            plt.plot(self.hxTrue[0, :],
                    self.hxTrue[1, :], "-b")
            plt.plot(self.hxDR[0, :],
                    self.hxDR[1, :], "-k")
            plt.plot(self.hxEst[0, :],
                    self.hxEst[1, :], "-r")
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


    def ekf_slam(self, xEst, PEst, u, z):
        """
        Performs an iteration of EKF SLAM from the available information.
        
        :param xEst: the belief in last position
        :param PEst: the uncertainty in last position
        :param u:    the control function applied to the last position
        :param z:    measurements at this step
        :returns:    the next estimated position and associated covariance
        """
        # Predict
        S = self.STATE_SIZE
        G, Fx = self.jacob_motion(xEst[0:S], u)
        xEst[0:S] = self.motion_model(xEst[0:S], u)
        PEst[0:S, 0:S] = G.T @ PEst[0:S, 0:S] @ G + Fx.T @ self.Cx @ Fx
        initP = np.eye(2)

        # Update
        for iz in range(len(z[:, 0])):  # for each observation
            min_id = self.search_correspond_landmark_id(xEst, PEst, z[iz, 0:2])
            nLM = self.calc_n_lm(xEst)
            if min_id == nLM:
                # Extend state and covariance matrix
                xAug = np.vstack((xEst, self.calc_landmark_position(xEst, z[iz, :])))
                PAug = np.vstack((np.hstack((PEst, np.zeros((len(xEst), self.LM_SIZE)))),
                                np.hstack((np.zeros((self.LM_SIZE, len(xEst))), initP))))
                xEst = xAug
                PEst = PAug
            lm = self.get_landmark_position_from_state(xEst, min_id)
            y, S, H = self.calc_innovation(lm, xEst, PEst, z[iz, 0:2], min_id)

            K = (PEst @ H.T) @ np.linalg.inv(S)
            xEst = xEst + (K @ y)
            PEst = (np.eye(len(xEst)) - (K @ H)) @ PEst
        xEst[2] = self.pi_2_pi(xEst[2])
        return xEst, PEst

    def calc_input(self):
        v = 1.0  # [m/s]
        yaw_rate = 0.1  # [rad/s]
        u = np.array([[v, yaw_rate]]).T
        return u

    def observation(self, xTrue, xd, u, RFID):
        xTrue = self.motion_model(xTrue, u)
        # add noise to gps x-y
        z = np.zeros((0, 3))
        for i in range(len(RFID[:, 0])):
            dx = RFID[i, 0] - xTrue[0, 0]
            dy = RFID[i, 1] - xTrue[1, 0]
            d = math.hypot(dx, dy)
            angle = self.pi_2_pi(math.atan2(dy, dx) - xTrue[2, 0])
            if d <= self.MAX_RANGE:
                dn = d + np.random.randn() * self.Q_sim[0, 0] ** 0.5  # add noise
                angle_n = angle + np.random.randn() * self.Q_sim[1, 1] ** 0.5  # add noise
                zi = np.array([dn, angle_n, i])
                z = np.vstack((z, zi))
        # add noise to input
        ud = np.array([[
            u[0, 0] + np.random.randn() * self.R_sim[0, 0] ** 0.5,
            u[1, 0] + np.random.randn() * self.R_sim[1, 1] ** 0.5]]).T
        xd = self.motion_model(xd, ud)
        return xTrue, z, xd, ud

    def motion_model(self, x, u):
        F = np.array([[1.0, 0, 0],
                    [0, 1.0, 0],
                    [0, 0, 1.0]])
        B = np.array([
            [self.DT * math.cos(x[2, 0]), 0],
            [self.DT * math.sin(x[2, 0]), 0],
            [0.0, self.DT]])
        x = (F @ x) + (B @ u)
        return x

    def calc_n_lm(self, x):
        n = int((len(x) - self.STATE_SIZE) / self.LM_SIZE)
        return n

    def jacob_motion(self, x, u):
        Fx = np.hstack((np.eye(self.STATE_SIZE), np.zeros(
            (self.STATE_SIZE, self.LM_SIZE * self.calc_n_lm(x)))))
        jF = np.array([
            [0.0, 0.0, -self.DT * u[0, 0] * math.sin(x[2, 0])],
            [0.0, 0.0, self.DT * u[0, 0] * math.cos(x[2, 0])],
            [0.0, 0.0, 0.0]], dtype=float)
        G = np.eye(self.STATE_SIZE) + Fx.T @ jF @ Fx
        return G, Fx,

    def calc_landmark_position(self, x, z):
        zp = np.zeros((2, 1))
        zp[0, 0] = x[0, 0] + z[0] * math.cos(x[2, 0] + z[1])
        zp[1, 0] = x[1, 0] + z[0] * math.sin(x[2, 0] + z[1])
        return zp

    def get_landmark_position_from_state(self, x, ind):
        lm = x[self.STATE_SIZE + self.LM_SIZE * ind: self.STATE_SIZE + self.LM_SIZE * (ind + 1), :]
        return lm

    def search_correspond_landmark_id(self, xAug, PAug, zi):
        """Landmark association with Mahalanobis distance"""
        nLM = self.calc_n_lm(xAug)
        min_dist = []
        for i in range(nLM):
            lm = self.get_landmark_position_from_state(xAug, i)
            y, S, H = self.calc_innovation(lm, xAug, PAug, zi, i)
            min_dist.append(y.T @ np.linalg.inv(S) @ y)
        min_dist.append(self.M_DIST_TH)  # new landmark
        min_id = min_dist.index(min(min_dist))
        return min_id

    def calc_innovation(self, lm, xEst, PEst, z, LMid):
        delta = lm - xEst[0:2]
        q = (delta.T @ delta)[0, 0]
        z_angle = math.atan2(delta[1, 0], delta[0, 0]) - xEst[2, 0]
        zp = np.array([[math.sqrt(q), self.pi_2_pi(z_angle)]])
        y = (z - zp).T
        y[1] = self.pi_2_pi(y[1])
        H = self.jacob_h(q, delta, xEst, LMid + 1)
        S = H @ PEst @ H.T + self.Cx[0:2, 0:2]
        return y, S, H

    def jacob_h(self, q, delta, x, i):
        sq = math.sqrt(q)
        G = np.array([[-sq * delta[0, 0], - sq * delta[1, 0], 0, sq * delta[0, 0], sq * delta[1, 0]],
                    [delta[1, 0], - delta[0, 0], - q, - delta[1, 0], delta[0, 0]]])
        G = G / q
        nLM = self.calc_n_lm(x)
        F1 = np.hstack((np.eye(3), np.zeros((3, 2 * nLM))))
        F2 = np.hstack((np.zeros((2, 3)), np.zeros((2, 2 * (i - 1))),
                        np.eye(2), np.zeros((2, 2 * nLM - 2 * i))))
        F = np.vstack((F1, F2))
        H = G @ F
        return H

    def pi_2_pi(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi