import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MultipleLocator
import math
import numpy as np

NX = 4  # [x, y, v, yaw]
NY = 4  # reference state variables
NYN = 4  # reference terminal state variables
NU = 2  # [accel, delta]

STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 1000.0  # max simulation time

TARGET_SPEED = 10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.1  # [s] step size(time tick)

# Vehicle parameters
LENGTH = 3.2  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.32  # [m] wheel-base

# Vehicle constraints
MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/s]

show_animation = True


# ------------------------------------------------------------
class State:
    """
    vehicle state class
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0, d=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.delta = d
        self.predelta = None


# ------------------------------------------------------------
def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def get_linear_model_matrix(v, phi, delta):
    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[3, 3] = 1.0
    A[0, 2] = DT * math.cos(phi)
    A[0, 3] = - DT * v * math.sin(phi)
    A[1, 2] = DT * math.sin(phi)
    A[1, 3] = DT * v * math.cos(phi)
    A[3, 2] = DT * math.tan(delta) / WB

    B = np.zeros((NX, NU))
    B[2, 0] = DT
    B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[3] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C


def plot_car(x, y, yaw, steer=0.0, cabcolor="-r", truckcolor="-k"):  # pragma: no cover

    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
                          -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fr_wheel[0, :]).flatten(),
             np.array(fr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten(),
             np.array(rr_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten(),
             np.array(fl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten(),
             np.array(rl_wheel[1, :]).flatten(), truckcolor)
    plt.plot(x, y, "*")


def update_state(state, a, delta, dt, predict):
    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / WB * math.tan(delta) * dt
    state.v = state.v + a * dt

    if state.v > MAX_SPEED:
        state.v = MAX_SPEED
    elif state.v < MIN_SPEED:
        state.v = MIN_SPEED

    return state


def get_nparray_from_matrix(x):
    return np.array(x).flatten()


def calc_nearest_index(state, cx, cy, cyaw, pind):
    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind


def smooth_yaw(yaw):
    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw


def animation_car(cx, cy, cyaw, ck, sp, dl, initial_state):
    """
    Simulation

    cx: course x position list
    cy: course y position list
    cy: course yaw position list
    ck: course curvature list
    sp: speed profile
    dl: course tick [m]

    """

    goal = [cx[-1], cy[-1]]

    state = initial_state

    # initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    time = 0.0
    nextPlotTime = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    d = [0.0]
    a = [0.0]

    # vref = [0.0]
    # yawref = [0.0]
    # delta = [0.0]
    # ex = [0.0]
    # ey = [0.0]
    # ldu = [0.0]

    target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

    odelta, oa = None, None

    cyaw = smooth_yaw(cyaw)

    while MAX_TIME >= time:
        # xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)

        x0 = [state.x, state.y, state.v, state.yaw]  # current state

        # oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

        odelta = [0.3] * 100
        oa = [0.1] * 100
        di, ai = odelta[0], oa[0]
        # if odelta is not None:
        # di, ai = odelta[0], oa[0]

        # for i in range(100):
        #  state = update_state(state, ai, di, 0.001, False)
        state = update_state(state, ai, di, DT, False)

        time = time + DT

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        d.append(di)
        a.append(ai)

        # yawref.append(pi_2_pi(cyaw[target_ind]))
        # vref.append(sp[len(v)-1])
        # vref.append(sp[target_ind])
        # delta.append(state.delta)
        # ldu.append(du*100)
        # ex.append(state.x-cx[target_ind])
        # ey.append(state.y-cy[target_ind])

        # isgoal = check_goal(state, goal, target_ind, len(cx))
        ox = None

        if show_animation and (time > nextPlotTime):  # pragma: no cover
            nextPlotTime = time + 10.0
            fig = plt.figure(1, figsize=(10, 5))
            plt.subplot(121)
            plt.cla()
            if ox is not None:
                plt.plot(ox, oy, "xr", label="MPC")
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            # plt.plot(xref[0, :], xref[1, :], "xk", label="xref")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plot_car(state.x, state.y, state.yaw, steer=di)
            plt.axis("equal")
            plt.grid(True)
            plt.title("Time[s]:" + str(round(time, 2))
                      + ", speed[km/h]:" + str(round(state.v * 3.6, 2)))
            ax = plt.subplot(122)
            ax.cla()
            # ax.plot(t, vref, "-c", label="vref")
            ax.plot(t, v, "-m", label="speed")
            # ax.plot(t, ex, "-g", label="err_x")
            # ax.plot(t, ey, "-b", label="err_y")
            # ax.plot(t, yawref, '-y', label='yawref')
            ax.plot(t, yaw, '-r', label='yaw')
            spacing = 0.1
            minorLocator = MultipleLocator(spacing)
            ax.yaxis.set_minor_locator(minorLocator)
            # ax.xaxis.set_minor_locator(minorLocator)
            ax.grid(which='minor')
            plt.legend()
            plt.xlabel("Time [s]")
            plt.ylabel("Speed [m/s], Error x/y [m]")
            plt.pause(0.0001)

        # if isgoal:
        # print("Goal")
        # break

    print('time over')
    plt.show()

    return t, x, y, yaw, v, d, a


# ------------------------------------------------------------
cx, cy, cyaw, ck, sp, dl = [0] * 100, [0] * 100, [0.785398] * 100, [0] * 100, [0] * 100, [0] * 100
initial_state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=2.0)
# animation_car(cx, cy, cyaw, ck, sp, dl, initial_state)
# ------------------------------------------------------------

state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=2.0)
#
# initial yaw compensation
if state.yaw - cyaw[0] >= math.pi:
    state.yaw -= math.pi * 2.0
elif state.yaw - cyaw[0] <= -math.pi:
    state.yaw += math.pi * 2.0

time = 0.0
nextPlotTime = 0.0
x = [state.x]
y = [state.y]
yaw = [state.yaw]
v = [state.v]
t = [0.0]
d = [0.0]
a = [0.0]

target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

odelta, oa = None, None

cyaw = smooth_yaw(cyaw)

while MAX_TIME >= time:
    # xref, target_ind, dref = calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, target_ind)

    x0 = [state.x, state.y, state.v, state.yaw]  # current state

    # oa, odelta, ox, oy, oyaw, ov = iterative_linear_mpc_control(xref, x0, dref, oa, odelta)

    odelta = [0.3] * 100
    oa = [0.0] * 100
    di, ai = odelta[0], oa[0]
    # if odelta is not None:
    # di, ai = odelta[0], oa[0]

    # for i in range(100):
    #  state = update_state(state, ai, di, 0.001, False)
    state = update_state(state, ai, di, DT, False)

    time = time + DT

    x.append(state.x)
    y.append(state.y)
    yaw.append(state.yaw)
    v.append(state.v)
    t.append(time)
    d.append(di)
    a.append(ai)

# ----------------------------------------------------------------------
# --------------------------- Animation ---------------------------------
cabcolor = "-r"
truckcolor = "-k"
# global x, y, yaw, v, t, d, a

# ----------------------------------------------------------------------
# Set up empty figure
fig = plt.figure()
line_outline, = plt.plot([], truckcolor)
line_fr_wheel, = plt.plot([], truckcolor)
line_rr_wheel, = plt.plot([], truckcolor)
line_fl_wheel, = plt.plot([], truckcolor)
line_rl_wheel, = plt.plot([], truckcolor)
line_center, = plt.plot([], "*")
plt.xlim(-20, 20)
plt.ylim(-10, 10)
plt.grid()


# plt.title('Time [s] and speed [km/h]')

def update(i):
    # update plot
    # Preperation for plotting the car ----------------------------------------
    steer = 0.0
    time = 0.0
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
                          -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw[i]), math.sin(yaw[i])],
                     [-math.sin(yaw[i]), math.cos(yaw[i])]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x[i]
    outline[1, :] += y[i]
    fr_wheel[0, :] += x[i]
    fr_wheel[1, :] += y[i]
    rr_wheel[0, :] += x[i]
    rr_wheel[1, :] += y[i]
    fl_wheel[0, :] += x[i]
    fl_wheel[1, :] += y[i]
    rl_wheel[0, :] += x[i]
    rl_wheel[1, :] += y[i]
    # --------------------------------------------------------------------------
    line_outline.set_data((np.array(outline[0, :]).flatten(), np.array(outline[1, :]).flatten()))
    line_fr_wheel.set_data((np.array(fr_wheel[0, :]).flatten(), np.array(fr_wheel[1, :]).flatten()))
    line_rr_wheel.set_data((np.array(rr_wheel[0, :]).flatten(), np.array(rr_wheel[1, :]).flatten()))
    line_fl_wheel.set_data((np.array(fl_wheel[0, :]).flatten(), np.array(fl_wheel[1, :]).flatten()))
    line_rl_wheel.set_data((np.array(rl_wheel[0, :]).flatten(), np.array(rl_wheel[1, :]).flatten()))
    line_center.set_data((x[i], y[i]))
    # ax.set_title('Time [s]: ' + str(time + i) + ', speed [km/h]:' + str(time + i))


anim = animation.FuncAnimation(fig, update, frames=1000, interval=20)

# anim.save('car_2.mp4', fps=30, extra_args=['-vcodec', 'libx264'])
plt.show()