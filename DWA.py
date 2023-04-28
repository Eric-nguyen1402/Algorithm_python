import numpy as np

# Constants
MAX_SPEED = 0.5  # m/s
MAX_YAW_RATE = np.pi / 4  # rad/s
MAX_ACCEL = 0.2  # m/s^2
MAX_D_YAW_RATE = np.pi / 8  # rad/s^2
V_RESOLUTION = 0.01  # m/s
YAW_RATE_RESOLUTION = np.pi / 180.0  # rad/s
DT = 0.2  # s
PREDICTION_TIME = 2.0  # s
N_SAMPLES = 10  # number of paths to sample
GOAL_DISTANCE = 0.1  # m


def generate_velocity_profile(path):
    v_profile = []
    d = 0
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        ds = np.sqrt(dx**2 + dy**2)
        d += ds
        v = min(MAX_SPEED, d/(PREDICTION_TIME - i*DT))
        v_profile.append(v)
    return v_profile


def DWA(x, v, w, goal, ob):
    dw_range = np.arange(-MAX_D_YAW_RATE, MAX_D_YAW_RATE + YAW_RATE_RESOLUTION, YAW_RATE_RESOLUTION)
    v_range = np.arange(0, MAX_SPEED + V_RESOLUTION, V_RESOLUTION)

    best_traj = None
    min_cost = float("inf")

    for v_i in v_range:
        for dw_i in dw_range:
            traj = np.array([x])
            for _ in range(int(PREDICTION_TIME/DT)):
                x = motion(x, v_i, w+dw_i, DT)
                traj = np.vstack((traj, x))

            # calculate costs
            to_goal_cost = calc_to_goal_cost(traj[-1], goal)
            speed_cost = (MAX_SPEED - traj[-1][3])**2
            ob_cost = calc_obstacle_cost(traj, ob)

            # total cost
            cost = to_goal_cost + speed_cost + ob_cost

            # check if this trajectory is better than the best one so far
            if cost < min_cost:
                best_traj = traj
                min_cost = cost

    return best_traj, min_cost


def motion(x, v, w, dt):
    x[2] += w*dt
    x[0] += v*np.cos(x[2])*dt
    x[1] += v*np.sin(x[2])*dt
    x[3] = v
    return x


def calc_obstacle_cost(traj, ob):
    min_r = float("inf")
    for point in traj:
        for o in ob:
            d = np.sqrt((point[0]-o[0])**2 + (point[1]-o[1])**2)
            if d < min_r:
                min_r = d

    if min_r == float("inf"):
        return 0  # no obstacle

    return 1.0 / min_r  # inverse of the minimum distance


def calc_to_goal_cost(last_point, goal):
    d = np.sqrt((last_point[0]-goal[0])**2 + (last_point[1]-goal[1])**2)
    if d > GOAL_DISTANCE:
        return float("inf")  # not reached the goal
    return d  # distance to the goal

def main():
    # initialize the robot's current position and goal position
    start_pos = (0, 0)
    goal_pos = (10, 10)

    # plan the robot's path from start to goal
    path = plan_path(start_pos, goal_pos)

    # initialize the robot's velocity and acceleration limits
    max_speed = 1.0  # m/s
    max_accel = 0.5  # m/s^2

    # initialize the robot's initial velocity and initial heading angle
    init_speed = 0.0  # m/s
    init_heading = 0.0  # radians

    # generate the velocity profile for the robot's path
    velocity_profile = generate_velocity_profile(path, max_speed, max_accel, init_speed, init_heading)

    # initialize the robot's pose (position and heading angle) to the start position and heading angle
    pose = (start_pos[0], start_pos[1], init_heading)

    # initialize the robot's current velocity to the initial velocity
    vel = init_speed

    # loop through the velocity profile and use DWA to control the robot's motion
    for vel_cmd, ang_cmd in velocity_profile:
        # update the robot's velocity based on the commanded velocity
        vel += vel_cmd

        # clip the robot's velocity to the maximum speed
        vel = min(vel, max_speed)

        # update the robot's pose and velocity using DWA
        pose, vel = DWA(pose, vel, ang_cmd, max_speed, max_accel)

        # check if the robot has reached the goal position
        if np.linalg.norm(np.array(pose[:2]) - np.array(goal_pos)) < 0.1:
            print("Goal reached!")
            break
