# Created by qian at 4/28/22

# Description:

import numpy as np

from pydrake.all import (SymbolicVectorSystem, DiagramBuilder, LogVectorOutput, Simulator, ConstantVectorSource,
                         MathematicalProgram, Solve, IpoptSolver, PiecewisePolynomial)

import world


def interpolate_init_guess(control_points, time_steps_list, time_interval):
    np.random.seed(0)

    state_guess = np.array([])
    for i in range(len(time_steps_list) - 1):
        time_limits = [time_steps_list[i] * time_interval, time_steps_list[i + 1] * time_interval]
        position_limits = np.column_stack((control_points[i], control_points[i + 1]))
        state_limits = np.vstack((position_limits, np.zeros([2, 2])))

        state = PiecewisePolynomial.FirstOrderHold(time_limits, state_limits)

        seg_guess = np.vstack(
            [state.value(t * time_interval).T for t in range(time_steps_list[i], time_steps_list[i + 1])])
        # state_guess += np.random.rand(*state_guess.shape) * 5e-6
        if state_guess.any():
            state_guess = np.vstack([state_guess, seg_guess])
        else:
            state_guess = seg_guess
        if i == len(time_steps_list) - 2:
            state_guess = np.append(state_guess, state.value(time_steps_list[-1] * time_interval).T, axis=0)

    return state_guess


def deviation_with_ref(self, state):
    x = state[0]
    y = state[1]
    if x <= 4:
        if y < 0.5:
            dev = np.abs(y)
        else:
            dev = np.abs(y - 1)
        if x < 0:
            dev = np.sqrt(x ** 2 + dev ** 2)
    else:
        dev = np.sqrt((x - 4) ** 2 + (y - 0.5) ** 2) - 0.5
    return dev * 0.1


# The state of segway is q = [x, y, heading, vel, angular_vel]
# torque is actually angular acceleration of two wheels. Just don't want to add more segway variables
def program_formulation(prog, state, torque, seg_world, time_interval, time_steps):
    # prog = MathematicalProgram()

    # initial state constraints
    for state_item in state[0]:
        prog.AddConstraint(state_item == 0)
    # terminate state (orientation can be non-zero)
    prog.AddConstraint(state[-1][0] == 0)
    prog.AddConstraint(state[-1][1] == 1)
    prog.AddConstraint(state[-1][3] == 0)
    prog.AddConstraint(state[-1][4] == 0)

    # discrete dynamics
    for t in range(time_steps):
        for residual in seg_world.segway_dynamics(state[t], state[t + 1], torque[t], time_interval):
            prog.AddConstraint(residual == 0)

    # initial guess
    segment_num = len(seg_world.control_points) - 1
    time_steps_list = [((time_steps * i) // segment_num) for i in range(segment_num + 1)]
    state_guess = interpolate_init_guess(seg_world.control_points, time_steps_list, time_interval)
    prog.SetInitialGuess(state, state_guess)

    # torque / acc limit
    for t in range(time_steps):
        prog.AddConstraint(torque[t][0] >= seg_world.segway.torque_limit[0])
        prog.AddConstraint(torque[t][0] <= seg_world.segway.torque_limit[1])
        prog.AddConstraint(torque[t][1] >= seg_world.segway.torque_limit[0])
        prog.AddConstraint(torque[t][1] <= seg_world.segway.torque_limit[1])

    # speed limit
    for t in range(time_steps):
        prog.AddConstraint(state[t][3] >= seg_world.segway.velocity_limit[0])
        prog.AddConstraint(state[t][3] <= seg_world.segway.velocity_limit[1])

    # obstacle
    for t in range(time_steps):
        for obs in seg_world.obstacles:
            dis = state[t][:2] - obs.position
            prog.AddConstraint(np.sum(dis ** 2) >= (obs.radius + seg_world.segway.safe_radius) ** 2)

    # distance to reference trajectory
    for t in range(time_steps):
        prog.AddConstraint(deviation_with_ref, lb=[0], ub=[1], vars=state[t])

    prog.AddCost(np.sum(torque ** 2) * time_interval)


if __name__ == '__main__':
    # define some variables
    time_steps = 100
    time_interval = 0.1  # sec
    seg_world = world.World(4)
    # seg_world.visualize()

    # optimization variables
    prog = MathematicalProgram()
    state = prog.NewContinuousVariables(time_steps + 1, 5, 'state')
    torque = prog.NewContinuousVariables(time_steps, 2, 'torque')
    # add constraints and costs
    program_formulation(prog, state, torque, seg_world, time_interval, time_steps)

    # solve
    solver = IpoptSolver()
    result = solver.Solve(prog)

    assert result.is_success()

    # solution
    torque_opt = result.GetSolution(torque)
    state_opt = result.GetSolution(state)
