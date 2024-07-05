from acados_template import AcadosOcp, AcadosOcpSolver, AcadosSimSolver
from robot_model import export_robot_model
import numpy as np
import scipy.linalg
from create_ocp_solver_description import create_ocp_solver_description
from utils import plot_robot
import matplotlib.pyplot as plt

#X0 = np.array([1.0, 0.0, 1.56])  # Intital state
X0 = np.array([1.1, 0.0, 2.0])  # Intital state
v_max = 1  # Define the max linear speed allowed
w_max = 1 # Define the max angle speed allowed
T_horizon = 2.0  # Define the prediction horizon

def generate_circle_trajectory(radius, num_points):
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x_coords = radius * np.cos(angles)
    y_coords = radius * np.sin(angles)
    #theta_coords = np.unwrap(angles + np.pi/2)
    return np.vstack((x_coords, y_coords)).T
    #return np.vstack((x_coords, y_coords, theta_coords)).T


def open_loop_simulation():

    # create solvers
    ocp = create_ocp_solver_description()
    model = ocp.model
    acados_ocp_solver = AcadosOcpSolver(ocp)
    acados_integrator = AcadosSimSolver(ocp)

    N_horizon = acados_ocp_solver.N

    # prepare simulation
    Nsim = 100
    nx = ocp.model.x.rows()
    nu = ocp.model.u.rows()

    simX = np.zeros((Nsim + 1, nx))
    simU = np.zeros((Nsim, nu))

    xcurrent = X0
    simX[0, :] = xcurrent

    #set ref states
    circle_points = generate_circle_trajectory(1, Nsim)
    yref_values = np.hstack([circle_points, np.zeros((Nsim, 3))])
    yref_N_values = np.hstack([circle_points, np.zeros((Nsim, 1))])
    #yref_N_values = circle_points
    #yref = np.array([1, 1, 0, 0, 0])
    #yref_N = np.array([1, 1, 0])

    # initialize solver
    for stage in range(N_horizon + 1):
        acados_ocp_solver.set(stage, "x", 0.0 * np.ones(xcurrent.shape))
    for stage in range(N_horizon):
        acados_ocp_solver.set(stage, "u", np.zeros((nu,)))

    # closed loop
    for i in range(Nsim):
        for j in range(N_horizon):
            acados_ocp_solver.set(j, "yref", yref_values[(i + j)% Nsim])
        acados_ocp_solver.set(N_horizon, "yref", yref_N_values[(i + j)% Nsim])

        # solve ocp
        simU[i, :] = acados_ocp_solver.solve_for_x0(xcurrent)
        status = acados_ocp_solver.get_status()

        if status not in [0, 2]:
            acados_ocp_solver.print_statistics()
            plot_robot(
                np.linspace(0, T_horizon / N_horizon * i, i + 1),
                [v_max, w_max],
                simU[:i, :],
                simX[: i + 1, :],
            )
            raise Exception(
                f"acados acados_ocp_solver returned status {status} in closed loop instance {i} with {xcurrent}"
            )

        # simulate system
        xcurrent = acados_integrator.simulate(xcurrent, simU[i, :])
        simX[i + 1, :] = xcurrent

    # plot results
    plot_robot(
        np.linspace(0, T_horizon / N_horizon * Nsim, Nsim + 1), [v_max, w_max], simU, simX,
        x_labels=model.x_labels, u_labels=model.u_labels, time_label=model.t_label
    )
    
    # plot traj
    plt.figure()
    plt.plot(circle_points[:, 0], circle_points[:, 1], 'r--', label='Desired Trajectory')
    plt.plot(simX[:, 0], simX[:, 1], 'b-', label='Actual Trajectory')
    points = plt.scatter(simX[:, 0], simX[:, 1], c=range(len(simX[:, 0])), cmap='viridis', label='Actual Trajectory Points')  # 使用色彩映射
    plt.colorbar(points)  # 添加颜色条
    plt.legend()
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Comparison of Desired and Actual Trajectories')
    plt.grid(True)
    plt.show()



open_loop_simulation()
