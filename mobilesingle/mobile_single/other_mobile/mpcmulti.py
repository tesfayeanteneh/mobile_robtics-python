import numpy as np
import matplotlib.pyplot as plt
from casadi import *

def dynamics(x, u, dt):
    """System dynamics for a differential drive robot."""
    return vertcat(x[0] + u[0] * cos(x[2]) * dt,
                   x[1] + u[0] * sin(x[2]) * dt,
                   x[2] + u[1] * dt)

def setup_mpc_problem(num_robots, N, dt, collision_distance):
    """Set up the MPC optimization problem."""
    X = [MX.sym(f'X_{i}', 3, N+1) for i in range(num_robots)]
    U = [MX.sym(f'U_{i}', 2, N) for i in range(num_robots)]
    P = MX.sym('P', 3*num_robots + 2*N*num_robots)
    
    cost = 0
    g = []
    
    for i in range(num_robots):
        g.append(X[i][:,0] - P[3*i:3*(i+1)])
        for k in range(N):
            st = X[i][:,k]
            con = U[i][:,k]
            st_next = dynamics(st, con, dt)
            ref = P[3*num_robots + 2*(i*N + k):3*num_robots + 2*(i*N + k + 1)]
            cost += mtimes((st_next[0:2] - ref).T, (st_next[0:2] - ref)) + 0.1 * mtimes(con.T, con)
            g.append(st_next - X[i][:,k+1])
            
            # Collision avoidance constraints
            for j in range(num_robots):
                if i != j:
                    g.append((X[i][0,k] - X[j][0,k])**2 + (X[i][1,k] - X[j][1,k])**2 - collision_distance**2)
    
    # Flatten state and control variables for optimization
    OPT_variables = vertcat(*[reshape(X[i], 3*(N+1), 1) for i in range(num_robots)] + [reshape(U[i], 2*N, 1) for i in range(num_robots)])
    
    # Define the NLP problem
    nlp_prob = {'f': cost, 'x': OPT_variables, 'g': vertcat(*g), 'p': P}
    
    # Solver options
    opts = {'ipopt.print_level': 0, 'print_time': 0, 'ipopt.tol': 1e-8}
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)
    
    return solver, P

def simulate_mpc(solver, P, x0_all, waypoints, num_robots, N, dt):
    """Simulate the MPC for multi-robot system."""
    x_mpc = [x0_all]
    u_mpc = []
    
    for t in range(len(waypoints) - N):
        ref = np.tile(waypoints[t:t+N], (num_robots, 1)).flatten()
        args = {
            'p': np.concatenate([x0_all, ref]),
            'lbg': np.zeros(3*(N+1)*num_robots + N*(num_robots*(num_robots-1))),
            'ubg': np.zeros(3*(N+1)*num_robots + N*(num_robots*(num_robots-1))),
            'lbx': -np.inf,
            'ubx': np.inf,
        }
        
        sol = solver(lbx=args['lbx'], ubx=args['ubx'], lbg=args['lbg'], ubg=args['ubg'], p=args['p'])
        u_opt = np.array(sol['x'][3*(N+1)*num_robots:3*(N+1)*num_robots + 2*N*num_robots]).reshape(num_robots, 2, N)
        x0_all = np.zeros(3*num_robots)
        for i in range(num_robots):
            x0_all[3*i:3*(i+1)] = dynamics(x0_all[3*i:3*(i+1)], u_opt[i,:,0], dt).full().flatten()
        
        x_mpc.append(x0_all)
        u_mpc.append(u_opt)
    
    return np.array(x_mpc), u_mpc

def plot_trajectories(x_mpc, waypoints, num_robots):
    """Plot the trajectories of the robots."""
    for i in range(num_robots):
        plt.plot(x_mpc[:, 3 * i], x_mpc[:, 3 * i + 1], label=f'Robot {i + 1}')
    plt.plot(waypoints[:, 0], waypoints[:, 1], 'ro', label='Waypoints')
    plt.xlim(0, 20)
    plt.ylim(0, 20)
    plt.grid(True)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

def main():
    num_robots = 3
    N = 10  # Prediction horizon
    dt = 0.1
    collision_distance = 0.5
    waypoints = np.array([[2, 1], [4, 0.5], [6, 1.5], [8, 0.5], [10, 1.5], [12, 0.5], [14, 1.5], [16, 0.5], [18, 2]])
    x0 = np.array([0, 0, 0])
    x0_all = np.tile(x0, num_robots)

    # Set up the MPC problem
    solver, P = setup_mpc_problem(num_robots, N, dt, collision_distance)

    # Simulate the MPC
    x_mpc, u_mpc = simulate_mpc(solver, P, x0_all, waypoints, num_robots, N, dt)

    # Plot the trajectories
    plot_trajectories(x_mpc, waypoints, num_robots)

if __name__ == "__main__":
    main()

        
