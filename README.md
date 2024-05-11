from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


# Constants
g = 9.81   # Gravitational acceleration (m/s^2)
m = 0.18   # Mass (kg)


# dx/dt = f(t, x)
# 
# t     : Current time (seconds), scalar
# x     : Current state, [z, vz]
# return: First derivative of state, [vz, az]
def xdot(t, x):
    # Desired z, vz, az
    z_des  = 1
    vz_des = 0
    az_des = 0

    # PD Controller (input, u)
    kp = 30
    kv = 3
    u  = m * (az_des + kp * (z_des - x[0]) + kv * (vz_des - x[1]) + g)
    
    # Clamp to actuator limits (0 to 2.12N)
    u = min(max(0, u), 2.12)
    
    # Quadrotor dynamics (dx/dt = xdot = [vz, az])
    return [x[1], u/m - g]


x0     = [0, 0] # Initial state, [z0, vz0]
t_span = [0, 5] # Simulation time (seconds), [from, to]


# Solve for the states, x(t) = [z(t), vz(t)]
sol = solve_ivp(xdot, t_span, x0)


# Plot z vs t
plt.plot(sol.t, sol.y[0], 'k-o')
plt.show()
