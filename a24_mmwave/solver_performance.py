import numpy as np
import time

# Polynomial coefficients and offsets for both motors
r_reverse_cof = [0.00040997, 0.17173445, 0.13462213]
r_forward_cof = [-4.32870853e-04,  1.67879421e-01,  2.35361022e+00]
r_reverse_clip = -38
r_forward_clip = 52

l_reverse_cof = [0.00052984, 0.19408175, -0.14836343]
l_forward_cof = [-4.25785323e-04,  1.57314982e-01,  4.38386866e+00]
l_reverse_clip = -30
l_forward_clip = 52

# Function to solve the quadratic polynomial
def solve_poly(value, basic_cofs, offset):
    a, b, c = basic_cofs
    cofs = [a, (2 * a * -offset) + b, a * -offset**2 + b * -offset + c - value]
    roots = np.roots(cofs)
    closest_root = min(roots, key=abs)
    return closest_root

# Function to calculate PWM for both motors
def get_pwm(ang_vel_r, ang_vel_l):
    if ang_vel_r < 0:
        r_pwm = solve_poly(ang_vel_r, r_reverse_cof, r_reverse_clip)
    else:
        r_pwm = solve_poly(ang_vel_r, r_forward_cof, r_forward_clip)
    if ang_vel_l < 0:
        l_pwm = solve_poly(ang_vel_l, l_reverse_cof, l_reverse_clip)
    else:
        l_pwm = solve_poly(ang_vel_l, l_forward_cof, l_forward_clip)

    if r_pwm <= r_forward_clip and r_pwm >= r_reverse_clip:
        r_pwm = 0

    if l_pwm <= l_forward_clip and l_pwm >= l_reverse_clip:
        l_pwm = 0

    return [r_pwm, l_pwm]

# Generate 10,000 random angular velocity pairs
random_ang_vel_r = np.random.uniform(-15, 15, 10000)
random_ang_vel_l = np.random.uniform(-15, 15, 10000)

# Measure the time taken
start_time = time.time()

# Calculate PWMs for all pairs
pwms = [get_pwm(ang_vel_r, ang_vel_l) for ang_vel_r, ang_vel_l in zip(random_ang_vel_r, random_ang_vel_l)]

end_time = time.time()

# Print the results
print(f"Time taken for 10,000 PWM calculations: {end_time - start_time:.4f} seconds")
