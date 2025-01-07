import numpy as np

r_reverse_cof = [0.00040997, 0.17173445, 0.13462213]
r_forward_cof = [-4.32870853e-04,  1.67879421e-01,  2.35361022e+00]
r_reverse_clip = -38
r_forward_clip = 52

l_reverse_cof = [ 0.00052984,  0.19408175, -0.14836343]
l_forward_cof = [-4.25785323e-04,  1.57314982e-01,  4.38386866e+00]
l_reverse_clip = -30
l_forward_clip = 52

def solve_poly(value, basic_cofs, offset):
    a, b, c = basic_cofs
    cofs = [a, (2*a*-offset)+b, a * -offset**2 + b * -offset + c - value] 

    roots = np.roots(cofs)
    # return roots
    # return -np.sign(value) * max(roots)

    closest_root = min(roots, key=abs)
    return closest_root

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

print(get_pwm(-10, -10))
print(type(get_pwm(-10, -10)[0]))
print(get_pwm(0.2857142857142857, 0.2857142857142857))
print(type(get_pwm(0.2857142857142857, 0.2857142857142857)[0]))

# print(solve_poly(-10, r_reverse_cof, r_reverse_clip))
# print(solve_poly(10, r_forward_cof, r_forward_clip))

# print(solve_poly(-10, l_reverse_cof, l_reverse_clip))
# print(solve_poly(10, l_forward_cof, l_forward_clip))