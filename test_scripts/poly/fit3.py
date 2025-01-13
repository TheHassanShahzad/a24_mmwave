import numpy as np
import matplotlib.pyplot as plt

# Example 2D list: [angular_velocity_R, angular_velocity_L] for each PWM
data = [[-18.786088964468103, -18.405290218100397], [-17.643692725364986, -17.643692725364986], [-17.389825646096423, -17.89755793209841], [-17.389825646096423, -17.77062439246413], [-17.009026899728717, -17.516759185730706], [-17.135960439363, -17.26289210646214], [-16.120495867359026, -17.009026899728717], [-15.612763581357038, -16.755161692995294], [-15.105031295355051, -16.374361074092448], [-14.597299009353064, -15.86662878809046], [-13.835700580350082, -15.612763581357038], [-13.201035690981383, -15.105031295355051], [-12.439437261978405, -14.470366405986352], [-11.677839769242993, -13.835700580350082], [-10.408509054238026, -12.693303404979398], [-9.26611187886734, -11.931704975976418], [-7.86984856049566, -10.662375197239019], [-6.473584773990196, -9.393044482234052], [-4.823455312617522, -7.615982417494666], [-3.300258922745346, -5.458120670120007], [-1.650129461372673, -3.1733258512448494], [0.0, -0.7615981949360882], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [1.9039956043736666, 0.0], [3.4271919942458426, 4.823455312617522], [5.077321455618516, 6.854383988491685], [6.600517845490692, 8.63144605323107], [8.123714703496653, 10.02771030787032], [9.519978021868333, 11.29704008660772], [10.916241340240013, 12.566370801612686], [11.931704975976418, 13.327969230615668], [13.074103087614674, 14.216500262985358], [14.089566723351076, 15.231964834989332], [14.851165152354056, 15.993562327724744], [15.485830041722757, 16.50129461372673], [16.120495867359026, 17.009026899728717], [16.50129461372673, 17.135960439363], [17.135960439363, 17.135960439363], [17.135960439363, 17.89755793209841], [17.77062439246413, 18.278356678466114], [18.278356678466114, 18.65915729736896], [18.405290218100397, 18.913022504102386], [18.278356678466114, 18.786088964468103], [19.547688329738655, 19.801553536472078]]


# Extract angular velocity for both motors
angular_velocity_R = [item[0] for item in data]
angular_velocity_L = [item[1] for item in data]

# PWM values from -250 to 250
pwm = np.arange(-250, 251, 10)

# Define dead zone boundaries for each motor
dead_zone_R_min = -38  # Right motor reverse dead zone limit
dead_zone_R_max = 52   # Right motor forward dead zone limit
dead_zone_L_min = -38  # Left motor reverse dead zone limit
dead_zone_L_max = 52   # Left motor forward dead zone limit

# Apply offsets for the right motor
pwm_R_effective_reverse = pwm[pwm < dead_zone_R_min] + abs(dead_zone_R_min)
pwm_R_effective_forward = pwm[pwm > dead_zone_R_max] - dead_zone_R_max

# Apply offsets for the left motor
pwm_L_effective_reverse = pwm[pwm < dead_zone_L_min] + abs(dead_zone_L_min)
pwm_L_effective_forward = pwm[pwm > dead_zone_L_max] - dead_zone_L_max

# Segment angular velocities for the right motor
angular_velocity_R_reverse = np.array(angular_velocity_R)[pwm < dead_zone_R_min]
angular_velocity_R_forward = np.array(angular_velocity_R)[pwm > dead_zone_R_max]

# Segment angular velocities for the left motor
angular_velocity_L_reverse = np.array(angular_velocity_L)[pwm < dead_zone_L_min]
angular_velocity_L_forward = np.array(angular_velocity_L)[pwm > dead_zone_L_max]

# Fit quadratic curves for the right motor
coeffs_R_reverse = np.polyfit(pwm_R_effective_reverse, angular_velocity_R_reverse, 2)
coeffs_R_forward = np.polyfit(pwm_R_effective_forward, angular_velocity_R_forward, 2)

# Fit quadratic curves for the left motor
coeffs_L_reverse = np.polyfit(pwm_L_effective_reverse, angular_velocity_L_reverse, 2)
coeffs_L_forward = np.polyfit(pwm_L_effective_forward, angular_velocity_L_forward, 2)

# Print coefficients
print("Right Motor Reverse Fit Coefficients:", coeffs_R_reverse)
print("Right Motor Forward Fit Coefficients:", coeffs_R_forward)
print("Left Motor Reverse Fit Coefficients:", coeffs_L_reverse)
print("Left Motor Forward Fit Coefficients:", coeffs_L_forward)

# Generate fitted curves for plotting
pwm_fit_reverse_R = np.linspace(pwm.min(), dead_zone_R_min, 100)
pwm_fit_forward_R = np.linspace(dead_zone_R_max, pwm.max(), 100)

pwm_fit_reverse_L = np.linspace(pwm.min(), dead_zone_L_min, 100)
pwm_fit_forward_L = np.linspace(dead_zone_L_max, pwm.max(), 100)

R_reverse_fit = np.polyval(coeffs_R_reverse, pwm_fit_reverse_R + abs(dead_zone_R_min))
R_forward_fit = np.polyval(coeffs_R_forward, pwm_fit_forward_R - dead_zone_R_max)

L_reverse_fit = np.polyval(coeffs_L_reverse, pwm_fit_reverse_L + abs(dead_zone_L_min))
L_forward_fit = np.polyval(coeffs_L_forward, pwm_fit_forward_L - dead_zone_L_max)

# Plot results
plt.figure(figsize=(12, 8))

# Right motor
plt.plot(pwm, angular_velocity_R, 'bo', label='Right Motor Data')
plt.plot(pwm_fit_reverse_R, R_reverse_fit, 'b--', label='Right Motor Reverse Fit')
plt.plot(pwm_fit_forward_R, R_forward_fit, 'b-', label='Right Motor Forward Fit')

# Left motor
plt.plot(pwm, angular_velocity_L, 'ro', label='Left Motor Data')
plt.plot(pwm_fit_reverse_L, L_reverse_fit, 'r--', label='Left Motor Reverse Fit')
plt.plot(pwm_fit_forward_L, L_forward_fit, 'r-', label='Left Motor Forward Fit')

# Dead zones
plt.axvspan(dead_zone_R_min, dead_zone_R_max, color='blue', alpha=0.1, label='Right Motor Dead Zone')
plt.axvspan(dead_zone_L_min, dead_zone_L_max, color='red', alpha=0.1, label='Left Motor Dead Zone')

plt.xlabel('PWM')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('PWM to Angular Velocity Mapping with Dead Zones')
plt.legend()
plt.grid()
plt.show()
