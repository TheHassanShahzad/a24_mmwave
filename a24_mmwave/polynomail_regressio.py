import numpy as np
import matplotlib.pyplot as plt
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.metrics import mean_squared_error

# Copy-paste your recorded data here
data = [
    [-18.786088964468103, -18.405290218100397],
    [-17.643692725364986, -17.643692725364986],
    [-17.389825646096423, -17.89755793209841],
    [-17.389825646096423, -17.77062439246413],
    [-17.009026899728717, -17.516759185730706],
    [-17.135960439363, -17.26289210646214],
    [-16.120495867359026, -17.009026899728717],
    [-15.612763581357038, -16.755161692995294],
    [-15.105031295355051, -16.374361074092448],
    [-14.597299009353064, -15.86662878809046],
    [-13.835700580350082, -15.612763581357038],
    [-13.201035690981383, -15.105031295355051],
    [-12.439437261978405, -14.470366405986352],
    [-11.677839769242993, -13.835700580350082],
    [-10.408509054238026, -12.693303404979398],
    [-9.26611187886734, -11.931704975976418],
    [-7.86984856049566, -10.662375197239019],
    [-6.473584773990196, -9.393044482234052],
    [-4.823455312617522, -7.615982417494666],
    [-3.300258922745346, -5.458120670120007],
    [-1.650129461372673, -3.1733258512448494],
    [0.0, -0.7615981949360882],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [1.9039956043736666, 0.0],
    [3.4271919942458426, 4.823455312617522],
    [5.077321455618516, 6.854383988491685],
    [6.600517845490692, 8.63144605323107],
    [8.123714703496653, 10.02771030787032],
    [9.519978021868333, 11.29704008660772],
    [10.916241340240013, 12.566370801612686],
    [11.931704975976418, 13.327969230615668],
    [13.074103087614674, 14.216500262985358],
    [14.089566723351076, 15.231964834989332],
    [14.851165152354056, 15.993562327724744],
    [15.485830041722757, 16.50129461372673],
    [16.120495867359026, 17.009026899728717],
    [16.50129461372673, 17.135960439363],
    [17.135960439363, 17.135960439363],
    [17.135960439363, 17.89755793209841],
    [17.77062439246413, 18.278356678466114],
    [18.278356678466114, 18.65915729736896],
    [18.405290218100397, 18.913022504102386],
    [18.278356678466114, 18.786088964468103],
    [19.547688329738655, 19.801553536472078],
]

# Extract PWM steps and velocities for the right and left wheels
pwm_steps = list(range(len(data)))
right_wheel_velocities = [row[0] for row in data]
left_wheel_velocities = [row[1] for row in data]

# Fit a polynomial model
degree = 5  # Degree of the polynomial
poly = PolynomialFeatures(degree)
X = np.array(pwm_steps).reshape(-1, 1)  # PWM steps as input
y_right = np.array(right_wheel_velocities)  # Right wheel velocities as output
y_left = np.array(left_wheel_velocities)  # Left wheel velocities as output

# Transform the data for polynomial regression
X_poly = poly.fit_transform(X)

# Fit models for the right and left wheels
model_right = LinearRegression()
model_right.fit(X_poly, y_right)
y_right_pred = model_right.predict(X_poly)

model_left = LinearRegression()
model_left.fit(X_poly, y_left)
y_left_pred = model_left.predict(X_poly)

# Calculate Mean Squared Error
mse_right = mean_squared_error(y_right, y_right_pred)
mse_left = mean_squared_error(y_left, y_left_pred)

# Print MSE
print(f"Right Wheel MSE: {mse_right}")
print(f"Left Wheel MSE: {mse_left}")

# Plot the original data and polynomial fit
plt.figure(figsize=(10, 6))

# Right Wheel Plot
plt.plot(pwm_steps, right_wheel_velocities, 'bo', label="Right Wheel Data", alpha=0.6)
plt.plot(pwm_steps, y_right_pred, 'b-', label=f"Right Wheel Fit (degree={degree})")

# Left Wheel Plot
plt.plot(pwm_steps, left_wheel_velocities, 'ro', label="Left Wheel Data", alpha=0.6)
plt.plot(pwm_steps, y_left_pred, 'r-', label=f"Left Wheel Fit (degree={degree})")

# Labels and Title
plt.xlabel("PWM Steps")
plt.ylabel("Angular Velocity (rad/s)")
plt.title("Polynomial Fit for PWM to Angular Velocity")
plt.legend()
plt.grid()

# Show Plot
plt.show()
