with open("imu_kinematic_measurements.txt") as f:
    lines = f.readlines()

time = []
gyro = []
acc = []
contact = []
rotation = []
position = []
covariance = []

for line in lines:
    list_line = line.split()

    # An IMU line
    if list_line[0] == "IMU":
        # Only get the time with the IMU
        time.append(list_line[1])
        # Gyro and acc
        gyro.append([list_line[2], list_line[3], list_line[4]])
        acc.append([list_line[5], list_line[6], list_line[7]])

    # Foot touch sensors
    # Data is arranged with an id and a boolean value for that id
    # Two data points exist, one for each foot
    if list_line[0] == "CONTACT":
        contact.append([list_line[3], list_line[5]])

    # Kinematics includes the homogeneous transformation matrix for each foot
    # in the form of a quaternion and a 3d position vector
    # also includes a covariance matrix
    if list_line[0] == "KINEMATIC":
        for i in range(2, 90, 44):
            rotation.append(
                [list_line[i + 1], list_line[i + 2], list_line[i + 3], list_line[i + 4]]
            )
            position.append([list_line[i + 5], list_line[i + 6], list_line[i + 7]])
            c = []
            for j in range(6):
                for k in range(6):
                    c.append(list_line[i + 8 + j * 6 + k])
            covariance.append(c)

# Print all the data in the form of a yaml file
print(
    "initial_state: \n\
  orientation: [1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0] \n\
  velocity: [0.0, 0.0, 0.0] \n\
  position: [0.0, 0.0, 0.0] \n\
  gyro_bias: [0.0, 0.0, 0.0] \n\
  acc_bias: [0.0, 0.0, 0.0] \n"
)

print(
    "noise: \n\
  gyro: 0.01 \n\
  acc: 0.1 \n\
  gyro_bias: 0.00001 \n\
  acc_bias: 0.0001 \n\
  contact: 0.01 \n"
)

print("measurements:")

print("  time:")
for t in time:
    print("    -", t)

print("  gyroscope:")
for g in gyro:
    print("    -", g)

print("  accelerometer:")
for a in acc:
    print("    -", a)

print("  force_sensors:")
for c in contact:
    print("    -", c)

print("  kinematics_quaternion:")
for r in rotation:
    print("    -", r)

print("  kinematics_position:")
for p in position:
    print("    -", p)

print("  covariance:")
for c in covariance:
    print("    -", c)
