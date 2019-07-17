#!/usr/bin/env python3
try:
    import os
    import tensorflow as tf
    from tensorflow import keras
    import yaml
    from tqdm import tqdm
    from util import nbs_decoder
    import numpy as np
    import re

    SERVO_ID = {
        "R_SHOULDER_PITCH": 0,
        "L_SHOULDER_PITCH": 1,
        "R_SHOULDER_ROLL": 2,
        "L_SHOULDER_ROLL": 3,
        "R_ELBOW": 4,
        "L_ELBOW": 5,
        "R_HIP_YAW": 6,
        "L_HIP_YAW": 7,
        "R_HIP_ROLL": 8,
        "L_HIP_ROLL": 9,
        "R_HIP_PITCH": 10,
        "L_HIP_PITCH": 11,
        "R_KNEE": 12,
        "L_KNEE": 13,
        "R_ANKLE_PITCH": 14,
        "L_ANKLE_PITCH": 15,
        "R_ANKLE_ROLL": 16,
        "L_ANKLE_ROLL": 17,
        "HEAD_YAW": 18,
        "HEAD_PITCH": 19,
    }

    def displacement(fk):

        # Sometimes the matrix is transposed so if x, y and z translation are 0, it's the bottom rather than the translation
        Htf = (
            np.array(
                [
                    [fk.x.x, fk.x.y, fk.x.z, fk.x.t],
                    [fk.y.x, fk.y.y, fk.y.z, fk.y.t],
                    [fk.z.x, fk.z.y, fk.z.z, fk.z.t],
                    [fk.t.x, fk.t.y, fk.t.z, fk.t.t],
                ]
            )
            if fk.t.x == 0 and fk.t.y == 0 and fk.t.z == 0
            else np.array(
                [
                    [fk.x.x, fk.y.x, fk.z.x, fk.t.x],
                    [fk.x.y, fk.y.y, fk.z.y, fk.t.y],
                    [fk.x.z, fk.y.z, fk.z.z, fk.t.z],
                    [fk.x.t, fk.y.t, fk.z.t, fk.t.t],
                ]
            )
        )

        return -(np.linalg.inv(Htf)[2, 3])

    def dataset(path, left_state, right_state, servos, fields, lr_duplicate, foot_delta, accelerometer, gryoscope):
        xs = []
        ys = []
        for type_name, timestamp, msg in tqdm(nbs_decoder.decode(path), dynamic_ncols=True, unit="packet"):
            if type_name == "message.input.Sensors":

                # Work out how far the foot is from the torso
                l_height = displacement(msg.forward_kinematics[SERVO_ID["L_ANKLE_ROLL"]])
                r_height = displacement(msg.forward_kinematics[SERVO_ID["R_ANKLE_ROLL"]])
                delta = abs(l_height - r_height)

                # Calculate what our foot down should be for each foot based on the state
                y = {
                    "R_": {"up": 0, "down": 1, "mixed": 1 if r_height - foot_delta < l_height else 0}[right_state],
                    "L_": {"up": 0, "down": 1, "mixed": 1 if l_height - foot_delta < r_height else 0}[left_state],
                }

                # If we are duplicating left right/right left do that here
                for sides in [("R_", "L_"), ("L_", "R_")] if lr_duplicate else [("R_", "L_")]:
                    x = []
                    for servo in servos:
                        for s in sides:
                            s = msg.servo[SERVO_ID[s + servo]]
                            values = {"LOAD": s.load, "POSITION": s.present_position, "VELOCITY": s.present_velocity}

                            for field in fields:
                                x.append(values[field])
                    if accelerometer:
                        # If we mirror the robot, it means we mirror the y axis
                        x.extend(
                            [
                                msg.accelerometer.x,
                                msg.accelerometer.y if sides[0] == "R_" else -msg.accelerometer.y,
                                msg.accelerometer.z,
                            ]
                        )
                    if gryoscope:
                        x.extend([msg.gyroscope.x, msg.gyroscope.y, msg.gyroscope.z])

                    # Swap the truth values if we are swap
                    ys.append((y[sides[0]], y[sides[1]]))
                    xs.append(x)

        return np.array(xs), np.array(ys)

    def register(command):

        # Install help
        command.help = "Train a foot down network using sensor data from the legs"

        # Drone arguments
        command.add_argument(
            "data_dir",
            metavar="data_dir",
            help="The foldering containing the segmented training data and configuration file",
        )

    def run(data_dir, **kwargs):

        # Load configuration file for the FootDownNetwork
        config_path = os.path.join(
            os.path.dirname(__file__),
            os.path.pardir,
            "module",
            "platform",
            "darwin",
            "SensorFilter",
            "data",
            "config",
            "FootDownNetwork.yaml",
        )
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)

            lr_duplicate = config["train"]["lr_duplicate"]
            foot_delta = config["train"]["foot_delta"]
            servos = config["network"]["input"]["servos"]
            fields = config["network"]["input"]["fields"]
            use_accel = config["network"]["input"]["accelerometer"]
            use_gyro = config["network"]["input"]["gyroscope"]

        print("Loading data from NBS files")
        group_xs = []
        group_ys = []
        group_desc = []
        for state in tqdm(os.listdir(data_dir), dynamic_ncols=True, unit="dir"):
            state_dir = os.path.join(data_dir, state)
            if os.path.isdir(state_dir):
                state_info = re.match("left_(up|down|mixed)_right_(up|down|mixed)", state)
                if state_info is not None:
                    l_state = state_info.group(1)
                    r_state = state_info.group(2)

                    for group in tqdm(os.listdir(state_dir), dynamic_ncols=True, unit="group"):
                        group_dir = os.path.join(state_dir, group)
                        if os.path.isdir(group_dir):

                            # Gather all groups into a single dataset for this specific type
                            xs = []
                            ys = []

                            for nbs in tqdm(os.listdir(group_dir), dynamic_ncols=True, unit="file"):
                                nbs_path = os.path.join(group_dir, nbs)
                                if nbs_path.endswith((".nbs.gz", ".nbs", ".nbz")):

                                    # Load the file
                                    x, y = dataset(
                                        nbs_path,
                                        r_state,
                                        l_state,
                                        servos,
                                        fields,
                                        lr_duplicate,
                                        foot_delta,
                                        use_accel,
                                        use_gyro,
                                    )

                                    # Cut off the end 10% to account for nonsense setup and teardown
                                    x = x[len(x) // 10 : -len(x) // 10]
                                    y = y[len(y) // 10 : -len(y) // 10]

                                    xs.append(x)
                                    ys.append(y)

                            group_desc.append(group)
                            group_xs.append(np.concatenate(xs, axis=0))
                            group_ys.append(np.concatenate(ys, axis=0))

        # Find the largest size we have and replicate random elements in the other dataset to fill
        mx = max([x.shape[0] for x in group_xs])

        # Oversample our categories so they are the same size
        xs = []
        ys = []
        for x, y in zip(group_xs, group_ys):
            idx = np.random.randint(0, len(x), mx)
            xs.append(x[idx])
            ys.append(y[idx])

        # Join into single dataset
        xs = np.concatenate(xs)
        ys = np.concatenate(ys)

        # Random shuffle the data
        idx = np.arange(len(xs))
        np.random.shuffle(idx)
        xs = xs[idx]
        ys = ys[idx]

        # Split into training and validation
        split = int(len(xs) * 0.8)
        train_x = xs[:split]
        train_y = ys[:split]
        valid_x = xs[split:]
        valid_y = ys[split:]

        # Build our model
        model = keras.Sequential(
            [keras.layers.Dense(8, activation=tf.nn.relu), keras.layers.Dense(2, activation=tf.nn.sigmoid)]
        )

        model.compile(
            optimizer=keras.optimizers.Adam(),
            loss=keras.losses.BinaryCrossentropy(),
            metrics=[keras.metrics.BinaryAccuracy()],
        )

        history = model.fit(
            train_x,
            train_y,
            batch_size=4096,
            epochs=1000,
            validation_data=(valid_x, valid_y),
            callbacks=[keras.callbacks.EarlyStopping(patience=5)],
        )

        print("Final Accuracy", history.history["val_binary_accuracy"][-1])

        for desc, x, y in zip(group_desc, group_xs, group_ys):
            print("Evaluating", desc)
            model.evaluate(x, y)

        config["network"] = {
            "input": {"servos": servos, "fields": fields, "accelerometer": use_accel, "gyroscope": use_gyro},
            "layers": [],
        }

        for layer in model.layers:
            h = layer.get_weights()

            weights = np.array(h[0]).tolist()
            biases = np.array(h[1]).tolist()

            config["network"]["layers"].append({"weights": weights, "biases": biases})

        with open(config_path, "w") as f:
            f.write(yaml.dump(config, width=120))


except:
    print("Unable to load footdown tool")

    def register(command):
        pass

    def run(**kwargs):
        pass
