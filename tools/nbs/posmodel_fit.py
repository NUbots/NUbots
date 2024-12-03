import matplotlib.pyplot as plt
from utility.nbs import LinearDecoder
from tqdm import tqdm

def register(command):
    command.description = "Derive a position model for the walk engine"
    command.add_argument("files", metavar="files", nargs="+", help="The nbs files to merge and trim.")

def run(files, **kwargs):
    servo_current_positions = []

    # Expected servos
    expected_servos =["r_shoulder_pitch",
         "l_shoulder_pitch",
         "r_shoulder_roll",
         "l_shoulder_roll",
         "r_elbow",
         "l_elbow",
         "r_hip_yaw",
         "l_hip_yaw",
         "r_hip_roll",
         "l_hip_roll",
         "r_hip_pitch",
         "l_hip_pitch",
         "r_knee",
         "l_knee",
         "r_ankle_pitch",
         "l_ankle_pitch",
         "r_ankle_roll",
         "l_ankle_roll",
         "head_pan",
         "head_tilt"]

    timestamps = []
    expected_servos = {k: [] for k in expected_servos}

    decoder = LinearDecoder(*files, show_progress=True)
    # packets = tqdm(decoder, unit="packet", unit_scale=True, dynamic_ncols=True)

    prev_ts = 0

    for packet in decoder:
        print(packet.msg)
        # print(packet.type)
    #     if packet.type.name == "message.platform.RawSensors":
    #         curr_ts = packet.msg.timestamp.seconds * 1e9 + packet.msg.timestamp.nanos

    #         if prev_ts > curr_ts:
    #             print("prev ts > curr ts")
    #             exit(1)

    #         prev_ts = curr_ts

    #         timestamps.append(curr_ts)

    #         expected_servos["r_shoulder_pitch"].append(packet.msg.servo.r_shoulder_pitch)
    #         expected_servos["l_shoulder_pitch"].append(packet.msg.servo.l_shoulder_pitch)
    #         expected_servos["r_shoulder_roll"].append(packet.msg.servo.r_shoulder_roll)
    #         expected_servos["l_shoulder_roll"].append(packet.msg.servo.l_shoulder_roll)
    #         expected_servos["r_elbow"].append(packet.msg.servo.r_elbow)
    #         expected_servos["l_elbow"].append(packet.msg.servo.l_elbow)
    #         expected_servos["r_hip_yaw"].append(packet.msg.servo.r_hip_yaw)
    #         expected_servos["l_hip_yaw"].append(packet.msg.servo.l_hip_yaw)
    #         expected_servos["r_hip_roll"].append(packet.msg.servo.r_hip_roll)
    #         expected_servos["l_hip_roll"].append(packet.msg.servo.l_hip_roll)
    #         expected_servos["r_hip_pitch"].append(packet.msg.servo.r_hip_pitch)
    #         expected_servos["l_hip_pitch"].append(packet.msg.servo.l_hip_pitch)
    #         expected_servos["r_knee"].append(packet.msg.servo.r_knee)
    #         expected_servos["l_knee"].append(packet.msg.servo.l_knee)
    #         expected_servos["r_ankle_pitch"].append(packet.msg.servo.r_ankle_pitch)
    #         expected_servos["l_ankle_pitch"].append(packet.msg.servo.l_ankle_pitch)
    #         expected_servos["r_ankle_roll"].append(packet.msg.servo.r_ankle_roll)
    #         expected_servos["l_ankle_roll"].append(packet.msg.servo.l_ankle_roll)
    #         expected_servos["head_pan"].append(packet.msg.servo.head_pan)
    #         expected_servos["head_tilt"].append(packet.msg.servo.head_tilt)

    # # Create a 4x5 grid of subplots (adjust the rows and columns as desired)
    # fig, axes = plt.subplots(4, 5, figsize=(15, 10))

    # axes = axes.flatten()
    # ctr = 0

    # for k in expected_servos.keys():
    #     axes[ctr].plot(timestamps, expected_servos[k])
    #     axes[ctr].set_title(k)

    # plt.tight_layout()

    # plt.show()
