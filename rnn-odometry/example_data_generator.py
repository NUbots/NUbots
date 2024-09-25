import numpy as np
import tensorflow as tf
from scipy.ndimage import gaussian_filter1d


class DataGenerator(tf.keras.utils.Sequence):
    def __init__(self, directories, batch_size=32, sequence_length=100, shuffle=True):
        self.directories = directories
        self.batch_size = batch_size
        self.sequence_length = sequence_length
        self.shuffle = shuffle
        self.on_epoch_end()

    def __len__(self):
        # Number of batches per epoch
        return int(np.floor(len(self.directories) * self.batch_size))

    def __getitem__(self, index):
        # Generate one batch of data
        batch_directories = self.directories[index * self.batch_size:(index + 1) * self.batch_size]
        imu, truth_all, truth_start_end_indicator = self.__data_generation(batch_directories)
        return imu, truth_all

    def on_epoch_end(self):
        # Shuffle directories after each epoch
        if self.shuffle:
            np.random.shuffle(self.directories)

    def __data_generation(self, batch_directories):
        # Generate data for the batch
        imu = []
        truth_all = []
        truth_start_end_indicator = []

        for directory in batch_directories:
            prefix = directory["prefix"]
            first_file = directory["first_file"]
            num_files = directory["num_files"]
            skip_files = directory["skip_files"]

            for i in range(first_file, num_files + 1):
                if i in skip_files:
                    continue

                imu_data = np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-imu-{i}.npy")
                imu.append(imu_data)

                truth_data = np.load(f"processed-outputs/numpy/{prefix}/{i}/{prefix}-truth-{i}.npy")
                truth_data = self.convert_to_relative(truth_data)
                truth_all.append(truth_data)

                chunk_size = truth_data.shape[0]
                indicator_chunk = np.zeros(chunk_size)
                indicator_chunk[0] = 1
                indicator_chunk[-1] = 1
                truth_start_end_indicator.append(indicator_chunk)

        imu = np.concatenate(imu, axis=0)
        truth_all = np.concatenate(truth_all, axis=0)
        truth_start_end_indicator = np.concatenate(truth_start_end_indicator, axis=0)

        # Apply Gaussian smoothing
        truth_all = self.gaussian_smooth(truth_all, window_size=50)

        # Normalize the data
        imu, truth_all = self.normalize_data(imu, truth_all)

        # Partition the data into sequences
        imu, truth_all = self.partition_dataset(imu, truth_all, self.sequence_length)

        return imu, truth_all

    def convert_to_relative(self, data):
        starting_position = data[0]
        relative_positions = data - starting_position
        return relative_positions

    def gaussian_smooth(self, data, window_size):
        smoothed_data = np.empty_like(data)
        for i in range(data.shape[1]):
            smoothed_data[:, i] = gaussian_filter1d(data[:, i], window_size, axis=0)
        return smoothed_data

    def normalize_data(self, imu, truth_all):
        mean = imu.mean(axis=0)
        std = imu.std(axis=0)
        imu = (imu - mean) / std
        truth_all = (truth_all - mean) / std
        return imu, truth_all

    def partition_dataset(self, features, targets, window_size):
        inputs = []
        outputs = []
        for i in range(0, len(features) - window_size):
            window = features[i:i + window_size]
            next_value = targets[i + window_size]
            inputs.append(window)
            outputs.append(next_value)
        return np.array(inputs), np.array(outputs)

# Example usage with Keras model
directories = [
    {"prefix": "quad", "first_file": 1, "num_files": 20, "skip_files": []},
    {"prefix": "s", "first_file": 1, "num_files": 20, "skip_files": []},
    {"prefix": "s-new", "first_file": 1, "num_files": 20, "skip_files": []},
    {"prefix": "zz", "first_file": 1, "num_files": 20, "skip_files": []},
    {"prefix": "circ", "first_file": 1, "num_files": 20, "skip_files": []},
    {"prefix": "sprl", "first_file": 1, "num_files": 20, "skip_files": [35]},
]

training_generator = DataGenerator(directories, batch_size=32, sequence_length=100, shuffle=True)

# Define your model (example)
model = tf.keras.Sequential([
    tf.keras.layers.LSTM(128, return_sequences=True, input_shape=(100, 6)),
    tf.keras.layers.Dropout(0.5),
    tf.keras.layers.LSTM(128, return_sequences=True),
    tf.keras.layers.Dropout(0.5),
    tf.keras.layers.LSTM(128),
    tf.keras.layers.Dropout(0.5),
    tf.keras.layers.Dense(2, activation='softmax')
])

# Compile the model
model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=0.0001), loss='categorical_crossentropy', metrics=['accuracy'])

# Train the model using the data generator
model.fit(training_generator, epochs=50)
