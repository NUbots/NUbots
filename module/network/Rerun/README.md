# Rerun

## Description

This module allows for sending data from NUbots to [Rerun](https://www.rerun.io/) for visualization in real time. It listens for `DataPoint` messages (what we use for plotting in NUsight), transforms them to a suitable format, and sends them to Rerun for visualization.

## Usage

To visualize data from a role you're working on using Rerun, do the following:

1. Add the `network::Rerun` module to the role you're working on.
2. Update the `Rerun.yaml` config file:
   - Set `forward_datapoints` to true to enable forwarding of data to Rerun.
   - Set `recording_name` to a suitable name for your recording.

3. In your role, anywhere you want to visualize data:

   - Include the `graph()` helper:

     ```cpp
     // In the includes section
     #include "utility/nusight/NUhelpers.hpp"

     // In the module namespace, to use `graph()` without specifying the namespace prefix every time
     using utility::nusight::graph;
     ```

   - Call `graph()` with the label and values you want to visualize, and `emit()` the result:

     ```cpp
     // Visualizing a single value
     emit(graph("Distance to ball", distance_to_ball));

     // 2-4 values will be labelled "x", "y", "z", "w" automatically
     emit(graph("Gyro", my_gyro_data.x(), my_gyro_data.y(), my_gyro_data.z()));

     // 5 or more values will be labelled "s0", "s1", "s2", ..., "s[n-1]" automatically
     emit(graph("My data", s0, s1, s2, s3, s4));
     ```

4. Build and run your role.
5. Open the Rerun viewer in NUsight2 to visualize the data.

> **Note**
> If you're not seeing data in the Rerun viewer, double-check that `Rerun.yaml` has the right configuration and your role is sending data. You can also set the `send_debug_data` config option to `true` to send sample data to Rerun to test the connection.

## Consumes

- `message::eye::DataPoint` - listens for these from the rest of the codebase, transforming and forwarding them to Rerun

## Emits

- Rerun-formatted data to the Rerun viewer

## Dependencies
