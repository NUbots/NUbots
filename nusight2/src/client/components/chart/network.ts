import bounds from "binary-search-bounds";
import { action } from "mobx";

import { BrowserSystemClock } from "../../../client/time/browser_clock";
import { Vector2 } from "../../../shared/math/vector2";
import { message } from "../../../shared/messages";
import { Clock } from "../../../shared/time/clock";
import { toSeconds } from "../../../shared/time/timestamp";
import { Network } from "../../network/network";
import { NUsightNetwork } from "../../network/nusight_network";
import { RobotModel } from "../robot/model";

import { ChartModel } from "./model";
import { DataSeries } from "./model";
import { TreeData } from "./model";

import Sensors = message.input.Sensors;
import DataPoint = message.eye.DataPoint;

const ServoIds = [
  "Right Shoulder Pitch",
  "Left Shoulder Pitch",
  "Right Shoulder Roll",
  "Left Shoulder Roll",
  "Right Elbow",
  "Left Elbow",
  "Right Hip Yaw",
  "Left Hip Yaw",
  "Right Hip Roll",
  "Left Hip Roll",
  "Right Hip Pitch",
  "Left Hip Pitch",
  "Right Knee",
  "Left Knee",
  "Right Ankle Pitch",
  "Left Ankle Pitch",
  "Right Ankle Roll",
  "Left Ankle Roll",
  "Head Yaw",
  "Head Pitch",
];

export class ChartNetwork {
  constructor(private clock: Clock, private network: Network, private model: ChartModel) {
    this.network.on(DataPoint, this.onDataPoint);
    this.network.on(Sensors, this.onSensorData);
  }

  static of(nusightNetwork: NUsightNetwork, model: ChartModel): ChartNetwork {
    const network = Network.of(nusightNetwork);
    return new ChartNetwork(BrowserSystemClock, network, model);
  }

  destroy() {
    this.network.off();
  }

  @action
  private onDataPoint = (robotModel: RobotModel, data: DataPoint) => {
    if (data.value.length === 0) {
      return;
    }

    const basePath = [robotModel.name].concat(data.label.split("/"));
    const keys =
      data.value.length === 1
        ? [basePath.pop()]
        : data.value.length < 5
        ? ["x", "y", "z", "w"]
        : data.value.map((v, i) => `s${i}`);

    const node = basePath.reduce((accumulator: TreeData, p: string) => {
      if (!accumulator.has(p)) {
        accumulator.set(p, new Map<string, TreeData | DataSeries>());
      }
      return accumulator.get(p)! as TreeData;
    }, this.model.treeData);

    data.value.forEach((v, i) => {
      const key = keys[i]!;

      if (!node.has(key)) {
        // Create a new series with the start time of this datapoint
        node.set(key, DataSeries.of(toSeconds(data.timestamp)));
      }

      const leaf = node.get(key) as DataSeries;
      const series = leaf.series;

      // Now according to the chart timespace
      const chartTime = this.clock.now() - this.model.startTime;

      // Now according to the datapoint
      const pointTime = toSeconds(data.timestamp) - leaf.startTime;

      // Estimate the drifting distance between the clocks
      leaf.updateDelta(pointTime - chartTime);

      // Add the series element
      series.push(Vector2.of(pointTime, v));

      // Swap it backward until it's in place (keeping the list sorted)
      for (let i = series.length - 1; i > 0; i--) {
        if (series[i - 1].x > pointTime) {
          [series[i - 1], series[i]] = [series[i], series[i - 1]];
        } else {
          break;
        }
      }

      // Find where our old data starts so we can remove it
      const cutoff = chartTime + leaf.timeDelta - this.model.bufferSeconds;
      const newStart = bounds.lt(series, Vector2.of(), (p) => p.x - cutoff);

      // Remove old series elements in batches
      if (newStart > 50) {
        series.splice(0, newStart);
      }
    });
  };

  @action
  private onSensorData = (robotModel: RobotModel, sensorData: Sensors) => {
    const { accelerometer, gyroscope, battery, voltage, button, servo, feet } = sensorData;
    const timestamp = sensorData.timestamp!;

    if (accelerometer) {
      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/Accelerometer",
          value: [accelerometer.x!, accelerometer.y!, accelerometer.z!],
          timestamp,
        }),
      );
    }

    if (gyroscope) {
      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/Gyroscope",
          value: [gyroscope.x!, gyroscope.y!, gyroscope.z!],
          timestamp,
        }),
      );
    }

    if (battery) {
      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/Battery",
          value: [battery],
          timestamp,
        }),
      );
    }

    if (voltage) {
      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/CM740 Voltage",
          value: [voltage],
          timestamp,
        }),
      );
    }

    this.onDataPoint(
      robotModel,
      new DataPoint({
        label: "Sensor/Buttons",
        value: button.map((b) => (b.value ? 1 : 0)),
        timestamp,
      }),
    );

    if (feet.length == 2) {
      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/Foot Down/Right",
          value: [feet[0].down ? 1 : 0],
          timestamp,
        }),
      );

      this.onDataPoint(
        robotModel,
        new DataPoint({
          label: "Sensor/Foot Down/Left",
          value: [feet[1].down ? 1 : 0],
          timestamp,
        }),
      );
    }

    // Servos
    if (servo.length) {
      servo.forEach((servo: Sensors.IServo, index: number) => {
        const name = ServoIds[index];

        // PID gain
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Gain`,
            value: [servo.pGain!, servo.iGain!, servo.dGain!],
            timestamp,
          }),
        );

        // Goal position
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Position/Goal`,
            value: [servo.goalPosition!],
            timestamp,
          }),
        );

        // Goal Velocity
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Velocity/Goal`,
            value: [servo.goalVelocity!],
            timestamp,
          }),
        );

        // Present position
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Position/Present`,
            value: [servo.presentPosition!],
            timestamp,
          }),
        );

        // Present Velocity
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Velocity/Present`,
            value: [servo.presentVelocity!],
            timestamp,
          }),
        );

        // Load
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Load`,
            value: [servo.load!],
            timestamp,
          }),
        );

        // Voltage
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Voltage`,
            value: [servo.voltage!],
            timestamp,
          }),
        );

        // Temperature
        this.onDataPoint(
          robotModel,
          new DataPoint({
            label: `Sensor/Servos/${name}/Temperature`,
            value: [servo.temperature!],
            timestamp,
          }),
        );
      });
    }
  };
}
