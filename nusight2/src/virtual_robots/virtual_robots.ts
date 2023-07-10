import { DirectNUClearNetClient } from "../server/nuclearnet/direct_nuclearnet_client";
import { FakeNUClearNetClient } from "../server/nuclearnet/fake_nuclearnet_client";

import { ChartSimulator } from "./simulators/chart_data_simulator";
import { OverviewSimulator } from "./simulators/overview_simulator";
import { ScriptDataSimulator } from "./simulators/script_data_simulator";
import { SensorsSimulator } from "./simulators/sensors_simulator";
import { VisionSimulator } from "./simulators/vision_simulator";
import { VirtualRobot } from "./virtual_robot";

export class VirtualRobots {
  private robots: VirtualRobot[];

  constructor({ robots }: { robots: VirtualRobot[] }) {
    this.robots = robots;
  }

  static of({
    fakeNetworking,
    nuclearnetAddress,
    numRobots,
  }: {
    fakeNetworking: boolean;
    nuclearnetAddress: string;
    numRobots: number;
  }) {
    const robots = Array.from({ length: numRobots }, (_, i) => {
      const nuclearnetClient = fakeNetworking ? FakeNUClearNetClient.of() : DirectNUClearNetClient.of();
      return VirtualRobot.of({
        name: `Virtual Robot ${i + 1}`,
        nuclearnetClient,
        nuclearnetAddress,
        simulators: [
          OverviewSimulator.of({ nuclearnetClient, robotIndex: i, numRobots }),
          SensorsSimulator.of({ nuclearnetClient, robotIndex: i, numRobots }),
          ChartSimulator.of({ nuclearnetClient }),
          VisionSimulator.of({ nuclearnetClient }),
          ScriptDataSimulator.of({ nuclearnetClient }),
        ],
      });
    });
    return new VirtualRobots({ robots });
  }

  start(): () => void {
    const stops = this.robots.map((robot) => robot.start());
    return () => stops.forEach((stop) => stop());
  }
}
