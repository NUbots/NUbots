import minimist from "minimist";

import { DirectNUClearNetClient } from "../server/nuclearnet/direct_nuclearnet_client";
import { NUClearNetClient } from "../shared/nuclearnet/nuclearnet_client";

import { Simulator } from "./simulator";
import { ChartSimulator } from "./simulators/chart_data_simulator";
import { OverviewSimulator } from "./simulators/overview_simulator";
import { ScriptDataSimulator } from "./simulators/script_data_simulator";
import { SensorsSimulator } from "./simulators/sensors_simulator";
import { VirtualRobot } from "./virtual_robot";
import { VirtualRobots } from "./virtual_robots";

function main() {
  const args = minimist(process.argv.slice(2));
  const nuclearnetAddress = args.address || "10.1.255.255";

  const numRobots = 6;
  const virtualRobots = new VirtualRobots({
    robots: Array.from({ length: numRobots }, (_, i) => {
      const nuclearnetClient = DirectNUClearNetClient.of();
      return VirtualRobot.of({
        name: `Virtual Robot ${i + 1}`,
        nuclearnetClient,
        nuclearnetAddress,
        simulators: getSimulators(args, nuclearnetClient, i, numRobots),
      });
    }),
  });
  virtualRobots.start();
}

function getSimulators(
  args: minimist.ParsedArgs,
  nuclearnetClient: NUClearNetClient,
  robotIndex: number,
  numRobots: number,
): Simulator[] {
  const simulators = [];

  if (args.overview || args.all) {
    simulators.push(OverviewSimulator.of({ nuclearnetClient, robotIndex, numRobots }));
  }
  if (args.sensors || args.all) {
    simulators.push(SensorsSimulator.of({ nuclearnetClient, robotIndex, numRobots }));
  }
  if (args.chart || args.all) {
    simulators.push(ChartSimulator.of({ nuclearnetClient }));
  }
  if (args.scripts || args.all) {
    simulators.push(ScriptDataSimulator.of({ nuclearnetClient }));
  }
  if (simulators.length === 0) {
    // If no simulators given, enable them all.
    return getSimulators({ ...args, all: true }, nuclearnetClient, robotIndex, numRobots);
  }
  return simulators;
}

if (require.main === module) {
  main();
}
