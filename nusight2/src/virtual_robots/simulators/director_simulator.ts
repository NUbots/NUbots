import { DirectorState, DirectorState_Provider_ClassificationEnum } from "@proto/message/behaviour/Director";
import { autorun } from "mobx";

import { SeededRandom } from "../../shared/base/random/seeded_random";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { Message, Simulator } from "../simulator";

import { periodic } from "./periodic";

export class DirectorSimulator extends Simulator {
  constructor(
    nuclearnetClient: NUClearNetClient,
    private readonly robotIndex: number,
    private readonly numRobots: number,
    private readonly random: SeededRandom,
  ) {
    super(nuclearnetClient);
  }

  static of({
    nuclearnetClient,
    robotIndex,
    numRobots,
  }: {
    nuclearnetClient: NUClearNetClient;
    robotIndex: number;
    numRobots: number;
  }) {
    return new DirectorSimulator(nuclearnetClient, robotIndex, numRobots, SeededRandom.of("director_simulator"));
  }

  start() {
    return autorun(() => this.send(this.directorState));
  }

  get directorState(): Message {
    const messageType = "message.behaviour.DirectorState";

    const time = periodic(0.33);

    // Cycle through different director states
    const phase = Math.floor(time) % 5;
    const plannerOn1 = phase === 1;
    const plannerOn2 = phase === 2;
    const plannersOn = plannerOn1 || plannerOn2;
    const kickOn = phase === 3;
    const reset = phase === 4;

    // Setup group and provider IDs for a robot
    const groupBase = 1000 + this.robotIndex * 100;
    const providerBase = 2000 + this.robotIndex * 100;

    const groupIds = {
      root: groupBase + 1,
      purpose: groupBase + 2,
      strategy: groupBase + 3,
      walkPlan: groupBase + 4,
      lookPlan: groupBase + 5,
      kickPlan: groupBase + 6,
      recovery: groupBase + 7,
    };

    const providerIds = {
      root: providerBase + 1,
      purpose: providerBase + 2,
      strategy: providerBase + 3,
      walkTo: providerBase + 4,
      turnOnSpot: providerBase + 5,
      look: providerBase + 6,
      kick: providerBase + 7,
      recovery: providerBase + 8,
    };

    // Pick an active provider for playing WalkPlanner
    const walkActiveProvider = plannersOn
      ? plannerOn1
        ? BigInt(providerIds.walkTo)
        : BigInt(providerIds.turnOnSpot)
      : 0n;

    // Some basic subtasks for the groups
    const rootSubtasks = [
      { name: "Soccer", targetGroup: BigInt(groupIds.purpose), priority: 10, optional: false },
      { name: "FallRecovery", targetGroup: BigInt(groupIds.recovery), priority: 5, optional: true },
    ];

    const purposeSubtasks = [
      { name: "Striker", targetGroup: BigInt(groupIds.strategy), priority: 10, optional: false },
      ...(!plannersOn && !kickOn
        ? []
        : [{ name: "StandStill", targetGroup: BigInt(groupIds.recovery), priority: 2, optional: true }]),
    ];

    const strategySubtasks = [
      ...(plannersOn
        ? [{ name: "WalkToBall", targetGroup: BigInt(groupIds.walkPlan), priority: 6, optional: false }]
        : []),
      ...(plannersOn
        ? [{ name: "LookAround", targetGroup: BigInt(groupIds.lookPlan), priority: 6, optional: false }]
        : []),
    ];

    // Build core groups map
    const groups: Record<number, Record<string, unknown>> = {
      [groupIds.root]: {
        type: "RootBehaviour",
        providerIds: [BigInt(providerIds.root)],
        activeProvider: BigInt(providerIds.root),
        parentProvider: 0n,
        subtasks: rootSubtasks,
      },
      [groupIds.purpose]: {
        type: "FindPurpose",
        providerIds: [BigInt(providerIds.purpose)],
        activeProvider: BigInt(providerIds.purpose),
        parentProvider: BigInt(providerIds.root),
        subtasks: purposeSubtasks,
      },
      [groupIds.strategy]: {
        type: "FindBall",
        providerIds: [BigInt(providerIds.strategy)],
        activeProvider: BigInt(providerIds.strategy),
        parentProvider: BigInt(providerIds.purpose),
        subtasks: strategySubtasks,
      },
      [groupIds.recovery]: {
        type: "FallRecovery",
        providerIds: [BigInt(providerIds.recovery)],
        activeProvider: BigInt(providerIds.recovery),
        parentProvider: BigInt(providerIds.root),
        subtasks: [{ name: "GetUp", targetGroup: BigInt(groupIds.recovery), priority: 6, optional: true }],
      },
    };

    // Build core providers map
    const providers: Record<number, Record<string, unknown>> = {
      [providerIds.root]: {
        id: BigInt(providerIds.root),
        group: BigInt(groupIds.root),
        classification: DirectorState_Provider_ClassificationEnum.ROOT,
        when: [],
        causing: {},
        needs: [],
      },
      [providerIds.purpose]: {
        id: BigInt(providerIds.purpose),
        group: BigInt(groupIds.purpose),
        classification: DirectorState_Provider_ClassificationEnum.PROVIDE,
        when: [],
        causing: {},
        needs: [],
      },
      [providerIds.strategy]: {
        id: BigInt(providerIds.strategy),
        group: BigInt(groupIds.strategy),
        classification: DirectorState_Provider_ClassificationEnum.PROVIDE,
        when: [],
        causing: {},
        needs: plannersOn ? [BigInt(groupIds.lookPlan), BigInt(groupIds.walkPlan)] : [],
      },
      [providerIds.recovery]: {
        id: BigInt(providerIds.recovery),
        group: BigInt(groupIds.recovery),
        classification: DirectorState_Provider_ClassificationEnum.START,
        when: [],
        causing: {},
        needs: [],
      },
    };

    // Build planner groups and providers if planners are on
    if (plannersOn) {
      groups[groupIds.walkPlan] = {
        type: "WalkTo",
        providerIds: [BigInt(providerIds.walkTo), BigInt(providerIds.turnOnSpot)],
        activeProvider: walkActiveProvider,
        parentProvider: BigInt(providerIds.strategy),
        subtasks: [{ name: "WalkTo", targetGroup: BigInt(groupIds.walkPlan), priority: 6, optional: true }],
      };

      groups[groupIds.lookPlan] = {
        type: "LookAround",
        providerIds: [BigInt(providerIds.look)],
        activeProvider: BigInt(providerIds.look),
        parentProvider: BigInt(providerIds.strategy),
        subtasks: [{ name: "LookAround", targetGroup: BigInt(groupIds.lookPlan), priority: 6, optional: true }],
      };

      providers[providerIds.walkTo] = {
        id: BigInt(providerIds.walkTo),
        group: BigInt(groupIds.walkPlan),
        classification: DirectorState_Provider_ClassificationEnum.START,
        when: [{ type: "Phase", comparator: "==", expectedState: { name: "PLAYING", value: 1 }, current: plannerOn1 }],
        causing: {},
        needs: [BigInt(groupIds.lookPlan)],
      };

      providers[providerIds.turnOnSpot] = {
        id: BigInt(providerIds.turnOnSpot),
        group: BigInt(groupIds.walkPlan),
        classification: DirectorState_Provider_ClassificationEnum.START,
        when: [{ type: "Phase", comparator: "==", expectedState: { name: "PLAYING", value: 1 }, current: plannerOn2 }],
        causing: {},
        needs: [BigInt(groupIds.lookPlan)],
      };

      providers[providerIds.look] = {
        id: BigInt(providerIds.look),
        group: BigInt(groupIds.lookPlan),
        classification: DirectorState_Provider_ClassificationEnum.START,
        when: [],
        causing: {},
        needs: [],
      };
    }

    // Build kick group and provider if kick is on
    if (kickOn) {
      groups[groupIds.kickPlan] = {
        type: "KickTo",
        providerIds: [BigInt(providerIds.kick)],
        activeProvider: BigInt(providerIds.kick),
        parentProvider: BigInt(providerIds.strategy),
        subtasks: [{ name: "KickTo", targetGroup: BigInt(groupIds.kickPlan), priority: 6, optional: true }],
      };

      providers[providerIds.kick] = {
        id: BigInt(providerIds.kick),
        group: BigInt(groupIds.kickPlan),
        classification: DirectorState_Provider_ClassificationEnum.START,
        when: [],
        causing: {},
        needs: [],
      };
    }

    // Reset to core only
    if (reset) {
      groups[groupIds.root].activeProvider = BigInt(providerIds.root);
      groups[groupIds.purpose].activeProvider = 0n;
      groups[groupIds.strategy].activeProvider = 0n;
      groups[groupIds.recovery].activeProvider = 0n;
    }

    const buffer = new DirectorState({
      groups,
      providers,
    }).toBinary();

    return { messageType, buffer };
  }
}
