import { autorun } from "mobx";

import { SeededRandom } from "../../shared/base/random/seeded_random";
import { message } from "../../shared/messages";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { Message, Simulator } from "../simulator";

import { periodic } from "./periodic";

import DirectorState = message.behaviour.DirectorState;
import Classification = message.behaviour.DirectorState.Provider.Classification;

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
    const walkActiveProvider = plannersOn ? (plannerOn1 ? providerIds.walkTo : providerIds.turnOnSpot) : 0;

    // Some basic subtasks for the groups
    const rootSubtasks: DirectorState.DirectorTask[] = [
      { name: "Soccer", targetGroup: groupIds.purpose, priority: 10, optional: false },
      { name: "FallRecovery", targetGroup: groupIds.recovery, priority: 5, optional: true },
    ];

    const purposeSubtasks: DirectorState.DirectorTask[] = [
      { name: "Striker", targetGroup: groupIds.strategy, priority: 10, optional: false },
      ...(!plannersOn && !kickOn
        ? []
        : [{ name: "StandStill", targetGroup: groupIds.recovery, priority: 2, optional: true }]),
    ];

    const strategySubtasks: DirectorState.DirectorTask[] = [
      ...(plannersOn ? [{ name: "WalkToBall", targetGroup: groupIds.walkPlan, priority: 6, optional: false }] : []),
      ...(plannersOn ? [{ name: "LookAround", targetGroup: groupIds.lookPlan, priority: 6, optional: false }] : []),
    ];

    // Build core groups map
    const groups: Record<number, DirectorState.Group> = {
      [groupIds.root]: {
        type: "RootBehaviour",
        providerIds: [providerIds.root],
        activeProvider: providerIds.root,
        parentProvider: 0,
        subtasks: rootSubtasks,
      },
      [groupIds.purpose]: {
        type: "FindPurpose",
        providerIds: [providerIds.purpose],
        activeProvider: providerIds.purpose,
        parentProvider: providerIds.root,
        subtasks: purposeSubtasks,
      },
      [groupIds.strategy]: {
        type: "FindBall",
        providerIds: [providerIds.strategy],
        activeProvider: providerIds.strategy,
        parentProvider: providerIds.purpose,
        subtasks: strategySubtasks,
      },
      [groupIds.recovery]: {
        type: "FallRecovery",
        providerIds: [providerIds.recovery],
        activeProvider: providerIds.recovery,
        parentProvider: providerIds.root,
        subtasks: [{ name: "GetUp", targetGroup: groupIds.recovery, priority: 6, optional: true }],
      },
    };

    // Build core providers map
    const providers: Record<number, DirectorState.Provider> = {
      [providerIds.root]: {
        id: providerIds.root,
        group: groupIds.root,
        classification: Classification.ROOT,
        when: [],
        causing: {},
        needs: [],
      },
      [providerIds.purpose]: {
        id: providerIds.purpose,
        group: groupIds.purpose,
        classification: Classification.PROVIDE,
        when: [],
        causing: {},
        needs: [],
      },
      [providerIds.strategy]: {
        id: providerIds.strategy,
        group: groupIds.strategy,
        classification: Classification.PROVIDE,
        when: [],
        causing: {},
        needs: plannersOn ? [groupIds.lookPlan, groupIds.walkPlan] : [],
      },
      [providerIds.recovery]: {
        id: providerIds.recovery,
        group: groupIds.recovery,
        classification: Classification.START,
        when: [],
        causing: {},
        needs: [],
      },
    };

    // Build planner groups and providers if planners are on
    if (plannersOn) {
      groups[groupIds.walkPlan] = {
        type: "WalkTo",
        providerIds: [providerIds.walkTo, providerIds.turnOnSpot],
        activeProvider: walkActiveProvider,
        parentProvider: providerIds.strategy,
        subtasks: [{ name: "WalkTo", targetGroup: groupIds.walkPlan, priority: 6, optional: true }],
      };

      groups[groupIds.lookPlan] = {
        type: "LookAround",
        providerIds: [providerIds.look],
        activeProvider: providerIds.look,
        parentProvider: providerIds.strategy,
        subtasks: [{ name: "LookAround", targetGroup: groupIds.lookPlan, priority: 6, optional: true }],
      };

      providers[providerIds.walkTo] = {
        id: providerIds.walkTo,
        group: groupIds.walkPlan,
        classification: Classification.START,
        when: [{ type: "Phase", comparator: "==", expectedState: { name: "PLAYING", value: 1 }, current: plannerOn1 }],
        causing: {},
        needs: [groupIds.lookPlan],
      };

      providers[providerIds.turnOnSpot] = {
        id: providerIds.turnOnSpot,
        group: groupIds.walkPlan,
        classification: Classification.START,
        when: [{ type: "Phase", comparator: "==", expectedState: { name: "PLAYING", value: 1 }, current: plannerOn2 }],
        causing: {},
        needs: [groupIds.lookPlan],
      };

      providers[providerIds.look] = {
        id: providerIds.look,
        group: groupIds.lookPlan,
        classification: Classification.START,
        when: [],
        causing: {},
        needs: [],
      };
    }

    // Build kick group and provider if kick is on
    if (kickOn) {
      groups[groupIds.kickPlan] = {
        type: "KickTo",
        providerIds: [providerIds.kick],
        activeProvider: providerIds.kick,
        parentProvider: providerIds.strategy,
        subtasks: [{ name: "KickTo", targetGroup: groupIds.kickPlan, priority: 6, optional: true }],
      };

      providers[providerIds.kick] = {
        id: providerIds.kick,
        group: groupIds.kickPlan,
        classification: Classification.START,
        when: [],
        causing: {},
        needs: [],
      };
    }

    // Reset to core only
    if (reset) {
      groups[groupIds.root].activeProvider = providerIds.root;
      groups[groupIds.purpose].activeProvider = 0;
      groups[groupIds.strategy].activeProvider = 0;
      groups[groupIds.recovery].activeProvider = 0;
    }

    const buffer = DirectorState.encode({
      groups,
      providers,
    }).finish();

    return { messageType, buffer };
  }
}
