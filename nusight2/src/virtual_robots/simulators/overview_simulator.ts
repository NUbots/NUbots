import { autorun } from "mobx";

import { SeededRandom } from "../../shared/base/random/seeded_random";
import { FieldDimensions } from "../../shared/field/dimensions";
import { Vector2 } from "../../shared/math/vector2";
import { Vector3 } from "../../shared/math/vector3";
import { Ivec2, message } from "../../shared/messages";
import { NUClearNetClient } from "../../shared/nuclearnet/nuclearnet_client";
import { toTimestamp } from "../../shared/time/timestamp";
import { Message, Simulator } from "../simulator";

import { periodic } from "./periodic";

import Mode = message.input.GameState.Data.Mode;
import PenaltyReason = message.input.GameState.Data.PenaltyReason;
import Phase = message.input.GameState.Data.Phase;
import Overview = message.support.nusight.Overview;

export class OverviewSimulator extends Simulator {
  constructor(
    nuclearnetClient: NUClearNetClient,
    private readonly robotIndex: number,
    private readonly numRobots: number,
    private readonly field: FieldDimensions,
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
    return new OverviewSimulator(
      nuclearnetClient,
      robotIndex,
      numRobots,
      FieldDimensions.of(),
      SeededRandom.of("overview_simulator"),
    );
  }

  start() {
    return autorun(() => this.send(this.overview));
  }

  get overview(): Message {
    const time = periodic(2);

    const messageType = "message.support.nusight.Overview";

    const t = time / 10 - this.robotIndex;

    const fieldLength = this.field.fieldLength;
    const fieldWidth = this.field.fieldWidth;

    const robotPosition = this.figureEight(t, fieldLength / 2, fieldWidth / 2);

    const ballPosition = this.figureEight(t, fieldLength / 4, fieldWidth / 4);

    const robotHeading = ballPosition.subtract(robotPosition);
    // TODO (Annable): Add helper for getting the angle for a unit vector.
    const robotAngle = Math.atan2(robotHeading.y, robotHeading.x);

    const modes = getEnumValues<Mode>(Mode);
    const phases = getEnumValues<Phase>(Phase);
    const penaltyReasons = getEnumValues<PenaltyReason>(PenaltyReason);

    const buffer = Overview.encode({
      timestamp: toTimestamp(time),
      robotId: this.robotIndex + 1,
      roleName: "Overview Simulator",
      battery: this.random.float(),
      voltage: this.randomFloat(10, 13),
      robotPosition: new Vector3(robotPosition.x, robotPosition.y, robotAngle),
      robotPositionCovariance: {
        x: { x: this.random.float(), y: this.random.float(), z: this.random.float() },
        y: { x: this.random.float(), y: this.random.float(), z: this.random.float() },
        z: { x: this.random.float(), y: this.random.float(), z: this.random.float() },
      },
      ballPosition,
      ballPositionCovariance: {
        x: { x: this.random.float(), y: this.random.float() },
        y: { x: this.random.float(), y: this.random.float() },
      },
      kickTarget: this.figureEight(-t).add(ballPosition),
      gameMode: this.random.choice(modes),
      gamePhase: this.random.choice(phases),
      penaltyReason: this.random.choice(penaltyReasons),
      lastCameraImage: toTimestamp(this.randomSeconds(time, -5)),
      lastSeenBall: toTimestamp(this.randomSeconds(time, -30)),
      lastSeenGoal: toTimestamp(this.randomSeconds(time, -30)),
      walkCommand: {
        x: Math.cos(time / 3 + this.robotIndex),
        y: Math.cos(time / 5 + this.robotIndex),
        z: Math.cos(time / 7 + this.robotIndex),
      },
    }).finish();

    const message = { messageType, buffer };

    return message;
  }

  private randomFieldPosition(): Ivec2 {
    const fieldLength = this.field.fieldLength;
    const fieldWidth = this.field.fieldWidth;
    return {
      x: this.random.float() * fieldLength - fieldLength * 0.5,
      y: this.random.float() * fieldWidth - fieldWidth * 0.5,
    };
  }

  private randomFloat(min: number, max: number): number {
    return this.random.float() * (max - min) + min;
  }

  private randomSeconds(now: number, offset: number): number {
    return now + offset * this.random.float();
  }

  private figureEight(t: number, scaleX: number = 1, scaleY: number = 1) {
    return new Vector2(scaleX * Math.cos(t), scaleY * Math.sin(2 * t));
  }
}

function getEnumValues<T>(enumObject: any): T[] {
  return Object.keys(enumObject).map((key) => enumObject[key]) as T[];
}
