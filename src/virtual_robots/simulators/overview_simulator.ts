import { Vector2 } from '../../client/math/vector2'
import { Vector3 } from '../../client/math/vector3'
import { SeededRandom } from '../../shared/base/random/seeded_random'
import { FieldDimensions } from '../../shared/field/dimensions'
import { message } from '../../shared/proto/messages'
import { vec2$Properties } from '../../shared/proto/messages'
import { Simulator } from '../simulator'
import { Message } from '../simulator'
import State = message.behaviour.Behaviour.State
import Mode = message.input.GameState.Data.Mode
import PenaltyReason = message.input.GameState.Data.PenaltyReason
import Phase = message.input.GameState.Data.Phase
import Overview = message.support.nubugger.Overview

export class OverviewSimulator implements Simulator {
  constructor(private field: FieldDimensions,
              private random: SeededRandom) {
  }

  static of() {
    return new OverviewSimulator(
      FieldDimensions.postYear2017(),
      SeededRandom.of('overview_simulator'),
    )
  }

  simulate(time: number, index: number, numRobots: number): Message[] {
    const messageType = 'message.support.nubugger.Overview'

    const t = time / 10 - index

    const fieldLength = this.field.fieldLength
    const fieldWidth = this.field.fieldWidth

    const robotPosition = this.figureEight(t, fieldLength / 2, fieldWidth / 2)

    const ballPosition = this.figureEight(t, fieldLength / 4, fieldWidth / 4)

    const robotHeading = ballPosition.clone().subtract(robotPosition)
    // TODO (Annable): Add helper for getting the angle for a unit vector.
    const robotAngle = Math.atan2(robotHeading.y, robotHeading.x)

    const states = getEnumValues<State>(State)
    const modes = getEnumValues<Mode>(Mode)
    const phases = getEnumValues<Phase>(Phase)
    const penaltyReasons = getEnumValues<PenaltyReason>(PenaltyReason)

    const buffer = Overview.encode({
      timestamp: { seconds: time },
      robotId: index + 1,
      roleName: 'Overview Simulator',
      battery: this.random.float(),
      voltage: this.randomFloat(10, 13),
      behaviourState: this.random.choice(states),
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
      lastCameraImage: { seconds: this.randomSeconds(time, -5) },
      lastSeenBall: { seconds: this.randomSeconds(time, -30) },
      lastSeenGoal: { seconds: this.randomSeconds(time, -30) },
      walkCommand: {
        x: Math.cos(time / 3 + index),
        y: Math.cos(time / 5 + index),
        z: Math.cos(time / 7 + index),
      },
      walkPathPlan: [
        robotPosition,
        this.randomFieldPosition(),
        this.randomFieldPosition(),
        this.randomFieldPosition(),
        ballPosition,
      ],
    }).finish()

    const message = { messageType, buffer }

    return [message]
  }

  private randomFieldPosition(): vec2$Properties {
    const fieldLength = this.field.fieldLength
    const fieldWidth = this.field.fieldWidth
    return {
      x: this.random.float() * fieldLength - (fieldLength * 0.5),
      y: this.random.float() * fieldWidth - (fieldWidth * 0.5),
    }
  }

  private randomFloat(min: number, max: number): number {
    return this.random.float() * (max - min) + min
  }

  private randomSeconds(now: number, offset: number): number {
    return now + (offset * this.random.float())
  }

  private figureEight(t: number, scaleX: number = 1, scaleY: number = 1) {
    return new Vector2(
      scaleX * Math.cos(t),
      scaleY * Math.sin(2 * t),
    )
  }
}

function getEnumValues<T>(enumObject: any): T[] {
  return Object.keys(enumObject).map(key => enumObject[key]) as T[]
}
