import { action } from 'mobx'
import { message } from '../../../shared/messages'
import { toSeconds } from '../../../shared/time/timestamp'
import { Matrix2 } from '../../math/matrix2'
import { Matrix3 } from '../../math/matrix3'
import { Vector2 } from '../../math/vector2'
import { Vector3 } from '../../math/vector3'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'
import { DashboardRobotModel } from './dashboard_robot/model'

import Overview = message.support.nusight.Overview

export class DashboardNetwork {
  constructor(private network: Network) {
    this.network.on(Overview, this.onOverview)
  }

  static of(nusightNetwork: NUsightNetwork): DashboardNetwork {
    const network = Network.of(nusightNetwork)
    return new DashboardNetwork(network)
  }

  destroy() {
    this.network.off()
  }

  @action
  private onOverview = (robotModel: RobotModel, overview: Overview) => {
    const robot = DashboardRobotModel.of(robotModel)

    // Timestamp this message was sent (for comparison with last seen)
    robot.time = toSeconds(overview.timestamp)

    // The id number of the robot
    robot.playerId = overview.robotId

    // Name of the executing binary
    robot.roleName = overview.roleName

    // Battery as a value between 0 and 1 (percentage)
    robot.battery = overview.battery

    // Voltage (in volts!)
    robot.voltage = overview.voltage

    // The current behaviour state as an enum
    robot.behaviourState = overview.behaviourState

    // The position of the robot on the field in field coordinates
    // overview.robotPosition is a 3 sized vector (x,y,heading)
    robot.robotPosition = Vector3.from(overview.robotPosition)
    robot.robotPositionCovariance = Matrix3.from(overview.robotPositionCovariance)

    // The position of the ball in field coordinates
    robot.ballPosition = Vector2.from(overview.ballPosition)
    robot.ballCovariance = Matrix2.from(overview.ballPositionCovariance)

    // The location on the field the robot wants to kick in field coordinates
    robot.kickTarget = Vector2.from(overview.kickTarget)

    // The game mode the robot thinks it is
    robot.gameMode = overview.gameMode
    robot.gamePhase = overview.gamePhase
    robot.penaltyReason = overview.penaltyReason

    // The last time we had a camera image, saw a ball/goal
    robot.lastCameraImage = toSeconds(overview.lastCameraImage)
    robot.lastSeenBall = toSeconds(overview.lastSeenBall)
    robot.lastSeenGoal = toSeconds(overview.lastSeenGoal)

    // The walk command and
    robot.walkCommand = Vector3.from(overview.walkCommand)
  }
}
