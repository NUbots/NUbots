import { action } from 'mobx'
import { message } from '../../../shared/proto/messages'
import { GlobalNetwork } from '../../network/global_network'
import { Network } from '../../network/network'
import { LocalisationModel } from './model'
import Sensors = message.input.Sensors

export class LocalisationNetwork {
  public constructor(private network: Network,
                     private model: LocalisationModel) {
    this.network.on(Sensors, this.onSensors)
  }

  public static of(globalNetwork: GlobalNetwork, model: LocalisationModel): LocalisationNetwork {
    const network = Network.of(globalNetwork)
    return new LocalisationNetwork(network, model)
  }

  public destroy() {
    this.network.off()
  }

  @action
  private onSensors = (sensors: Sensors) => {
    // TODO (Annable): Remove hardcoded values.
    const robot = this.model.robots[0]
    robot.motors.rightShoulderPitch.angle = Number(sensors.servo[0].presentPosition)
    robot.motors.leftShoulderPitch.angle = Number(sensors.servo[1].presentPosition)
    robot.motors.rightShoulderRoll.angle = Number(sensors.servo[2].presentPosition)
    robot.motors.leftShoulderRoll.angle = Number(sensors.servo[3].presentPosition)
    robot.motors.rightElbow.angle = Number(sensors.servo[4].presentPosition)
    robot.motors.leftElbow.angle = Number(sensors.servo[5].presentPosition)
    robot.motors.rightHipYaw.angle = Number(sensors.servo[6].presentPosition)
    robot.motors.leftHipYaw.angle = Number(sensors.servo[7].presentPosition)
    robot.motors.rightHipRoll.angle = Number(sensors.servo[8].presentPosition)
    robot.motors.leftHipRoll.angle = Number(sensors.servo[9].presentPosition)
    robot.motors.rightHipPitch.angle = Number(sensors.servo[10].presentPosition)
    robot.motors.leftHipPitch.angle = Number(sensors.servo[11].presentPosition)
    robot.motors.rightKnee.angle = Number(sensors.servo[12].presentPosition)
    robot.motors.leftKnee.angle = Number(sensors.servo[13].presentPosition)
    robot.motors.rightAnklePitch.angle = Number(sensors.servo[14].presentPosition)
    robot.motors.leftAnklePitch.angle = Number(sensors.servo[15].presentPosition)
    robot.motors.rightAnkleRoll.angle = Number(sensors.servo[16].presentPosition)
    robot.motors.leftAnkleRoll.angle = Number(sensors.servo[17].presentPosition)
    robot.motors.headPan.angle = Number(sensors.servo[18].presentPosition)
    robot.motors.headTilt.angle = Number(sensors.servo[19].presentPosition)
  }
}
