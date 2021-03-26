import { action } from 'mobx'
import * as THREE from 'three'
import { Matrix4 } from '../../math/matrix4'
import { Vector3 } from '../../math/vector3'
import { Network } from '../../network/network'
import { NUsightNetwork } from '../../network/nusight_network'
import { RobotModel } from '../robot/model'
import { OdometryRobotModel } from './model'
import { message } from '../../../shared/messages'

export class OdometryNetwork {
  constructor(private network: Network) {
    this.network.on(message.input.Sensors, this.onSensors)
  }

  static of(nusightNetwork: NUsightNetwork): OdometryNetwork {
    const network = Network.of(nusightNetwork)
    return new OdometryNetwork(network)
  }

  destroy = () => {
    this.network.off()
  }

  @action.bound
  private onSensors(robotModel: RobotModel, packet: message.input.Sensors) {
    const robot = OdometryRobotModel.of(robotModel)
    robot.visualizerModel.Hwt = Matrix4.fromThree(
      new THREE.Matrix4().getInverse(Matrix4.from(packet.Htw).toThree()),
    )
    robot.visualizerModel.accelerometer = Vector3.from(packet.accelerometer)
  }
}
