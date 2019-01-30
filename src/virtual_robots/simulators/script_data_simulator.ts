import { NUClearNetPacket } from 'nuclearnet.js'

import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { message } from '../../shared/proto/messages'
import { Simulator } from '../simulator'
import { Message } from '../simulator'
import Sensors = message.input.Sensors

export class ScriptDataSimulator implements Simulator {
  private network?: NUClearNetClient

  static of() {
    return new ScriptDataSimulator()
  }

  start(network: NUClearNetClient) {
    // console.log('ScriptDataSimulator: started')
    const stop = network.on('message.input.Sensors', this.onSensors)

    return () => {
      stop()
    }
  }

  onSensors(packet: NUClearNetPacket) {
    // console.log('ScriptDataSimulator: packet received', packet)
  }
}
