import { autorun } from 'mobx'

import { NUClearNetClient } from '../../shared/nuclearnet/nuclearnet_client'
import { message } from '../../shared/messages'
import { toTimestamp } from '../../shared/time/timestamp'
import { Simulator } from '../simulator'
import { Message } from '../simulator'

import { periodic } from './periodic'
import DataPoint = message.support.nusight.DataPoint

export class ChartSimulator extends Simulator {
  static of({ nuclearnetClient }: { nuclearnetClient: NUClearNetClient }): ChartSimulator {
    return new ChartSimulator(nuclearnetClient)
  }

  start() {
    return autorun(() => this.send(this.chartData))
  }

  get chartData(): Message {
    // Offset our time to test the adaptive window
    const time = periodic(60) - 3

    const messageType = 'message.support.nusight.DataPoint'
    const period = 10
    const theta = (2 * Math.PI * time) / period
    const sin = Math.sin(theta)
    const cos = Math.cos(theta)

    const buffer = DataPoint.encode({
      label: 'Debug Waves',
      value: [sin, cos, 2 * sin, 4 * cos],
      timestamp: toTimestamp(time),
    }).finish()

    const message = { messageType, buffer }
    return message
  }
}
