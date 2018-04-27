import { message } from '../../shared/proto/messages'
import { toTimestamp } from '../../shared/time/timestamp'
import { Simulator } from '../simulator'
import { Message } from '../simulator'

import DataPoint = message.support.nusight.DataPoint

export class ChartSimulator implements Simulator {
  static of(): ChartSimulator {
    return new ChartSimulator()
  }

  simulate(time: number, index: number, numRobots: number): Message[] {

    // Offset our time to test the adaptive window
    time = time - 3

    const messageType = 'message.support.nusight.DataPoint'
    const period = 10
    const theta = 2 * Math.PI * time / period
    const sin = Math.sin(theta)
    const cos = Math.cos(theta)

    const buffer = DataPoint.encode({
      label: 'Debug Waves',
      value: [sin, cos, 2 * sin, 4 * cos],
      timestamp: toTimestamp(time),
    }).finish()

    const message = { messageType, buffer }

    return [message]
  }
}

