import { message } from '../../src/shared/proto/messages'
import { Simulator } from './simulator'
import { Message } from './simulator'
import Sensors = message.input.Sensors

export class SensorDataSimulator implements Simulator {
  public static of() {
    return new SensorDataSimulator()
  }

  public simulate(time: number): Message[] {
    const messageType = 'message.input.Sensors'

    // Simulate a walk
    const t = time * 5E-3

    const buffer = Sensors.encode({
      servo: [
        { presentPosition: 3 * Math.PI / 4 + 0.5 * Math.cos(t - Math.PI) },
        { presentPosition: 3 * Math.PI / 4 + 0.5 * Math.cos(t) },
        { presentPosition: -Math.PI / 8 },
        { presentPosition: Math.PI / 8 },
        { presentPosition: -3 * Math.PI / 4 },
        { presentPosition: -3 * Math.PI / 4 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0.5 * (Math.cos(t) - 1) },
        { presentPosition: 0.5 * (Math.cos(t - Math.PI) - 1) },
        { presentPosition: 0.5 * (-Math.cos(t) + 1) },
        { presentPosition: 0.5 * (-Math.cos(t - Math.PI) + 1) },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0 },
        { presentPosition: 0.1 * Math.cos(t) },
        { presentPosition: 0.1 * Math.cos(t / 3) + 0.4 },
      ],
    }).finish()

    const message = { messageType, buffer }

    return [message]
  }
}
