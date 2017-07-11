import { MessageTypePath } from '../client/network/message_type_names'
import { Network } from '../client/network/network'
import { NUsightNetwork } from '../client/network/nusight_network'
import { FakeNUClearNetClient } from '../server/nuclearnet/fake_nuclearnet_client'
import { FakeNUClearNetServer } from '../server/nuclearnet/fake_nuclearnet_server'
import { NodeSystemClock } from '../server/time/node_clock'
import { message } from '../shared/proto/messages'
import { RobotSimulator } from '../simulators/robot_simulator'
import { SensorDataSimulator } from '../simulators/sensor_data_simulator'
import Sensors = message.input.Sensors
import VisionObject = message.vision.VisionObject
import Overview = message.support.nubugger.Overview

describe('Networking Integration', () => {
  let nuclearnetServer: FakeNUClearNetServer
  let nusightNetwork: NUsightNetwork
  let robotSimulator: RobotSimulator
  let disconnectNusightNetwork: () => void

  beforeEach(() => {
    nuclearnetServer = new FakeNUClearNetServer()
    nusightNetwork = createNUsightNetwork()
    disconnectNusightNetwork = nusightNetwork.connect({ name: 'nusight' })

    robotSimulator = new RobotSimulator(
      new FakeNUClearNetClient(nuclearnetServer),
      NodeSystemClock,
      {
        name: 'Robot #1',
        simulators: [
          // TODO (Annable): Add vision and overview simulators when they exist
          new SensorDataSimulator(),
        ],
      },
    )
  })

  function createNUsightNetwork() {
    const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer)
    const messageTypePath = new MessageTypePath()
    return new NUsightNetwork(nuclearnetClient, messageTypePath)
  }

  describe('a single networked component', () => {
    let network: Network

    beforeEach(() => {
      network = new Network(nusightNetwork)
    })

    it('receives a Sensors message after subscribing and a robot sending it', () => {
      const onSensors = jest.fn()
      network.on(Sensors, onSensors)

      robotSimulator.simulate()

      expect(onSensors).toHaveBeenCalledWith(expect.any(Sensors))
      expect(onSensors).toHaveBeenCalledTimes(1)
    })

    it('does not receive any messages after unsubscribing', () => {
      const onSensors1 = jest.fn()
      const onSensors2 = jest.fn()
      network.on(Sensors, onSensors1)
      network.on(Sensors, onSensors2)

      network.off()

      robotSimulator.simulate()

      expect(onSensors1).not.toHaveBeenCalled()
      expect(onSensors2).not.toHaveBeenCalled()
    })

    it('does not receive message on specific unsubscribed callback', () => {
      const onSensors1 = jest.fn()
      const onSensors2 = jest.fn()
      const off1 = network.on(Sensors, onSensors1)
      network.on(Sensors, onSensors2)

      off1()

      robotSimulator.simulate()

      expect(onSensors1).not.toHaveBeenCalled()
      expect(onSensors2).toHaveBeenCalledWith(expect.any(Sensors))
    })
  })

  describe('sessions', () => {
    let network: Network

    beforeEach(() => {
      network = new Network(nusightNetwork)
    })

    it('handles reconnects', () => {
      const onSensors = jest.fn()
      network.on(Sensors, onSensors)

      disconnectNusightNetwork()

      nusightNetwork.connect({ name: 'nusight' })

      robotSimulator.simulate()

      expect(onSensors).toHaveBeenCalledWith(expect.any(Sensors))
    })

    it('handles multiple sessions simutaneously', () => {
      const nusightNetwork2 = createNUsightNetwork()
      nusightNetwork2.connect({ name: 'nusight' })
      const network2 = new Network(nusightNetwork2)

      const onSensors1 = jest.fn()
      network.on(Sensors, onSensors1)

      const onSensors2 = jest.fn()
      network2.on(Sensors, onSensors2)

      robotSimulator.simulate()

      expect(onSensors1).toHaveBeenCalledWith(expect.any(Sensors))
      expect(onSensors2).toHaveBeenCalledWith(expect.any(Sensors))
    })
  })

  describe('multiple networked components', () => {
    let localisationNetwork: Network
    let visionNetwork: Network
    let dashboardNetwork: Network

    beforeEach(() => {
      localisationNetwork = new Network(nusightNetwork)
      visionNetwork = new Network(nusightNetwork)
      dashboardNetwork = new Network(nusightNetwork)
    })

    it('subscribes and unsubscribes as expected when switching between components', () => {
      const onSensors = jest.fn()
      localisationNetwork.on(Sensors, onSensors)

      robotSimulator.simulate()

      expect(onSensors).toHaveBeenCalledTimes(1)

      localisationNetwork.off()

      const onVisionObject = jest.fn()
      visionNetwork.on(VisionObject, onVisionObject)

      robotSimulator.simulate()

      expect(onVisionObject).toHaveBeenCalledTimes(0)
      expect(onSensors).toHaveBeenCalledTimes(1)

      visionNetwork.off()

      const onOverview = jest.fn()
      dashboardNetwork.on(Overview, onOverview)

      expect(onOverview).toHaveBeenCalledTimes(0)
      expect(onVisionObject).toHaveBeenCalledTimes(0)
      expect(onSensors).toHaveBeenCalledTimes(1)

      dashboardNetwork.off()
    })
  })
})
