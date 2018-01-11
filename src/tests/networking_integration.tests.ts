import { AppModel } from '../client/components/app/model'
import { AppNetwork } from '../client/components/app/network'
import { MessageTypePath } from '../client/network/message_type_names'
import { Network } from '../client/network/network'
import { NUsightNetwork } from '../client/network/nusight_network'
import { FakeNUClearNetClient } from '../server/nuclearnet/fake_nuclearnet_client'
import { FakeNUClearNetServer } from '../server/nuclearnet/fake_nuclearnet_server'
import { NodeSystemClock } from '../server/time/node_clock'
import { message } from '../shared/proto/messages'
import Sensors = message.input.Sensors
import Overview = message.support.nubugger.Overview
import VisionObject = message.vision.VisionObject
import { OverviewSimulator } from '../virtual_robots/simulators/overview_simulator'
import { SensorDataSimulator } from '../virtual_robots/simulators/sensor_data_simulator'
import { VirtualRobot } from '../virtual_robots/virtual_robot'
import { VirtualRobots } from '../virtual_robots/virtual_robots'

describe('Networking Integration', () => {
  let nuclearnetServer: FakeNUClearNetServer
  let nusightNetwork: NUsightNetwork
  let virtualRobots: VirtualRobots
  let disconnectNusightNetwork: () => void

  beforeEach(() => {
    nuclearnetServer = new FakeNUClearNetServer()
    nusightNetwork = createNUsightNetwork()
    disconnectNusightNetwork = nusightNetwork.connect({ name: 'nusight' })

    virtualRobots = new VirtualRobots({
      robots: [
        new VirtualRobot(
          new FakeNUClearNetClient(nuclearnetServer),
          NodeSystemClock,
          {
            name: 'Robot #1',
            simulators: [
              { frequency: 1, simulator: OverviewSimulator.of() },
              { frequency: 60, simulator: SensorDataSimulator.of() },
            ],
          },
        ),
      ],
    })
    virtualRobots.connect()
  })

  function createNUsightNetwork() {
    const appModel = AppModel.of()
    const nuclearnetClient = new FakeNUClearNetClient(nuclearnetServer)
    const messageTypePath = new MessageTypePath()
    const nusightNetwork = new NUsightNetwork(nuclearnetClient, appModel, messageTypePath)
    AppNetwork.of(nusightNetwork, appModel)
    return nusightNetwork
  }

  describe('a single networked component', () => {
    let network: Network

    beforeEach(() => {
      network = new Network(nusightNetwork)
    })

    it('receives a Sensors message after subscribing and a robot sending it', () => {
      const onSensors = jest.fn()
      network.on(Sensors, onSensors)

      virtualRobots.simulateAll()

      expect(onSensors).toHaveBeenCalledWith(expect.objectContaining({ name: 'Robot #1' }), expect.any(Sensors))
      expect(onSensors).toHaveBeenCalledTimes(1)
    })

    it('does not receive any messages after unsubscribing', () => {
      const onSensors1 = jest.fn()
      const onSensors2 = jest.fn()
      network.on(Sensors, onSensors1)
      network.on(Sensors, onSensors2)

      network.off()

      virtualRobots.simulateAll()

      expect(onSensors1).not.toHaveBeenCalled()
      expect(onSensors2).not.toHaveBeenCalled()
    })

    it('does not receive message on specific unsubscribed callback', () => {
      const onSensors1 = jest.fn()
      const onSensors2 = jest.fn()
      const off1 = network.on(Sensors, onSensors1)
      network.on(Sensors, onSensors2)

      off1()

      virtualRobots.simulateAll()

      expect(onSensors1).not.toHaveBeenCalled()
      expect(onSensors2).toHaveBeenCalledWith(expect.objectContaining({ name: 'Robot #1' }), expect.any(Sensors))
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

      virtualRobots.simulateAll()

      expect(onSensors).toHaveBeenCalledWith(expect.objectContaining({ name: 'Robot #1' }), expect.any(Sensors))
    })

    it('handles multiple sessions simutaneously', () => {
      const nusightNetwork2 = createNUsightNetwork()
      nusightNetwork2.connect({ name: 'nusight' })
      const network2 = new Network(nusightNetwork2)

      const onSensors1 = jest.fn()
      network.on(Sensors, onSensors1)

      const onSensors2 = jest.fn()
      network2.on(Sensors, onSensors2)

      virtualRobots.simulateAll()

      expect(onSensors1).toHaveBeenCalledWith(expect.objectContaining({ name: 'Robot #1' }), expect.any(Sensors))
      expect(onSensors2).toHaveBeenCalledWith(expect.objectContaining({ name: 'Robot #1' }), expect.any(Sensors))
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

      virtualRobots.simulateAll()

      expect(onSensors).toHaveBeenCalledTimes(1)

      localisationNetwork.off()

      const onVisionObject = jest.fn()
      visionNetwork.on(VisionObject, onVisionObject)

      virtualRobots.simulateAll()

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
