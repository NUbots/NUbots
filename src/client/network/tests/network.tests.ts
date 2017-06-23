import { message } from '../../../shared/proto/messages'
import { Network } from '../network'
import Sensors = message.input.Sensors
import { createMockInstance } from '../../../shared/common/testing/create_mock_instance'
import { GlobalNetwork } from '../global_network'

describe('Network', () => {
  let network: GlobalNetwork
  let helper: Network

  beforeEach(() => {
    network = createMockInstance(GlobalNetwork)
    helper = new Network(network)
  })

  it('off() automatically unregisters all callbacks', () => {
    const cb1 = jest.fn()
    const cb2 = jest.fn()
    helper.on(Sensors, cb1)
    expect(network.on).toHaveBeenCalledWith(Sensors, cb1)
    helper.on(Sensors, cb2)
    expect(network.on).toHaveBeenCalledWith(Sensors, cb2)

    helper.off()
    expect(network.off).toHaveBeenCalledWith(Sensors, cb1)
    expect(network.off).toHaveBeenCalledWith(Sensors, cb2)
  })
})
