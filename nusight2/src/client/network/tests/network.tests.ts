import { createMockInstance } from '../../../shared/base/testing/create_mock_instance'
import { message } from '../../../shared/messages'
import { Network } from '../network'
import { NUsightNetwork } from '../nusight_network'
import Mocked = jest.Mocked
import Sensors = message.input.Sensors

describe('Network', () => {
  let nusightNetwork: Mocked<NUsightNetwork>
  let network: Network

  beforeEach(() => {
    nusightNetwork = createMockInstance(NUsightNetwork)
    network = new Network(nusightNetwork)
  })

  it('off() automatically unregisters all callbacks', () => {
    const cb1 = jest.fn()
    const cb2 = jest.fn()

    const off1 = jest.fn()
    nusightNetwork.onNUClearMessage.mockReturnValue(off1)

    network.on(Sensors, cb1)
    expect(nusightNetwork.onNUClearMessage).toHaveBeenCalledWith(Sensors, cb1)

    const off2 = jest.fn()
    nusightNetwork.onNUClearMessage.mockReturnValue(off2)

    network.on(Sensors, cb2)
    expect(nusightNetwork.onNUClearMessage).toHaveBeenCalledWith(Sensors, cb2)

    network.off()
    expect(off1).toHaveBeenCalledTimes(1)
    expect(off2).toHaveBeenCalledTimes(1)
  })
})
