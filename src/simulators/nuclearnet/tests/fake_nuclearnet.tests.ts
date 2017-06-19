import { NUClearNet } from 'nuclearnet.js'
import { FakeNUClearNet } from '../fake_nuclearnet'
import { FakeNUClearNetServer } from '../fake_nuclearnet_server'

describe('FakeNUClearNet', () => {
  let server: FakeNUClearNetServer
  let alice: NUClearNet
  let bob: NUClearNet
  let eve: NUClearNet

  beforeEach(() => {
    server = new FakeNUClearNetServer()
    alice = new FakeNUClearNet(server)
    bob = new FakeNUClearNet(server)
    eve = new FakeNUClearNet(server)
  })

  it('calls nuclear_join event handlers on connect', () => {
    alice.connect({ name: 'alice' })
    eve.connect({ name: 'eve' })

    const aliceOnJoin = jest.fn()
    alice.on('nuclear_join', aliceOnJoin)

    const eveOnJoin = jest.fn()
    eve.on('nuclear_join', eveOnJoin)

    const bobOnJoin = jest.fn()
    bob.on('nuclear_join', bobOnJoin)

    bob.connect({ name: 'bob' })

    const expectedPeer = { name: 'bob', address: '127.0.0.1', port: 7447 }

    expect(aliceOnJoin).toHaveBeenCalledWith(expectedPeer)
    expect(eveOnJoin).toHaveBeenCalledWith(expectedPeer)
    expect(bobOnJoin).not.toHaveBeenCalled()
  })

  it('calls nuclear_leave event handlers on disconnect', () => {
    alice.connect({ name: 'alice' })
    eve.connect({ name: 'eve' })

    const aliceOnLeave = jest.fn()
    alice.on('nuclear_leave', aliceOnLeave)

    const eveOnLeave = jest.fn()
    eve.on('nuclear_leave', eveOnLeave)

    const bobOnLeave = jest.fn()
    bob.on('nuclear_leave', bobOnLeave)

    bob.connect({ name: 'bob' })

    const expectedPeer = { name: 'bob', address: '127.0.0.1', port: 7447 }

    bob.disconnect()

    expect(aliceOnLeave).toHaveBeenCalledWith(expectedPeer)
    expect(eveOnLeave).toHaveBeenCalledWith(expectedPeer)
    expect(bobOnLeave).not.toHaveBeenCalled()
  })
})
