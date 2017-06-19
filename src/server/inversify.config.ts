import { EventEmitter } from 'events'
import { Container } from 'inversify'
import { decorate } from 'inversify'
import { injectable } from 'inversify'
import { NUClearNet } from 'nuclearnet.js'
import { FakeNUClearNet } from '../simulators/nuclearnet/fake_nuclearnet'
import { FakeNUClearNetServer } from '../simulators/nuclearnet/fake_nuclearnet_server'
import { Clock } from './time/clock'
import { ClockType } from './time/clock'
import { NodeSystemClock } from './time/node_clock'

export const getContainer = ({ fakeNetworking = false }: { fakeNetworking: boolean }) => {
  const container = new Container()

  decorate(injectable(), EventEmitter)
  if (fakeNetworking) {
    container.bind<FakeNUClearNetServer>(FakeNUClearNetServer).to(FakeNUClearNetServer).inSingletonScope()
    container.bind<NUClearNet>(NUClearNet).to(FakeNUClearNet).inTransientScope()
  } else {
    decorate(injectable(), NUClearNet)
    container.bind<NUClearNet>(NUClearNet).to(NUClearNet).inTransientScope()
  }

  container.bind<Clock>(ClockType).toConstantValue(NodeSystemClock)

  return container
}
