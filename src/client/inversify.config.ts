import { Container } from 'inversify'
import { LocalisationController } from './components/localisation/controller'
import { LocalisationModel } from './components/localisation/model'
import { LocalisationNetwork } from './components/localisation/network'
import { GlobalNetwork } from './network/global_network'
import { MessageTypePath } from './network/message_type_names'
import { Network } from './network/network'
import { RawSocket } from './network/raw_socket'

export const container = new Container()

container.bind(RawSocket).to(RawSocket).inTransientScope()
container.bind(MessageTypePath).to(MessageTypePath).inSingletonScope()
container.bind(GlobalNetwork).to(GlobalNetwork).inSingletonScope()
container.bind(Network).to(Network).inTransientScope()
container.bind(LocalisationNetwork).to(LocalisationNetwork).inTransientScope()
container.bind(LocalisationController).to(LocalisationController).inSingletonScope()
container.bind(LocalisationModel).toConstantValue(LocalisationModel.of())
