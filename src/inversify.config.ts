import { Container } from 'inversify'
import getDecorators from 'inversify-inject-decorators'
import { LocalisationController } from './client/components/localisation/controller'
import { LocalisationModel } from './client/components/localisation/model'

export const container = new Container()

container.bind(LocalisationController).to(LocalisationController).inSingletonScope()
container.bind(LocalisationModel).toConstantValue(LocalisationModel.of())

export const inject = getDecorators(container).lazyInject
