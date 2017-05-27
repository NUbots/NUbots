import { Container } from 'inversify'
import getDecorators from 'inversify-inject-decorators'
import { LocalisationController } from './client/components/localisation/controller'

export const container = new Container()

container.bind(LocalisationController).to(LocalisationController).inSingletonScope()

export const inject = getDecorators(container).lazyInject
