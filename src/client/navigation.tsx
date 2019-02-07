import { ComponentType } from 'react'
import * as React from 'react'

export type Route = {
  exact?: boolean
  path: string
  Icon: any,
  label: string,
  Content: ComponentType
}

export class NavigationConfiguration {
  private routes: Route[] = []

  static of() {
    return new NavigationConfiguration()
  }

  addRoute(route: Route) {
    this.routes.push(route)
  }

  getRoutes() {
    return this.routes
  }
}
