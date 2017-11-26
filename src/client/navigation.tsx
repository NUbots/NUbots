import { ComponentType } from 'react'
import * as React from 'react'

type Route = {
  exact?: boolean
  path: string
  Icon: any,
  label: string,
  Content: ComponentType
}

export class NavigationConfiguration {
  private routes: Route[] = []

  public static of() {
    return new NavigationConfiguration()
  }

  public addRoute(route: Route) {
    this.routes.push(route)
  }

  public getRoutes() {
    return this.routes
  }
}
