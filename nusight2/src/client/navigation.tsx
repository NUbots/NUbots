import { ComponentType } from "react";

export type Route = {
  path: string;
  Icon: any;
  label: string;
  Content: ComponentType;
};

export class NavigationConfiguration {
  private routes: Route[] = [];

  static of() {
    return new NavigationConfiguration();
  }

  addRoute(route: Route) {
    this.routes.push(route);
  }

  getRoutes() {
    return this.routes;
  }
}
