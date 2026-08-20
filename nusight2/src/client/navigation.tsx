import { ComponentType } from "react";

export type Route = {
  path: string;
  Icon: any;
  label: string;
  Content: ComponentType;
};

export class NavigationConfiguration {
  private routes: Route[] = [];
  private basePath: string;

  constructor(basePath: string = "") {
    // Remove any trailing / from the base path
    this.basePath = basePath.replace(/\/$/, "");
  }

  static of(basePath: string = "") {
    return new NavigationConfiguration(basePath);
  }

  addRoute(route: Route) {
    // Trim any leading / from the route path, then prefix with the base path
    const routePath = route.path.replace(/^\//, "");
    this.routes.push({ ...route, path: `${this.basePath}/${routePath}` });
  }

  getRoutes() {
    return this.routes;
  }
}
