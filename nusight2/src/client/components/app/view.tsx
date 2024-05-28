import { Component, Suspense } from "react";
import React from "react";
import { Routes } from "react-router";
import { Navigate } from "react-router";
import { Route } from "react-router";
import { BrowserRouter } from "react-router-dom";

import { NavigationConfiguration } from "../../navigation";
import { ContextMenuProvider } from "../context_menu/context_menu";
import { NavigationView } from "../navigation/view";

export class AppView extends Component<{ nav: NavigationConfiguration }> {
  render() {
    return (
      <BrowserRouter>
        <ContextMenuProvider>
          <div className="min-h-screen flex items-stretch bg-white text-sm text-[#4d4d4d]">
            <NavigationView nav={this.props.nav} />
            <div className="flex flex-grow flex-col">
              <div className="flex flex-grow">
                <Suspense fallback={<div>Loading...</div>}>
                  <Routes>
                    {this.props.nav.getRoutes().map((config) => (
                      <Route key={config.path} path={config.path} element={<config.Content />} />
                    ))}
                    <Route path="/" element={<Navigate replace to={this.props.nav.getRoutes()[0].path} />} />
                  </Routes>
                </Suspense>
              </div>
            </div>
          </div>
        </ContextMenuProvider>
      </BrowserRouter>
    );
  }
}
