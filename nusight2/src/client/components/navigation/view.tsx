import React from "react";
import { NavLink } from "react-router-dom";

import { NavigationConfiguration } from "../../navigation";
import ThemeSwitcherButton from "../theme_switcher_button/view";

interface NavigationItemViewProps {
  url: string;
  Icon: any;
  children?: any;
}

const NavigationItemView = ({ url, Icon, children }: NavigationItemViewProps) => (
  <li className="whitespace-nowrap">
    <NavLink
      className={({ isActive }) => {
        return `p-4 flex flex-col items-center text-xs hover:bg-auto-contrast-1 ${isActive ? "!bg-nusight-600 text-white" : ""
          }`;
      }}
      to={url}
    >
      <Icon className="w-6 h-6" />
      <span>{children}</span>
    </NavLink>
  </li>
);

export const NavigationView = ({ nav }: { nav: NavigationConfiguration }) => (
  <div className="bg-auto-surface-1 border-r border-auto text-center flex flex-col justify-between items-center">
    <header>
      <h1 className="flex justify-center items-center text-nusight-500 text-xl font-medium h-[60px]">NUsight</h1>
      <ul>
        {nav.getRoutes().map((config) => (
          <NavigationItemView key={config.path} url={config.path} Icon={config.Icon}>
            {config.label}
          </NavigationItemView>
        ))}
      </ul>
    </header>
    <ThemeSwitcherButton className="m-8"></ThemeSwitcherButton>
  </div>
);
