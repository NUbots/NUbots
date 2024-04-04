import React from "react";
import { NavLink } from "react-router-dom";

import { NavigationConfiguration } from "../../navigation";

interface NavigationItemViewProps {
  url: string;
  Icon: any;
  children?: any;
}

const NavigationItemView = ({ url, Icon, children }: NavigationItemViewProps) => (
  <li className="whitespace-nowrap">
    <NavLink
      className={({ isActive }) => {
        return `p-4 flex flex-col items-center text-white text-xs leading-5 transition-colors duration-300 hover:bg-nusight-200 focus:bg-nusight-200 ${
          isActive ? "!bg-nusight-200" : ""
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
  <header className="bg-nusight-200 text-white text-center ">
    <h1 className="flex justify-center items-center bg-dark text-nusight-300 text-xl font-medium h-[60px]">NUsight</h1>
    <ul>
      {nav.getRoutes().map((config) => (
        <NavigationItemView key={config.path} url={config.path} Icon={config.Icon}>
          {config.label}
        </NavigationItemView>
      ))}
    </ul>
  </header>
);
