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
        return `p-4 flex flex-col items-center text-gray-100 text-xs leading-5 transition-colors duration-200 hover:bg-gray-700 focus:bg-gray-700 ${isActive ? "!bg-orange-500" : ""
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
  <div className="bg-gray-800 text-gray-300 text-center flex flex-col justify-between items-center dark">
    <header>
      <h1 className="flex justify-center items-center text-orange-500 text-xl font-medium h-[60px]">NUsight</h1>
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
