import React from "react";

export const ThemeDecorator = (Story, context) => {
  context.globals.showThemeButton = true; // Show the theme button when this decorator is used

  if (context.globals.theme === "both") {
    return (
      <div>
        <div className="light pb-4">
          <div className="bg-gray-100 text-gray-900 py-24 px-8">
            <Story />
          </div>
        </div>
        <div className="dark">
          <div className="bg-gray-900 text-gray-100 py-24 px-8">
            <Story />
          </div>
        </div>
      </div>
    );
  } else {
    return (
      <div className={context.globals.theme}>
        <div className={"bg-gray-100 dark:bg-gray-900 text-gray-900 dark:text-gray-100 py-24 px-4"}>
          <Story />
        </div>
      </div>
    );
  }
};
