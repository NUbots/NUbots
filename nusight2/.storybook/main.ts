import type { StorybookConfig } from "@storybook/react-vite";
import { mergeConfig } from "vite";

const config: StorybookConfig = {
  stories: ["../src/**/*.stories.@(js|jsx|ts|tsx)"],
  addons: ["@storybook/addon-essentials", "@storybook/addon-interactions"],
  framework: {
    name: "@storybook/react-vite",
    options: {},
  },
  docs: {
    autodocs: "tag",
  },
  staticDirs: ["../src/assets"],
  viteFinal: (config) => {
    return mergeConfig(config, {
      css: {
        postcss: {
          plugins: [require("tailwindcss")()],
        },
      },
    });
  },
};

export default config;
