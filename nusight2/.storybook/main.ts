import type { StorybookConfig } from "@storybook/react-vite";
import { mergeConfig, UserConfig } from "vite";

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
  viteFinal: async (config) => {
    return mergeConfig(config, {
      // Needed for Vite 5: https://github.com/storybookjs/storybook/issues/25256
      assetsInclude: ["/sb-preview/**"],
      // Support newer javascript features like bigint
      build: {
        target: "es2022",
      },
      css: {
        postcss: {
          plugins: [require("tailwindcss")()],
        },
      },
    } as UserConfig);
  },
};

export default config;
