/* eslint-env node */
import colors from "tailwindcss/colors";
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";
const colors = require("tailwindcss/colors");

/** @type {import("tailwindcss").Config} */
const config = {
  content: ["./index.html", "./src/client/**/*.{js,jsx,ts,tsx}", ".storybook/**/*.{js,jsx,ts,tsx}"],
  darkMode: "class",

  theme: {
    colors: {
      white: "#ffffff",
      black: "#000000",
      icon: "#0000008a",
      divider: "#0000001f",
      blue: colors.blue,
      red: "#ff0000ff",
      transparent: "transparent",

      gray: {
        50: "#fafafa",
        100: "#fcfcfcff",
        150: "#f9f9f9ff",
        200: "#f5f5f5",
        250: "#f2f2f2ff",
        300: "#e5e5e5",
        350: "#dcdcdcff",
        400: "#d4d4d4",
        450: "#c9c9c9ff",
        500: "#a3a3a3",
        550: "#8f8f8fff",
        600: "#737373",
        650: "#676767ff",
        700: "#525252",
        750: "#474747ff",
        800: "#404040",
        850: "#333333ff",
        900: "#262626",
        950: "#171717",
      },

      green: {
        100: "#B4D9BC",
        200: "#22D449",
        300: "#0ABC31",
        400: "#008F20",
        500: "#006C18",
      },

      orange: {
        400: "#ffaa00",
        500: "#F9A50D",
      },

      fontFamily: {
        sans: ["Roboto", ...defaultTheme.fontFamily.sans],
        inherit: "inherit",
      },
    },
    extend: {
      spacing: {
        11: "2.85rem",
        inherit: "inherit",
      },
    },

    plugins: [
      plugin(({ addVariant }) => {
        addVariant("not-first", "&:not(:first-child)");
        addVariant("not-last", "&:not(:last-child)");
      }),
    ],
  },
};

export default config;
