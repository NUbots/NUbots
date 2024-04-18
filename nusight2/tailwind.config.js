/* eslint-env node */
import colors from "tailwindcss/colors";
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";

/** @type {import("tailwindcss").Config} */
const config = {
  content: ["./index.html", "./src/client/**/*.{js,jsx,ts,tsx}"],

  darkMode: "class",

  theme: {
    colors: {
      // gray: colors.neutral,
      white: "#ffffff",
      black: "#000000",
      accent: "#F9A50D",
      icon: "#0000008a",
      divider: "#0000001f",
      danger: "#ff0000ff",
      warning: "#ffaa00",

      orangegray: {
        200: "#fdfbf9",
        300: "#fdf7f2",
      },

      gray: {
        100: "#f4f4f4ff",
        200: "#fcfcfcff",
        300: "#e0e0e0ff",
        400: "#5d5d5dff",
        500: "#484848ff",
        600: "#373737ff",
        700: "#2a2a2aff",
        800: "#232323ff",
        900: "#00000099",
      },

      green: {
        100: "#0c7700",
        200: "#00de46"
      },

      orange: {
        100: "#ffae00",
      },

      transparent: "transparent",

      nusight: {
        100: "#2bb1ee",
        200: "#1197d3",
        300: "#0d76a6",
        400: "#004363",
      },

      fontFamily: {
        sans: ["Roboto", ...defaultTheme.fontFamily.sans],
      },
    },
    extend: {
      spacing: {
        11: "2.85rem",
      }
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
