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
      gray: colors.neutral,
      primary: "rgba(0, 0, 0, 0.87)",
      icon: "rgba(0, 0, 0, 0.54)",
      divider: "rgba(0, 0, 0, 0.12)",
      navigation: "rgba(0,0,0,.87)",

      grey: {
        100: "#000000d9",
        200: "#00000099",
      },

      orange: {
        100: "#F9A50D",
      },

      nusight: {
        // Red
        100: "#ff0000",
        // Grey
        200: "#000000d9",
        // Orange
        300: "#F9A50D",
        400: "#2bb1ee",
        500: "#1197d3",
        600: "#0d76a6",
        700: "#004363",
      },

      fontFamily: {
        sans: ["Roboto", ...defaultTheme.fontFamily.sans],
      },
    },
    extend: {
      spacing: {
        // 11: "2.85rem",
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
