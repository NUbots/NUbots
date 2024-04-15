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

      gray: {
        100: "#f4f4f4ff",
        200: "#fcfcfcff",
        300: "#e0e0e0ff",
        400: "#888888ff",
        600: "#232323ff",
        800: "#000000ff",
        900: "#00000099",
      },

      orange: {
        100: "#F9A50D",
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
