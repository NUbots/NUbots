/* eslint-env node */
import colors from "tailwindcss/colors";
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";

/** @type {import("tailwindcss").Config} */
const config = {
  content: ["./index.html", "./src/client/**/*.{js,jsx,ts,tsx}"],


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
        800: "#30302d",
        900: "#1c1c1a"
      },

      gray: {
        100: "#fcfcfcff",
        200: "#f4f4f4ff",
        300: "#e0e0e0ff",
        400: "#5d5d5dff",
        500: "#484848ff",
        600: "#373737ff",
        700: "#2a2a2aff",
        800: "#232323ff",
        900: "#00000099",
      },

      blue: {
        100: "#ABB7CB",
        200: "#2F66C4",
        300: "#174CA5",
        400: "#05317D",
        500: "#04255E",
      },

      green: {
        100: "#B4D9BC",
        200: "#22D449",
        300: "#0ABC31",
        400: "#008F20",
        500: "#006C18"
      },

      orange: {
        100: "#ffd372",
        200: "#F9A50D",
        300: "#f09c00",
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
