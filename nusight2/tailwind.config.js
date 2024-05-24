/* eslint-env node */
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";

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
      red: "#ff0000ff",
      transparent: "transparent",

      blue: {
        50: "#eff6ff",
        100: "#dbeafe",
        200: "#bfdbfe",
        300: "#93c5fd",
        400: "#60a5fa",
        500: "#3b82f6",
        600: "#2563eb",
        700: "#1d4ed8",
        800: "#1e40af",
        900: "#1e3a8a",
        950: "#172554",
      },

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
