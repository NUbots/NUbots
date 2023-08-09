/* eslint-env node */
import colors from "tailwindcss/colors";
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";

/** @type {import("tailwindcss").Config} */
const config = {
  content: ["./index.html", "./src/client/**/*.{js,jsx,ts,tsx}"],

  darkMode: "class",

  theme: {
    extend: {
      colors: {
        gray: colors.neutral,
        primary: "rgba(0, 0, 0, 0.87)",
        icon: "rgba(0, 0, 0, 0.54)",
        divider: "rgba(0, 0, 0, 0.12)",

        nusight: {
          400: "#2bb1ee",
          500: "#F9A50D",
          600: "#0d76a6",
          700: "#004363",
          800: "#F9A50D", // Yellow orange
          1000: "#111111", // Black
          1100: "#212121", // Dark grey
          1200: "#383838", // Lighter dark grey
          1300: "#424242", // Even Lighter dark grey
        },

        fontFamily: {
          sans: ["Roboto", ...defaultTheme.fontFamily.sans],
        },
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
