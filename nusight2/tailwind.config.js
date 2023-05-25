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
          500: "#1197d3",
          600: "#0d76a6",
          700: "#004363",
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
