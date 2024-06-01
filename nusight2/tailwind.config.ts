/* eslint-env node */
import type { Config } from "tailwindcss";
import colors from "tailwindcss/colors";
import defaultTheme from "tailwindcss/defaultTheme";
import plugin from "tailwindcss/plugin";

type ColorConfig = Record<string, string | Record<string, string>>;

const config: Config = {
  content: ["./index.html", "./src/client/**/*.{js,jsx,ts,tsx}"],

  darkMode: "class",

  theme: {
    extend: {
      colors: {
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


        // Use for main UI text
        primary: {
          DEFAULT: "rgba(0, 0, 0, 0.87)",
          inverse: "rgba(255, 255, 255, 1)",
        },

        // Use for secondary UI text
        secondary: {
          DEFAULT: "rgba(0, 0, 0, 0.54)",
          inverse: "rgba(255, 255, 255, 0.70)",
        },

        // Use for hint UI text (field labels, help text, etc)
        hint: {
          DEFAULT: "rgba(0, 0, 0, 0.38)",
          inverse: "rgba(255, 255, 255, 0.50)",
        },

        // Use for disabled UI text
        disabled: {
          DEFAULT: "rgba(0, 0, 0, 0.38)",
          inverse: "rgba(255, 255, 255, 0.50)",
        },

        // Use for borders and dividers
        divider: {
          DEFAULT: "rgba(0, 0, 0, 0.12)",
          inverse: "rgba(255, 255, 255, 0.12)",
        },

        // Use for icons
        icon: {
          DEFAULT: "rgba(0, 0, 0, 0.54)",
          inverse: "rgba(255, 255, 255, 0.70)",
        },

        // Use for custom icon buttons when not using <IconButton>
        "icon-button": {
          DEFAULT: "rgba(0, 0, 0, 0.54)",
          focused: "rgba(0, 0, 0, 0.87)",
          disabled: "rgba(0, 0, 0, 0.38)",

          inverse: "rgba(255, 255, 255, 0.70)",
          "focused-inverse": "rgba(255, 255, 255, 1)",
          "disabled-inverse": "rgba(255, 255, 255, 0.50)",
        },

        // Used for inner and outer stroke color on elements with a double ring
        ring: {
          inner: "rgba(255, 255, 255, 1)",
          "inner-inverse": "rgba(0, 0, 0, 1)",

          outer: "rgba(0, 0, 0, 1)",
          "outer-inverse": "rgba(255, 255, 255, 1)",
        },

        // Use for background color of UI surfaces including panels, cards, modals, etc
        surface: {
          0: colors.neutral[200],
          "0-inverse": "#333333ff",

          1: colors.neutral[100],
          "1-inverse": colors.neutral[850],

          2: colors.white,
          "2-inverse": colors.neutral[850],
        },

        // Use to create contrast against the current background for indicating states like hovered, pressed, and selected.
        contrast: {
          1: "rgba(0, 0, 0, 0.05)",
          "1-inverse": "rgba(255, 255, 255, 0.05)",

          2: "rgba(0, 0, 0, 0.10)",
          "2-inverse": "rgba(255, 255, 255, 0.10)",

          3: "rgba(0, 0, 0, 0.15)",
          "3-inverse": "rgba(255, 255, 255, 0.15)",

          4: "rgba(0, 0, 0, 0.20)",
          "4-inverse": "rgba(255, 255, 255, 0.20)",

          5: "rgba(0, 0, 0, 0.25)",
          "5-inverse": "rgba(255, 255, 255, 0.25)",

          6: "rgba(0, 0, 0, 0.30)",
          "6-inverse": "rgba(255, 255, 255, 0.30)",

          max: colors.black,
          "max-inverse": colors.white,
        },

        shadow: {
          card: "rgba(0, 0, 0, 0.15)",
          "card-inverse": "rgba(0, 0, 0, 0.5)",
        },

        nusight: {
          50: "#fff4e0",
          100: "#ffe0b3",
          200: "#ffc980",
          300: "#ffb24d",
          400: "#ffaa00",
          500: "#F9A50D",
          600: "#e59400",
          700: "#cc8300",
          800: "#b37200",
          900: "#996100",
          950: "#804f00"
        },

      },

      fontFamily: {
        sans: ["Roboto", ...defaultTheme.fontFamily.sans],
      },

      boxShadow: {
        "auto-card":
          "var(--color-auto-contrast-3) 0 0 0 1px, var(--color-auto-shadow-card) 0 5px 15px -3px, var(--color-auto-shadow-card) 0 4px 8px -4px",
      },
    },
  },

  plugins: [
    // Add the `not-first:` and `not-last:` variants
    plugin(({ addVariant }) => {
      addVariant("not-first", "&:not(:first-child)");
      addVariant("not-last", "&:not(:last-child)");
    }),

    // Add the common UI colors as CSS variables on the `:root` element
    plugin(({ addBase, theme }) => {
      const flattenedColors = flattenColorsAndGroupByScheme(
        pick(theme("colors"), ["black", "white", "slate", "gray", "nusight"]),
        "--color-",
      );

      addBase({ ":root": flattenedColors.default });
    }),

    // Add the auto colors as CSS variables on the `:root` and `.dark` elements
    // These are colors that automatically change based on the current color scheme in light or dark mode.
    plugin(({ addBase, theme }) => {
      const autoColors = pick(theme("colors")!, [
        "primary",
        "secondary",
        "hint",
        "disabled",
        "divider",
        "icon",
        "icon-button",
        "shadow",
        "surface",
        "ring",
        "contrast",
      ]) as ColorConfig;

      const flattenedColors = flattenColorsAndGroupByScheme(autoColors, "--color-auto-");

      addBase({
        ":root": flattenedColors.default,
        ".dark": {
          ...flattenedColors.inverse,
          // Make scrollbars and other native UI elements use the dark color scheme
          "color-scheme": "dark",
        },
      });
    }),

    // Add utility classes for the auto colors.
    // Depends on the plugin above that adds the auto colors as CSS variables.
    plugin(({ addUtilities, theme }) => {
      const themeColors = theme("colors")!;

      // Add the `text-auto-...` utilities
      addUtilities(
        generateAutoColorUtilities(
          pick(themeColors, ["primary", "secondary", "hint", "disabled", "icon", "icon-button"]),
          "color",
          "text",
        ),
      );

      // Add the `bg-auto-...` utilities
      const bg = generateAutoColorUtilities(pick(themeColors, ["surface", "contrast"]), ["background-color"], "bg");
      addUtilities(bg);

      // Add the `border-auto-...` utilities
      const borderUtilities = {
        border: ["border-color"],
        "border-t": ["border-top-color"],
        "border-b": ["border-bottom-color"],
        "border-l": ["border-left-color"],
        "border-r": ["border-right-color"],
        "border-y": ["border-top-color", "border-bottom-color"],
        "border-x": ["border-left-color", "border-right-color"],
      };

      for (const [utilityName, cssProperties] of Object.entries(borderUtilities)) {
        addUtilities({
          [`.${utilityName}-auto`]: Object.fromEntries(
            cssProperties.map((cssProperty) => [cssProperty, "var(--color-auto-divider)"]),
          ),
        });
      }

      // Add the `ring-auto` utility
      addUtilities({
        ".ring-auto": {
          "--tw-ring-color": "var(--color-auto-divider)",
        },
      });

      // Add the `double-ring-...` utilities
      addUtilities({
        ".double-ring": {
          "--ring-offset-color": "var(--color-auto-ring-inner)",
          "--ring-color": "var(--color-auto-ring-outer)",
          "--ring-offset-shadow": "0 0 0 2px var(--ring-offset-color)",
          "--ring-shadow": "0 0 0 calc(2px + var(--ring-width, 1px)) var(--ring-color)",
          "box-shadow": "var(--ring-offset-shadow), var(--ring-shadow)",
        },
        ".double-ring-2": {
          "--ring-width": "2px",
        },
        ".double-ring-4": {
          "--ring-width": "4px",
        },
      });

      // Add the `divide-auto` utility
      addUtilities({
        ".divide-auto > :not([hidden]) ~ :not([hidden])": {
          "--tw-divide-opacity": "1",
          "border-color": "var(--color-auto-divider)",
        },
      });
    }),
  ],
};

export default config;

/** Pick specific keys from an object into a new object */
function pick<T extends {}, K extends keyof T>(object: T, keys: K[]): Pick<T, K> {
  return Object.fromEntries(Object.entries(object).filter(([key]) => keys.includes(key as K))) as Pick<T, K>;
}

/**
 * Generate "auto color" Tailwind utilities for the given colors that are applied with the given CSS properties.
 * This is used to generate utilities like `text-auto-primary` and `bg-auto-surface-1` that apply the auto colors.
 */
function generateAutoColorUtilities(
  colors: ColorConfig,
  cssPropertyOrProperties: string | string[],
  utilityName: string,
) {
  const cssProperties = Array.isArray(cssPropertyOrProperties) ? cssPropertyOrProperties : [cssPropertyOrProperties];
  const generatedUtilities = {};

  const flattenedColors = flattenColorsAndGroupByScheme(colors);

  for (const colorName of Object.keys(flattenedColors.default)) {
    const colorNameWithoutModifiers = colorName.replace(/-(focused|disabled)$/, "");
    const utilityClassName = `.${utilityName}-auto-${colorNameWithoutModifiers}`;
    const colorValue = `var(--color-auto-${colorName})`;

    if (colorName.endsWith("-focused")) {
      for (const cssProperty of cssProperties) {
        setValueByPath(generatedUtilities, [utilityClassName, "&:hover, &:focus", cssProperty], colorValue);
      }
    } else if (colorName.endsWith("-disabled")) {
      for (const cssProperty of cssProperties) {
        setValueByPath(generatedUtilities, [utilityClassName, "&:disabled, &:disabled:hover", cssProperty], colorValue);
      }
    } else {
      for (const cssProperty of cssProperties) {
        setValueByPath(generatedUtilities, [utilityClassName, cssProperty], colorValue);
      }
    }
  }

  return generatedUtilities;
}

/** Set a value in an object by a path of keys */
function setValueByPath(object: Record<string, any>, keyPath: string[], value: string) {
  const lastKey = keyPath.pop();

  let current = object;
  for (const key of keyPath) {
    if (!current[key]) {
      current[key] = {};
    }

    current = current[key];
  }

  current[lastKey!] = value;
}

/**
 * Flatten the given subset of colors from the Tailwind config into two color scheme objects (default and inverse)
 * where the keys are the full color names (including modifiers like focused and disabled) and the values are
 * the configured color values.
 */
function flattenColorsAndGroupByScheme(colors: ColorConfig, colorNamePrefix = "") {
  // We flatten the colors into two objects, one for the default color scheme
  // (light mode) and one for the inverse color scheme (dark mode)
  const flattened: { default: Record<string, string>; inverse: Record<string, string> } = {
    default: {},
    inverse: {},
  };

  for (const [colorName, colorValueOrOptions] of Object.entries(colors)) {
    // If the color value is a string, it's just a single color value
    if (typeof colorValueOrOptions === "string") {
      flattened.default[colorNamePrefix + colorName] = colorValueOrOptions;
    }
    // If the color value is an object, it's a set of color variants
    else if (typeof colorValueOrOptions === "object" && !Array.isArray(colorValueOrOptions)) {
      for (const [colorVariant, colorValue] of Object.entries(colorValueOrOptions)) {
        if (colorVariant === "DEFAULT") {
          flattened.default[colorNamePrefix + colorName] = colorValue;
        } else if (colorVariant === "inverse") {
          flattened.inverse[colorNamePrefix + colorName] = colorValue;
        } else if (colorVariant.endsWith("-inverse")) {
          flattened.inverse[colorNamePrefix + colorName + "-" + colorVariant.replace(/-inverse$/, "")] = colorValue;
        } else {
          flattened.default[colorNamePrefix + colorName + "-" + colorVariant] = colorValue;
        }
      }
    }
  }

  return flattened;
}
