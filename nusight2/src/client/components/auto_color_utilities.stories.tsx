import React, { useLayoutEffect, useRef } from "react";
import { Meta, StoryObj } from "@storybook/react";

import { Icon } from "./icon/view";
import { lightOrDarkDecorator } from "./storybook/color_mode.js";

type StoryComponent = React.FunctionComponent<{}>;

const meta: Meta<StoryComponent> = {
  title: "Utilities/Auto Color Tailwind Utilities",
  decorators: [
    lightOrDarkDecorator({
      // Re-render story content on toggle to re-compute applied style colors
      reRenderOnToggle: true,
    }),
  ],
};

export default meta;

export const TextAutoUtility: StoryObj<StoryComponent> = {
  name: "text-auto utility",
  render: () => {
    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Text Color"
          subtitle="Use these utilities to set a text color that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="text-auto-primary" notes="Use for main UI text like titles and body content">
            Primary text
          </ColorRow>
          <ColorRow colorClassName="text-auto-secondary" notes="Use for secondary UI text">
            Secondary text
          </ColorRow>
          <ColorRow colorClassName="text-auto-disabled" notes="Use for disabled UI text">
            Disabled text
          </ColorRow>
          <ColorRow colorClassName="text-auto-hint" notes="Use for hint UI text (field labels, help text, etc)">
            Hint text
          </ColorRow>
        </ColorTable>

        <ColorTable
          title="Icon Color"
          subtitle="Use these utilities to set a color for icons that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="text-auto-icon" notes="Use for non-interactive icons">
            <div className="flex gap-4 items-center cursor-default select-none text-auto-icon" data-compute-color-here>
              <Icon>account_circle</Icon>
              <Icon>draft</Icon>
              <Icon>folder</Icon>
            </div>
          </ColorRow>
          <ColorRow
            colorClassName="text-auto-icon-button"
            notes="Use for custom icon buttons when not using <IconButton>. Sets the icon color and colors for the hover, focus, and disabled states."
          >
            <div className="flex gap-4 items-center">
              <button className="inline-flex text-auto-icon-button" data-compute-color-here>
                <Icon>home</Icon>
              </button>
              <button className="inline-flex text-auto-icon-button">
                <Icon>delete</Icon>
              </button>
              <button className="inline-flex text-auto-icon-button">
                <Icon>refresh</Icon>
              </button>
              <button className="inline-flex text-auto-icon-button" disabled>
                <Icon>arrow_back</Icon>
              </button>
              <button className="inline-flex text-auto-icon-button" disabled>
                <Icon>arrow_forward</Icon>
              </button>
            </div>
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const BgAutoUtility: StoryObj<StoryComponent> = {
  name: "bg-auto utility",
  render: () => {
    function Surface({ className }: { className: string }) {
      return <div className={`h-12 rounded-md border border-auto ${className}`} data-compute-color-here />;
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Surface Background Color"
          subtitle="Use these utilities to set a background color for UI surfaces like panels, cards, modals, etc that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="bg-auto-surface-0" notes="Use for UI surfaces at level 0">
            <Surface className="bg-auto-surface-0" />
          </ColorRow>
          <ColorRow colorClassName="bg-auto-surface-1" notes="Use for UI surfaces at level 1">
            <Surface className="bg-auto-surface-1" />
          </ColorRow>
          <ColorRow colorClassName="bg-auto-surface-2" notes="Use for UI surfaces at level 2">
            <Surface className="bg-auto-surface-2" />
          </ColorRow>
        </ColorTable>

        <ColorTable
          title="Contrast Background Color"
          subtitle="Use these utilities to create relative contrast against the current background. Useful for indicating states like hovered, pressed, and selected."
        >
          <ColorRow
            colorClassName="bg-auto-contrast-1"
            notes="Use to create relative contrast against the current background by 1."
          >
            <Surface className="bg-auto-contrast-1" />
          </ColorRow>
          <ColorRow
            colorClassName="bg-auto-contrast-2"
            notes="Use to create relative contrast against the current background by 2."
          >
            <Surface className="bg-auto-contrast-2" />
          </ColorRow>
          <ColorRow
            colorClassName="bg-auto-contrast-3"
            notes="Use to create relative contrast against the current background by 3."
          >
            <Surface className="bg-auto-contrast-3" />
          </ColorRow>
          <ColorRow
            colorClassName="bg-auto-contrast-4"
            notes="Use to create relative contrast against the current background by 4."
          >
            <Surface className="bg-auto-contrast-4" />
          </ColorRow>
          <ColorRow
            colorClassName="bg-auto-contrast-5"
            notes="Use to create relative contrast against the current background by 5."
          >
            <Surface className="bg-auto-contrast-5" />
          </ColorRow>
          <ColorRow
            colorClassName="bg-auto-contrast-6"
            notes="Use to create relative contrast against the current background by 6."
          >
            <Surface className="bg-auto-contrast-6" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const BorderAutoUtility: StoryObj<StoryComponent> = {
  name: "border-auto utility",
  render: () => {
    function BorderBox({ className }: { className: string }) {
      return <div className={`w-16 h-16 bg-auto-contrast-1 ${className}`} data-compute-color-here />;
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Border Color"
          subtitle="Use these utilities to set a border color that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="border-auto" notes="Use for borders in the UI">
            <BorderBox className="border-2 border-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-t-auto" notes="Use for top borders in the UI">
            <BorderBox className="border-t-2 border-t-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-b-auto" notes="Use for bottom borders in the UI">
            <BorderBox className="border-b-2 border-b-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-l-auto" notes="Use for left borders in the UI">
            <BorderBox className="border-l-2 border-l-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-r-auto" notes="Use for right borders in the UI">
            <BorderBox className="border-r-2 border-r-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-x-auto" notes="Use for horizontal borders in the UI">
            <BorderBox className="border-x-2 border-x-auto" />
          </ColorRow>
          <ColorRow colorClassName="border-y-auto" notes="Use for vertical borders in the UI">
            <BorderBox className="border-y-2 border-y-auto" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const DoubleRingUtility: StoryObj<StoryComponent> = {
  name: "double-ring utility",
  render: () => {
    function BorderBox({ className }: { className: string }) {
      return <div className={`w-14 h-14 bg-blue-500 ${className}`} data-compute-color-here />;
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Double Ring"
          subtitle="Use these utilities to set a high-contrast double ring (inner and outer) that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="double-ring" notes="Use for setting a double ring outline on elements in the UI">
            <BorderBox className="double-ring" />
          </ColorRow>
          <ColorRow
            colorClassName="double-ring-2"
            notes="Use for setting a double ring outline of 2px on elements in the UI"
          >
            <BorderBox className="double-ring double-ring-2" />
          </ColorRow>
          <ColorRow
            colorClassName="double-ring-4"
            notes="Use for setting a double ring outline of 4px on elements in the UI"
          >
            <BorderBox className="double-ring double-ring-4" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const DivideAutoUtility: StoryObj<StoryComponent> = {
  name: "divide-auto utility",
  render: () => {
    function DivideItems({ className, horizontal }: { className: string; horizontal?: boolean }) {
      return (
        <div
          className={`bg-auto-contrast-1 h-24 rounded-md overflow-hidden text-auto-primary text-center flex ${
            horizontal ? "flex-col" : "flex-row"
          } ${className}`}
        >
          <div className="flex-grow flex items-center justify-center">A</div>
          <div className="flex-grow flex items-center justify-center" data-compute-color-here>
            B
          </div>
          <div className="flex-grow flex items-center justify-center">C</div>
        </div>
      );
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Divider Color"
          subtitle="Use these utilities to set a divider color that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="divide-auto" notes="Use for vertical dividers in the UI">
            <DivideItems className="divide-y divide-auto" horizontal />
          </ColorRow>
          <ColorRow colorClassName="divide-auto" notes="Use for horizontal dividers in the UI">
            <DivideItems className="divide-x divide-auto" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const RingAutoUtility: StoryObj<StoryComponent> = {
  name: "ring-auto utility",
  render: () => {
    function BorderBox({ className }: { className: string }) {
      return <div className={`w-16 h-16 ${className}`} data-compute-color-here />;
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Ring Color"
          subtitle="Use this utility to set a ring color that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="ring-auto" notes="Use for ring color in the UI">
            <BorderBox className="ring-2 ring-auto" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

export const ShadowAutoCardUtility: StoryObj<StoryComponent> = {
  name: "shadow-auto-card utility",
  render: () => {
    function CardBox({ className }: { className: string }) {
      return <div className={`w-20 h-20 rounded ${className}`} data-compute-color-here />;
    }

    return (
      <div className="flex flex-col gap-8 text-red-500">
        <ColorTable
          title="Shadow Color"
          subtitle="Use this utility to set a box shadow with an outline (used for e.g. cards) that will automatically change to suit the color scheme in light or dark mode."
        >
          <ColorRow colorClassName="shadow-auto-card" notes="Use for card shadows in the UI">
            <CardBox className="shadow-auto-card" />
          </ColorRow>
        </ColorTable>
      </div>
    );
  },
};

function ColorTable({ title, subtitle, children }: { title: string; subtitle?: string; children: React.ReactNode }) {
  return (
    <div>
      <div className="text-xl font-semibold text-auto-primary">{title}</div>
      {subtitle ? <div className="mt-2 text-auto-secondary">{subtitle}</div> : null}
      <div className="mt-4 border border-auto rounded overflow-hidden">
        <table className="w-full border-collapse table-fixed">
          <tbody>{children}</tbody>
        </table>
      </div>
    </div>
  );
}

interface ColorRowProps {
  children: React.ReactNode;
  notes: React.ReactNode;
  colorClassName: string;
}

function ColorRow({ children, notes: usage, colorClassName }: ColorRowProps) {
  const rootRef = useRef<HTMLTableRowElement>(null);

  const computedColor = useComputedStyle(colorClassName, rootRef);

  const cellClassName = "border-t group-first:border-0 border-auto py-3 px-4 align-top";

  return (
    <tr ref={rootRef} className="group">
      <td className={`${cellClassName} w-1/4`}>
        {typeof children === "string" ? (
          <span className={colorClassName} data-compute-color-here>
            {children}
          </span>
        ) : (
          children
        )}
      </td>
      <td className={`${cellClassName} w-1/4 font-mono text-sm`}>
        <div className="text-auto-primary whitespace-nowrap">.{colorClassName}</div>
        <div className="text-auto-secondary whitespace-nowrap">{computedColor}</div>
      </td>
      <td className={`${cellClassName} text-auto-primary`}>{usage}</td>
    </tr>
  );
}

const classPrefixToCssProperty: Record<string, keyof CSSStyleDeclaration> = {
  "text-": "color",
  "bg-": "backgroundColor",
  "border-t-": "borderTopColor",
  "border-b-": "borderBottomColor",
  "border-l-": "borderLeftColor",
  "border-r-": "borderRightColor",
  "border-x-": "borderLeftColor",
  "border-y-": "borderTopColor",
  "border-": "borderColor",
  "divide-": "borderColor",
  "double-ring": "boxShadow",
  "ring-": "boxShadow",
  "shadow-auto-": "boxShadow",
};

function useComputedStyle(colorClassName: string, ref: React.RefObject<HTMLElement>) {
  const [color, setColor] = React.useState<string | null>(null);

  useLayoutEffect(() => {
    if (ref.current) {
      const element = ref.current.querySelector("[data-compute-color-here]");

      if (!element) {
        setColor("<color not found: no element with [data-compute-color-here]>");
        return;
      }

      const computedStyle = getComputedStyle(element);

      if (computedStyle) {
        for (const [prefix, cssProperty] of Object.entries(classPrefixToCssProperty)) {
          if (colorClassName.startsWith(prefix)) {
            const computedValue = computedStyle[cssProperty] as string;

            if (cssProperty === "boxShadow") {
              // The Tailwind shadow utilities use multiple variables to set the box-shadow. This parses
              // the box-shadow value to extract the colors.
              const colorsFromTailwindRingBoxShadow = computedValue.match(/rgba?(\(.+?\))/g);

              // For each utility, the number specifies (by index) which color in the box shadow definition is
              // the "main" one, to be extracted and displayed
              const colorIndexOfPrefix: Record<string, number> = {
                "double-ring": 1,
                "ring-": 1,
                "shadow-auto-": 3,
              };

              const index = colorIndexOfPrefix[prefix] ?? -1;
              setColor(
                colorsFromTailwindRingBoxShadow?.[index] ??
                  "<colorNotFound: failed to parse color from box-shadow value>",
              );
            } else {
              setColor(computedValue);
            }

            return;
          }
        }

        setColor("<colorNotFound: no matching prefix for className " + colorClassName + ">");
      } else {
        setColor("<colorNotFound: getComputedStyle returned null>");
      }
    }
  }, [ref, colorClassName]);

  return color;
}
