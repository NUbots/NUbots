import React from "react";

export interface IconProps {
  /** The icon name */
  children: string;

  /**
   * Fill gives you the ability to modify the default icon style. The values are `false` for default
   * or `true` for completely filled. A single icon can render both unfilled and filled states.
   *
   * Default is `false` for no fill. To convey a state transition, use the fill axis for animation or interaction.
   */
  fill?: boolean;

  /**
   * Weight defines the symbol’s stroke weight, with a range of weights between thin (100)
   * and bold (700). Weight can also affect the overall size of the symbol.
   *
   * Default is 300. For consistency, stick to the default weight or use 600 for a bold icon.
   */
  weight?: 100 | 200 | 300 | 400 | 500 | 600 | 700 | "100" | "200" | "300" | "400" | "500" | "600" | "700";

  /**
   * Weight and grade affect a symbol’s thickness. Adjustments to grade are more granular than
   * adjustments to weight and have a small impact on the size of the symbol.
   *
   * You can use grade for different needs:
   *   - Low emphasis (e.g. -25 grade): To reduce glare for a light symbol on a dark background, use a low grade.
   *   - High emphasis (e.g. 200 grade): To highlight a symbol, increase the positive grade.
   */
  grade?: -25 | 0 | 200 | "-25" | "0" | "200";

  /**
   * Optical Sizes range from 20dp to 48dp. For the image to look the same at different sizes,
   * the stroke weight (thickness) changes as the icon size scales. Optical Size offers a way
   * to automatically adjust the stroke weight when you increase or decrease the symbol size.
   *
   * Defaults to the `size` prop or 24 if not specified.
   */
  opticalSize?: 20 | 24 | 40 | 48 | "20" | "24" | "40" | "48";

  /**
   * The size (width and height) of the icon in pixels. Converted to `rem` when set. Default is 24, which is 1.5rem.
   * For custom sizes, omit this prop and set a CSS `font-size` on the icon.
   */
  size?: 20 | 24 | 40 | 48 | "20" | "24" | "40" | "48";

  flip?: "none" | "horizontal" | "vertical" | "both";

  rotate?: 0 | 90 | 180 | 270 | "0" | "90" | "180" | "270";

  className?: string;

  style?: React.CSSProperties;
}

export function Icon(props: IconProps) {
  const { children, fill, weight, grade, opticalSize, size, rotate, flip, className } = props;

  const adjustments =
    fill !== undefined || weight !== undefined || grade !== undefined || opticalSize !== undefined || size !== undefined
      ? filterVars({
          "--mat-icon-fill": fill ? "1" : undefined,
          "--mat-icon-weight": weight,
          "--mat-icon-grade": grade,
          "--mat-icon-optical-size": opticalSize ?? size,
          "--mat-icon-size": size ? `${Number(size) / 16}rem` : undefined,
        })
      : undefined;

  const transform =
    flip !== undefined || rotate !== undefined
      ? filterVars({
          "--mat-icon-scaleX": flip === "horizontal" || flip === "both" ? "-1" : undefined,
          "--mat-icon-scaleY": flip === "vertical" || flip === "both" ? "-1" : undefined,
          "--mat-icon-rotate": rotate !== undefined ? `${rotate}deg` : undefined,
        })
      : undefined;

  const style =
    adjustments || transform || props.style ? Object.assign({}, props.style, adjustments, transform) : undefined;

  return (
    <span className={`material-symbols-outlined ${className ?? ""}`} style={style}>
      {children}
    </span>
  );
}

/** Filters out undefined values from the given object of CSS variables. */
function filterVars(
  style: Record<`--${string}`, string | number | undefined> | undefined,
): React.CSSProperties | undefined {
  if (style === undefined) {
    return undefined;
  }

  const result: React.CSSProperties = {};
  let hasVars = false;

  for (const [key, value] of Object.entries(style)) {
    if (value !== undefined) {
      (result as any)[key] = value;
      hasVars = true;
    }
  }

  return hasVars ? result : undefined;
}
