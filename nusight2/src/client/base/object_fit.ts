/** Based on the object-fit CSS property: https://developer.mozilla.org/en-US/docs/Web/CSS/object-fit */
export type ObjectFit =
  // Stretch content to fill entire container.
  | { type: "fill" }
  // Either cover the container with content, or contain the content in the container, while maintaining aspect ratio.
  | { type: "contain" | "cover"; aspect: number };

/** Given a container size and an object fit mode, get a width and height that will apply the object fit mode */
export function getObjectFit(
  container: { width: number; height: number },
  objectFit: ObjectFit,
): { width: number; height: number } {
  switch (objectFit.type) {
    case "fill":
      return { width: container.width, height: container.height };
    case "contain":
    case "cover": {
      const containerAspect = container.width / container.height;
      const ratio = containerAspect / objectFit.aspect;

      if (objectFit.type === "contain" ? ratio < 1 : ratio >= 1) {
        // Take entire container width, scale the height
        return { width: container.width, height: Math.ceil(container.width / objectFit.aspect) };
      } else {
        // Take entire container height, scale the width
        return { width: Math.ceil(container.height * objectFit.aspect), height: container.height };
      }
    }
    default:
      throw new Error(`unknown object fit mode: ${objectFit}`);
  }
}
