# Camera Text Renderer

The [`text.ts`](./text.ts) file in this folder exports a `TextViewModel` class which can generate THREE.JS meshes of text from strings to render text in a camera view.

Text meshes generated by the renderer have the option of being positioned either by a ray from the camera's perspective or by a pixel in the camera's image. There are also multiple options for styling the text geometry including:

- `textColor` - The colour of the text. Defaults to `"white"`.
- `backgroundColor` - The colour of the background behind the text. Defaults to `"transparent"`.
- `height` - A height in pixels of the text geometry. Defaults to `20`px.
- `align` - Horizontal alignment of the text. Can be either `"start"`, `"middle"`, or `"end"`.
- `baseline` - Vertical alignment of the text. Can be either `"top"`, `"middle"`, or `"bottom"`.

## Usage

```ts
// Create a Text Renderer (from Camera Model attributes)
const renderer = new TextViewModel(canvas, params, image);

// Create options for text positioned by a ray
const rayTextOpts: TextOpts = {
  type: "ray",
  ray: Vector3.of(x, y, z), // Ray from camera space
  text: "Example Text",
};

// Create options for text positioned by an image pixel
const pixelTextOpts: TextOpts = {
  type: "pixel",
  pixel: Vector2.of(x, y), // Image pixel coordinate
  text: "Example Text",
};

// Create text geometry
const rayTextGeometry = renderer.text(rayTextOpts);
const pixelTextGeometry = renderer.text(pixelTextOpts);

// Use the geometry to draw the text... (see the story linked below)
```

For a complete example, view the [Text Renderer Story](./stories/text.stories.tsx).