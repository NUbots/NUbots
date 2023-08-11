# Icon component

The [`view.tsx`](./view.tsx) file in this folder exports an `Icon` component that can be used to show an icon from the [Material Symbols](https://fonts.google.com/icons?icon.style=Outlined) icon set. The icons used are from the "Outlined" style.

Icons are rendered using a variable font, which is loaded in NUsight's root `index.html` file. This allows for dynamically adjusting the following properties of an icon, without having to load and switch between different fonts:

- `fill` - whether the icon is visually "filled in". Default is `false` for no fill.
- `weight` - the stroke weight (i.e. thickness of the icon's lines). Ranges from `100` to `700`. Default weight is `300`.
- `grade` - the "emphasis" of the icon - lower grades are less visually prominent. Default grade is `0`, for no emphasis.
- `opticalSize` - provides a way to automatically adjust the stroke weight when the icon is rendered at smaller or larger sizes. Default optical size is `24`, assuming most icons will be rendered at `24px`.
- `size` - the size of the icon, in pixels. This value is converted to `rem` when set. Default size is `24`, which is `1.5rem`.
- `flip` - whether the icon is flipped horizontally, vertically, or both. Default is `none`, for no flipping.
- `rotate` - the angle (in degrees) to rotate the icon. Default is `0`, for no rotation.

The icon's color and size can be adjusted using CSS `color` and `font-size` properties, respectively.

See https://m3.material.io/styles/icons/applying-icons for more information on these adjustable properties, including examples.

## Usage

An icon is shown by rendering the `<Icon>` component with the icon's name as the child. The icon's name can be found in the [Material Symbols](https://fonts.google.com/icons?icon.style=Outlined) icon set, converted to lowercase with spaces replaced by underscores. For example, the "Arrow Forward" icon from the set is rendered using `<Icon>arrow_forward</Icon>`.

```tsx
import { Icon } from './src/components/icon/view'

// Show a checkmark in a circle
<Icon>check_circle</Icon>

// Show a large checkmark in a circle
<Icon size="48">check_circle</Icon>

// Show a filled in checkmark in a circle
<Icon fill>check_circle</Icon>

// Show a thin checkmark in a circle
<Icon weight="200">check_circle</Icon>

// Show a green checkmark in a circle
<Icon className="text-green-500">check_circle</Icon>

// Show an arrow forward icon, rotated to point up
<Icon rotate={270}>arrow_forward</Icon>

// Show an arrow forward icon, flipped to point left
<Icon flip="horizontal">arrow_forward</Icon>
```

> **Note**
> If you find an icon in the library that doesn't work with the component, it could be a newer icon that is not available in the local copy of the font we have. If this is the case, you can update our font file (at `src/assets/fonts/material-symbols/material-symbols-outlined.woff2`) to the latest version of the file from [this repo](https://github.com/marella/material-symbols/tree/main/material-symbols), which tracks Google's latest releases of the font files.
