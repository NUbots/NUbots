# ResizeContainer Component

This folder exports a `ResizeContainer` component from [`resize_container.tsx`](./resize_container.tsx) and `ResizePanel` component from [`resize_panel.tsx`](./resize_panel.tsx) which combine to create a layout that can be resized at runtime by dragging the resize handles rendered between the panels.

The `ResizeContainer` component can have `ResizePanel`s as its children and is responsible for managing the size of the panels and rendering the handles between them.

The `ResizePanel` provides props for managing attributes of each individual panel such as sizing requirements.

## Usage

A resize container must contain `<ResizePanel>` elements as direct children. To nest containers, the order of elements should be `<ResizeContainer>` → `<ResizePanel>` → `<ResizeContainer>` → `<ResizePanel>`.

The `<ResizePanel>` component exposes a custom ref object with a method for toggling the panel's state between "minimized", "maximized" and "default".

```tsx
import { ResizeContainer } from './src/components/resize_container/resize_container';
import { ResizePanel } from './src/components/resize_container/resize_panel';
import { Button } from './src/components/button/button';

// Container with an ID to save its children's positions between page reloads
<ResizeContainer saveKey="main-layout">
  <ResizePanel>Content 1</ResizePanel>
  <ResizePanel>Content 2</ResizePanel>
</ResizeContainer>

// Container with two panels of different minimum and maximum sizes
<ResizeContainer>
  <ResizePanel minSize={100} maxSize={400}>Content 1</ResizePanel>
  <ResizePanel minSize={200} maxSize={500}>Content 2</ResizePanel>
</ResizeContainer>

// Nested container, where the inner container's children are laid out horizontally
<ResizeContainer>
  <ResizePanel minSize={200}>
    <ResizeContainer horizontal>
      <ResizePanel minSize={100}>Content 1.1</ResizePanel>
      <ResizePanel minSize={200}>Content 1.2</ResizePanel>
    </ResizeContainer>
  </ResizePanel>

  <ResizePanel minSize={200}>Content 2</ResizePanel>
</ResizeContainer>

// Render buttons in a panel to toggle it between its possible states,
// and show a message displaying the panel's current state
const panelRef = useRef<ResizePanelRef>(null);
const [currentState, setCurrentState] = useState("default");
<ResizeContainer>
  <ResizePanel ref={panelRef} minSize={100} onStateChange={setCurrentState}>
    <Button onClick={() => panelRef.current?.setState("minimized")}>
      Set Minimized
    </Button>
    <Button onClick={() => panelRef.current?.setState("maximized")}>
      Set Maximized
    </Button>
    <Button onClick={() => panelRef.current?.setState("default")}>
      Set Default
    </Button>
    <span>Current: {currentState}</span>
  </ResizePanel>
  <ResizePanel minSize={200}>Content 2</ResizePanel>
</ResizeContainer>
```
