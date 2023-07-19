declare module "react-resize-detector" {
  import { ComponentType } from "react";

  const component: ComponentType<{
    handleWidth?: boolean;
    handleHeight?: boolean;
    onResize: (width: number, height: number) => void;
  }>;

  export default component;
}
