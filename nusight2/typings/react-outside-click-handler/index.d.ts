/// <reference path="react" />

declare module "react-outside-click-handler/esm/OutsideClickHandler" {
  export interface DefaultProps {
    disabled: boolean;
    useCapture: boolean;
    display: "block" | "flex" | "inline" | "inline-block" | "contents";
  }
  export interface Props extends DefaultProps {
    children: React.ReactNode;
    onOutsideClick: (e: React.MouseEvent<HTMLElement>) => void;
  }
  export default class OutsideClickHandler extends React.Component<Props> {
    static defaultProps: DefaultProps;
  }
}
