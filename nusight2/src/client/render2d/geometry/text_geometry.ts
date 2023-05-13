import { observable } from "mobx";

export class TextGeometry {
  @observable worldAlignment: boolean;
  @observable worldScale: boolean;
  @observable fontFamily: string;
  @observable fontSize: string;
  @observable text: string;
  @observable textAlign: "start" | "middle" | "end" | "left" | "right" | "center";
  @observable textBaseline: "top" | "hanging" | "middle" | "alphabetic" | "ideographic" | "bottom";
  @observable x: number;
  @observable y: number;

  constructor(opts: TextGeometry) {
    this.worldAlignment = opts.worldAlignment;
    this.worldScale = opts.worldScale;
    this.fontFamily = opts.fontFamily;
    this.fontSize = opts.fontSize;
    this.text = opts.text;
    this.textAlign = opts.textAlign;
    this.textBaseline = opts.textBaseline;
    this.x = opts.x;
    this.y = opts.y;
  }

  static of({
    worldAlignment = false,
    worldScale = false,
    fontFamily = "sans-serif",
    fontSize = "1em",
    text = "",
    textAlign = "start",
    textBaseline = "alphabetic",
    x = 0,
    y = 0,
  }: Partial<TextGeometry> = {}): TextGeometry {
    return new TextGeometry({
      worldAlignment,
      worldScale,
      fontFamily,
      fontSize,
      text,
      textAlign,
      textBaseline,
      x,
      y,
    });
  }
}
