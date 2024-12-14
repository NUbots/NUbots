import { useEffect, useState } from "react";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";

export const TextGeometryHelper = (text: string): TextGeometry => {
  const [geometry, setGeometry] = useState<TextGeometry | null>(null);

  useEffect(() => {
    const loader = new FontLoader();

    // Load font asynchronously
    loader.load("/fonts/roboto/Roboto Medium_Regular.json", (font: any) => {
      const newGeometry = new TextGeometry(text, {
        font: font,
        size: 0.1,
        height: 0,
      });
      setGeometry(newGeometry);
    });

  }, [text]);

  return geometry || new TextGeometry("");
};
