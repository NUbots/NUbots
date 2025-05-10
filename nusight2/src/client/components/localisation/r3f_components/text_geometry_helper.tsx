import { useEffect, useState } from "react";
import { TextGeometry } from "three/examples/jsm/geometries/TextGeometry";
import type { Font } from "three/examples/jsm/loaders/FontLoader";
import { FontLoader } from "three/examples/jsm/loaders/FontLoader";

export const useTextGeometry = (text: string): TextGeometry | null => {
  const [geometry, setGeometry] = useState<TextGeometry | null>(null);

  useEffect(() => {
    const loader = new FontLoader();

    loader.load("/fonts/roboto/Roboto_Medium_Regular.json", (font: Font) => {
      const newGeometry = new TextGeometry(text, {
        font,
        size: 0.1,
        depth: 0,
      });
      setGeometry(newGeometry);
    });
  }, [text]);

  return geometry;
};
