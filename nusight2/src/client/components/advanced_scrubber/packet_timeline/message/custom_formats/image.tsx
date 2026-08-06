import React, { useEffect, useRef } from "react";
import { CompressedImage } from "@proto/message/output/CompressedImage";

export interface CompressedImageViewProps {
  msg: CompressedImage;
  children?: React.ReactNode;
}

/**
 * Renders a CompressedImage message as an image preview.
 *
 * Uses a canvas element with createImageBitmap for JPEG decoding.
 */
export function CompressedImageView(props: CompressedImageViewProps) {
  const { msg, children } = props;
  const canvasRef = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !msg.data?.length) {
      return;
    }

    const blob = new Blob([msg.data], { type: "image/jpeg" });
    createImageBitmap(blob, { colorSpaceConversion: "none", premultiplyAlpha: "none" })
      .then((bitmap) => {
        const ctx = canvas.getContext("2d");
        if (!ctx) return;
        canvas.width = bitmap.width;
        canvas.height = bitmap.height;
        ctx.drawImage(bitmap, 0, 0);
        bitmap.close();
      })
      .catch(() => {
        // Non-JPEG formats are not decoded here; show nothing
      });
  }, [msg]);

  return (
    <>
      <canvas
        ref={canvasRef}
        className="my-0.5 max-h-60 w-full object-contain bg-black"
        style={{ imageRendering: "pixelated" }}
      />
      {children}
    </>
  );
}
