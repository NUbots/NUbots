import React, { useEffect, useState } from "react";
import { Image, ImageFormat, jpegBufferToBitmap } from "@components/camera/image";
import { ObjectFitContain } from "@components/three/object_fit";
import { ImageView } from "@components/three/objects/image/image";
import { ThreeFiber } from "@components/three/three_fiber";
import { CompressedImage } from "@proto/message/output/CompressedImage";

export interface CompressedImageViewProps {
  msg: CompressedImage;
  children?: React.ReactNode;
}

export function CompressedImageView(props: CompressedImageViewProps) {
  const { msg, children } = props;
  const [image, setImage] = useState<Image | null>(null);

  useEffect(() => {
    let cancelled = false;
    jpegBufferToBitmap(msg.data).then((bitmap) => {
      if (cancelled) {
        bitmap.close();
        return;
      }
      setImage({
        type: "bitmap",
        bitmap,
        width: msg.dimensions?.x ?? 0,
        height: msg.dimensions?.y ?? 0,
        format: ImageFormat.JPEG,
      });
    });
    return () => {
      cancelled = true;
    };
  }, [msg]);

  return (
    <>
      <div className="relative my-0.5 aspect-video max-h-60 w-full">
        <ThreeFiber orthographic>
          {image ? (
            <ObjectFitContain width={image.width} height={image.height}>
              <ImageView image={image} />
            </ObjectFitContain>
          ) : null}
        </ThreeFiber>
      </div>
      {children}
    </>
  );
}
