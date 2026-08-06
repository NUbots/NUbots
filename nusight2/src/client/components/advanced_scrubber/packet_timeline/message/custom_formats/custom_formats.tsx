import React from "react";
import { CompressedImage } from "@proto/message/output/CompressedImage";
import { MessageInstance, MessageType } from "@shared/messages/types";
import { Duration } from "@shared/proto/google/protobuf/duration";
import { Timestamp as ProtobufTimestamp } from "@shared/proto/google/protobuf/timestamp";
import { Timestamp } from "@shared/time/timestamp";

import { BytesView } from "./bytes";
import { CompressedImageView } from "./image";
import { isMatrixLike, isVectorLike, MatrixView, VectorView } from "./math";

function isMessageInstance(value: unknown): value is MessageInstance<string> {
  return typeof value === "object" && value !== null && "$typeName" in value;
}

function isMessageType<T extends MessageInstance<string>>(
  value: MessageInstance<string>,
  type: MessageType<T>,
): value is T {
  return value.$typeName === type.typeName;
}

/** Get a custom display for the given value if one exists */
export function formatValue(value: unknown): { content: React.ReactNode; keepOriginal?: boolean } | null {
  // Skip checking how to format if value is a primitive.
  // Primitives can just be rendered as strings.
  if (!isMessageInstance(value)) {
    return null;
  }

  if (value instanceof Uint8Array) {
    return { content: <BytesView value={value} /> };
  } else if (isMessageType(value, ProtobufTimestamp)) {
    return { content: <TimestampView timestamp={value} /> };
  } else if (isMessageType(value, Duration)) {
    return { content: <DurationView duration={value} /> };
  } else if (isVectorLike(value)) {
    return { content: <VectorView vec={value} /> };
  } else if (isMatrixLike(value)) {
    return { content: <MatrixView matrix={value} /> };
  } else if (isMessageType(value, CompressedImage)) {
    return {
      content: <CompressedImageView msg={value} />,
      keepOriginal: true,
    };
  }

  return null;
}

function TimestampView(props: { timestamp: ProtobufTimestamp }) {
  const { timestamp } = props;
  const nanos = Timestamp.toNanos({ seconds: timestamp.seconds, nanos: timestamp.nanos });
  return <div className="text-auto-secondary w-full">Timestamp: {Timestamp.toFormattedDate(nanos)}</div>;
}

function DurationView(props: { duration: Duration }) {
  const { duration } = props;
  const nanos = Timestamp.toNanos({ seconds: duration.seconds, nanos: duration.nanos });
  return <div className="text-auto-secondary w-full">Duration: {Timestamp.toFormattedDuration(nanos)}</div>;
}
