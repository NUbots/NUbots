import React, { useMemo, useState } from "react";
import { siUnit } from "@client/base/si_unit";
import classNames from "classnames";

// The number of bytes to display on each row
const SECTION_LENGTH = 8;

/** Splits a Uint8Array into views of a specified size */
function splitArray(array: Uint8Array, size: number) {
  const sections = Math.ceil(array.length / size);
  return new Array(sections).fill(null).map((_, i) => array.subarray(i * size, i * size + size));
}

/** Render a row of formatted bytes */
function FormattedBytes(props: {
  bytes: Uint8Array;
  highlighted: number;
  onHover: (index: number) => void;
  formatByte: (byte: number) => string;
}) {
  const { bytes, highlighted, onHover, formatByte } = props;
  return (
    <div>
      {Array.from(bytes).map((byte, i) => (
        <span
          key={i}
          onMouseEnter={() => onHover(i)}
          onMouseLeave={() => onHover(-1)}
          className={classNames(
            "px-1 rounded-sm hover:bg-nusight-500/30",
            highlighted === i ? "bg-nusight-500/15" : "",
          )}
        >
          {formatByte(byte)}
        </span>
      ))}
    </div>
  );
}

/**
 * A row in the custom display for bytes.
 * Displays the offset of this set of bytes and the bytes themselves in hex and ASCII.
 */
function BytesViewRow(props: { offset: number; bytes: Uint8Array }) {
  const { offset, bytes } = props;

  const [highlighted, setHighlighted] = useState(-1);
  const divider = <div className="border-r border-auto" />;

  return (
    <div className="flex gap-2 cursor-default">
      {/* Offset of this set of bytes in hex */}
      <div className="text-auto-hint">{offset.toString(16).padStart(8, "0")}</div>
      {divider}
      {/* Bytes as their hex values */}
      <FormattedBytes
        bytes={bytes}
        highlighted={highlighted}
        onHover={setHighlighted}
        formatByte={(byte) => byte.toString(16).padStart(2, "0")}
      />
      {divider}
      {/* Bytes as ASCII characters, only converting the useful range */}
      <FormattedBytes
        bytes={bytes}
        highlighted={highlighted}
        onHover={setHighlighted}
        formatByte={(byte) => (byte > 32 && byte < 127 ? String.fromCharCode(byte) : ".")}
      />
    </div>
  );
}

export function BytesView(props: { value: Uint8Array }) {
  const [shownRows, setShownRows] = useState(10);

  const shownBytes = useMemo(
    () => props.value.subarray(0, Math.min(SECTION_LENGTH * shownRows, props.value.length)),
    [shownRows, props.value],
  );

  const sections = useMemo(() => splitArray(shownBytes, SECTION_LENGTH), [shownBytes]);
  const remainingBytes = props.value.length - shownBytes.length;
  const [formattedRemaining, units] = siUnit(remainingBytes, "b");

  if (props.value.length === 0) {
    return <div className="text-auto-secondary">(empty)</div>;
  }

  return (
    <div className="text-auto-secondary">
      {sections.map((bytes, i) => (
        <BytesViewRow key={i} offset={i * SECTION_LENGTH} bytes={bytes} />
      ))}
      {remainingBytes > 0 ? (
        <div
          onClick={() => setShownRows((value) => value + 10)}
          className="hover:text-blue-700 dark:hover:text-blue-500 cursor-pointer mt-2"
        >
          +{formattedRemaining.toFixed(2)}
          {units} more (click to show)
        </div>
      ) : null}
    </div>
  );
}
