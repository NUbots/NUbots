import React from "react";
import { MessageInstance } from "@shared/messages/types";

interface VectorLike {
  x?: number;
  y?: number;
  z?: number;
  t?: number;
}

interface MatrixLike {
  x?: VectorLike;
  y?: VectorLike;
  z?: VectorLike;
  t?: VectorLike;
}

/** Filter the given object to include only the keys `x`, `y`, `z` and `t` */
function filterVectorEntries(message: object) {
  return Object.entries(message).filter(([key]) => ["x", "y", "z", "t"].includes(key));
}

/** Check whether a VectorLike is an integer type */
function isIntVec(vec: MessageInstance & VectorLike) {
  // The most reliable way to do this is by checking the name of the message.
  // The integer vector types begin with 'uvec' or 'ivec'.
  const type = vec.$typeName;
  return type.startsWith("uvec") || type.startsWith("ivec");
}

function Bracket({ type }: { type: "open" | "close" }) {
  return <div className={`w-1 shrink-0 border-y border-gray-400 ${type === "open" ? "border-l" : "border-r"}`} />;
}

function VectorElements({ vec }: { vec: MessageInstance & VectorLike }) {
  const isInt = isIntVec(vec);
  const formatValue = (value?: number | null) => {
    return (value ?? 0).toFixed(isInt ? 0 : 3);
  };

  return (
    <>
      {filterVectorEntries(vec).map(([key, val]) => (
        <span key={key}>{formatValue(val)}</span>
      ))}
    </>
  );
}

/**
 * Check whether an object is a VectorLike.
 * An object is VectorLike if its keys are a subset of 'x', 'y', 'z', 't' and all values are numbers.
 */
export function isVectorLike(value: object): value is VectorLike {
  const entries = filterVectorEntries(value);
  return (
    entries.length > 0 &&
    entries.length <= 4 &&
    entries.every(([key, val]) => ["x", "y", "z", "t"].includes(key) && typeof val === "number")
  );
}

/**
 * Check whether an object is a MatrixLike.
 * An object is MatrixLike if its keys are a subset of 'x', 'y', 'z', 't' and all values are VectorLike.
 */
export function isMatrixLike(value: object): value is MatrixLike {
  const entries = filterVectorEntries(value);
  return (
    entries.length > 0 &&
    entries.length <= 4 &&
    entries.every(([key, val]) => ["x", "y", "z", "t"].includes(key) && isVectorLike(val))
  );
}

export function VectorView({ vec }: { vec: MessageInstance & VectorLike }) {
  return (
    <div className="relative flex h-fit gap-3 text-auto-secondary">
      <Bracket type="open" />
      <VectorElements vec={vec} />
      <Bracket type="close" />
    </div>
  );
}

export function MatrixView({ matrix }: { matrix: MessageInstance & MatrixLike }) {
  return (
    <div className="relative flex h-fit gap-3 text-auto-secondary">
      <Bracket type="open" />
      {filterVectorEntries(matrix).map(([key, vec]) => (
        <div key={key} className="flex flex-col items-end">
          <VectorElements vec={vec} />
        </div>
      ))}
      <Bracket type="close" />
    </div>
  );
}
