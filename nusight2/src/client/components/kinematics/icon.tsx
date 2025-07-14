import React from "react";

export default function IconServos(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 100 100"
      fill="none"
      stroke="currentColor"
      strokeLinejoin="round"
      strokeWidth="3"
    >
      {/* Main gear */}
      <circle cx="50" cy="50" r="25" />
      <circle cx="50" cy="50" r="15" />

      {/* Gear teeth */}
      <line x1="50" y1="15" x2="50" y2="25" />
      <line x1="50" y1="75" x2="50" y2="85" />
      <line x1="15" y1="50" x2="25" y2="50" />
      <line x1="75" y1="50" x2="85" y2="50" />

      {/* Diagonal teeth */}
      <line x1="25" y1="25" x2="35" y2="35" />
      <line x1="65" y1="65" x2="75" y2="75" />
      <line x1="25" y1="75" x2="35" y2="65" />
      <line x1="65" y1="35" x2="75" y2="25" />

      {/* Small servo indicators */}
      <circle cx="20" cy="20" r="8" />
      <circle cx="80" cy="20" r="8" />
      <circle cx="20" cy="80" r="8" />
      <circle cx="80" cy="80" r="8" />

      {/* Temperature indicator */}
      <path d="M 30 70 Q 40 60, 50 70 Q 60 80, 70 70" stroke="red" strokeWidth="2" />
    </svg>
  );
}
