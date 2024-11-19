import React from "react";

export default function IconPendulum(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 100 100"
      fill="none"
      stroke="currentColor"
      strokeLinejoin="round"
      strokeWidth="5"
    >
      <path d="M 10 60 Q 50 90, 90 60" />

      <line x1="50" y1="10" x2="50" y2="70" />
      <circle cx="50" cy="70" r="10" fill="currentColor" />

      <line x1="50" y1="10" x2="15" y2="50" />
      <circle cx="15" cy="60" r="10" fill="currentColor" />

      <line x1="50" y1="10" x2="85" y2="50" />
      <circle cx="85" cy="60" r="10" fill="currentColor" />

    </svg>
  );
}
