import React from "react";

export default function IconLocalisation(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      viewBox="0 0 32 32"
      xmlns="http://www.w3.org/2000/svg"
      fill="none"
      stroke="currentColor"
      strokeLinejoin="round"
      strokeWidth="2"
    >
      <path d="M21 2L11 6 1 2v24l10 4 10-4 10 4V6L21 2zM16 16a4 4 0 0 1-8 0M16 16a4 4 0 0 1 8 0" />
      <circle cx="8" cy="12" r="1" />
      <circle cx="24" cy="20" r="1" />
      <path d="M11 30v-5M21 7V2" />
    </svg>
  );
}
