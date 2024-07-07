import React from "react";

export default function IconChart(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      viewBox="-100 -100 200 100"
      stroke="currentColor"
      fill="transparent"
      strokeWidth="10"
      width="200"
      height="100"
      xmlns="http://www.w3.org/2000/svg"
    >
      <path d="M0 0l-50-86.603M0 0v-100M0 0l50-86.603M0 0l-86.603-50L-50-86.603 0-100l50 13.397L86.603-50 0 0" />
      <path d="M-28.868-16.667l12.201-12.2L0-33.334l16.667 4.465 12.2 12.201" />
      <path d="M-57.735-33.333l24.402-24.402L0-66.667l33.333 8.932 24.402 24.402" />
    </svg>
  );
}
