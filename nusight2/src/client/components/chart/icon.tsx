import React from "react";

export default function IconChart(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      height="32"
      viewBox="0 0 32 32"
      width="32"
      xmlns="http://www.w3.org/2000/svg"
      fill="currentColor"
    >
      <path d="M2 30h30v2H0V0h4v2H2v4h2v2H2v4h2v2H2v4h2v2H2v4h2v2H2v4zm22.916-16.962C24.801 14.805 24.656 17 24 17c-1.17 0-1.49-.636-2.051-2.316C21.43 13.129 20.721 11 18 11c-2.848 0-3.451 3.633-3.985 6.836C13.631 20.137 13.155 23 12 23c-2.041 0-2.996-5.371-3-9H7c0 1.125.142 11 5 11 2.849 0 3.452-3.631 3.986-6.837C16.371 15.862 16.846 13 18 13c1.17 0 1.49.635 2.051 2.315C20.571 16.871 21.28 19 24 19c2.529 0 2.726-2.966 2.912-5.828C27.226 8.387 27.773 5 32 5V3c-6.428 0-6.822 6.042-7.084 10.038z" />
    </svg>
  );
}
