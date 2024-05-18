import React from "react";

export function IconZoomIn(props: { className?: string }) {
  return (
    <svg className={props.className} viewBox="0 0 24 24"  xmlns="http://www.w3.org/2000/svg">
      <line x1="10.5" y1="7.75" x2="10.5" y2="13.25" strokeWidth="1.5" strokeLinecap="round" />
      <line x1="7.75" y1="10.5" x2="13.25" y2="10.5" strokeWidth="1.5" strokeLinecap="round" />
      <path d="M19.5 19.5L15 15" strokeWidth="1.5" strokeLinecap="round" />
      <circle cx="10.5" cy="10.5" r="5.75" fill="none" strokeWidth="1.5" />
    </svg>
  );
}

export function IconZoomOut(props: { className?: string }) {
  return (
    <svg className={props.className} viewBox="0 0 24 24"  xmlns="http://www.w3.org/2000/svg">
      <circle cx="10.5" cy="10.5" r="5.75" fill="none" strokeWidth="1.5" />
      <path d="M19.5 19.5L15 15" strokeWidth="1.5" strokeLinecap="round" />
      <line x1="7.75" y1="10.5" x2="13.25" y2="10.5" strokeWidth="1.5" strokeLinecap="round" />
    </svg>
  );
}

export function IconZoomReset(props: { className?: string }) {
  return (
    <svg className={props.className} viewBox="0 0 24 24" xmlns="http://www.w3.org/2000/svg">
      <path
        fillRule="evenodd"
        clipRule="evenodd"
        d="M14.4697 14.4697C14.7626 14.1768 15.2374 14.1768 15.5303 14.4697L20.0303 18.9697C20.3232 19.2626 20.3232 19.7374 20.0303 20.0303C19.7374 20.3232 19.2626 20.3232 18.9697 20.0303L14.4697 15.5303C14.1768 15.2374 14.1768 14.7626 14.4697 14.4697Z"
      />
      <path
        fillRule="evenodd"
        clipRule="evenodd"
        d="M5.37596 6.58397C6.54442 4.83129 8.6389 4 10.5 4C14.0722 4 17 6.8294 17 10.5C17 13.6526 14.4723 17 10.5 17C6.52772 17 4 13.6526 4 10.5C4 10.0858 4.33579 9.75 4.75 9.75C5.16421 9.75 5.5 10.0858 5.5 10.5C5.5 12.9453 7.47228 15.5 10.5 15.5C13.5277 15.5 15.5 12.9453 15.5 10.5C15.5 7.67212 13.2582 5.5 10.5 5.5C9.02443 5.5 7.45558 6.16871 6.62404 7.41603C6.39427 7.76067 5.92862 7.8538 5.58397 7.62404C5.23933 7.39427 5.1462 6.92862 5.37596 6.58397Z"
      />
      <path
        fillRule="evenodd"
        clipRule="evenodd"
        d="M5.75 8C5.33579 8 5 7.66421 5 7.25L5 4.75C5 4.33579 5.33579 4 5.75 4C6.16421 4 6.5 4.33579 6.5 4.75L6.5 7.25C6.5 7.66421 6.16421 8 5.75 8Z"
      />
      <path
        fillRule="evenodd"
        clipRule="evenodd"
        d="M5 7.25C5 6.83579 5.33579 6.5 5.75 6.5H8.25C8.66421 6.5 9 6.83579 9 7.25C9 7.66421 8.66421 8 8.25 8H5.75C5.33579 8 5 7.66421 5 7.25Z"
      />
    </svg>
  );
}
