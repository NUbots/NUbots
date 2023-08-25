import React from "react";

export function IconClose(props: { className?: string }) {
  return (
    <svg className={props.className} fill="currentColor" width="28" height="28" viewBox="0 0 24 24">
      <path d="M12 13.05 6.925 18.125Q6.725 18.325 6.413 18.337Q6.1 18.35 5.875 18.125Q5.65 17.9 5.65 17.6Q5.65 17.3 5.875 17.075L10.95 12L5.875 6.925Q5.675 6.725 5.663 6.412Q5.65 6.1 5.875 5.875Q6.1 5.65 6.4 5.65Q6.7 5.65 6.925 5.875L12 10.95L17.075 5.875Q17.275 5.675 17.588 5.662Q17.9 5.65 18.125 5.875Q18.35 6.1 18.35 6.4Q18.35 6.7 18.125 6.925L13.05 12L18.125 17.075Q18.325 17.275 18.337 17.587Q18.35 17.9 18.125 18.125Q17.9 18.35 17.6 18.35Q17.3 18.35 17.075 18.125Z" />
    </svg>
  );
}

export function IconFile(props: { className?: string }) {
  return (
    <svg className={props.className} xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
      <path d="M4 18h12v-12h-4v-4h-8v16zM2 19v-19h12l4 4v16h-16v-1z"></path>
    </svg>
  );
}

export function IconFolder(props: { className?: string }) {
  return (
    <svg className={props.className} xmlns="http://www.w3.org/2000/svg" viewBox="0 0 20 20" fill="currentColor">
      <path d="M0 4c0-1.1.9-2 2-2h7l2 2h7a2 2 0 0 1 2 2v10a2 2 0 0 1-2 2H2a2 2 0 0 1-2-2V4z" />
    </svg>
  );
}

export function IconChevron(props: { className?: string }) {
  return (
    <svg className={props.className} fill="currentColor" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
      <path d="M17.7 34.9Q17.3 34.4 17.275 33.8Q17.25 33.2 17.7 32.75L26.5 23.95L17.65 15.1Q17.25 14.7 17.275 14.025Q17.3 13.35 17.7 12.95Q18.2 12.45 18.775 12.475Q19.35 12.5 19.8 12.95L29.75 22.9Q30 23.15 30.1 23.4Q30.2 23.65 30.2 23.95Q30.2 24.25 30.1 24.5Q30 24.75 29.75 25L19.85 34.9Q19.4 35.35 18.8 35.325Q18.2 35.3 17.7 34.9Z" />
    </svg>
  );
}

export function IconArrowBackward(props: { className?: string }) {
  return (
    <svg className={props.className} fill="currentColor" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
      <path d="M22.35 38.95 8.45 25.05Q8.2 24.8 8.1 24.55Q8 24.3 8 24Q8 23.7 8.1 23.45Q8.2 23.2 8.45 22.95L22.4 9Q22.8 8.6 23.4 8.6Q24 8.6 24.45 9.05Q24.9 9.5 24.9 10.1Q24.9 10.7 24.45 11.15L13.1 22.5H37.9Q38.55 22.5 38.975 22.925Q39.4 23.35 39.4 24Q39.4 24.65 38.975 25.075Q38.55 25.5 37.9 25.5H13.1L24.5 36.9Q24.9 37.3 24.9 37.9Q24.9 38.5 24.45 38.95Q24 39.4 23.4 39.4Q22.8 39.4 22.35 38.95Z" />
    </svg>
  );
}

export function IconArrowForward(props: { className?: string }) {
  return (
    <svg className={props.className} fill="currentColor" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
      <path d="M22.95 38.9Q22.55 38.5 22.55 37.85Q22.55 37.2 22.95 36.8L34.25 25.5H9.5Q8.85 25.5 8.425 25.075Q8 24.65 8 24Q8 23.35 8.425 22.925Q8.85 22.5 9.5 22.5H34.25L22.95 11.2Q22.55 10.8 22.55 10.125Q22.55 9.45 22.95 9.05Q23.35 8.65 24 8.65Q24.65 8.65 25.05 9.05L38.95 22.95Q39.2 23.2 39.3 23.45Q39.4 23.7 39.4 24Q39.4 24.25 39.3 24.525Q39.2 24.8 38.95 25.05L25.05 38.95Q24.65 39.35 24 39.325Q23.35 39.3 22.95 38.9Z" />
    </svg>
  );
}

export function IconArrowUpward(props: { className?: string }) {
  return (
    <svg className={props.className} fill="currentColor" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 48 48">
      <path d="M24 40Q23.35 40 22.925 39.575Q22.5 39.15 22.5 38.5V13.7L11.15 25.05Q10.7 25.5 10.1 25.5Q9.5 25.5 9.05 25.05Q8.6 24.6 8.6 24Q8.6 23.4 9.05 22.95L22.95 9.05Q23.2 8.8 23.475 8.7Q23.75 8.6 24 8.6Q24.3 8.6 24.55 8.7Q24.8 8.8 25.05 9.05L38.95 22.95Q39.4 23.4 39.4 24Q39.4 24.6 38.95 25.05Q38.5 25.5 37.9 25.5Q37.3 25.5 36.85 25.05L25.5 13.7V38.5Q25.5 39.15 25.075 39.575Q24.65 40 24 40Z" />
    </svg>
  );
}

export function IconError(props: { className?: string }) {
  return (
    <svg
      className={props.className}
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth="2"
      strokeLinecap="round"
      strokeLinejoin="round"
    >
      <circle cx="12" cy="12" r="10" />
      <line x1="12" y1="8" x2="12" y2="12" />
      <line x1="12" y1="16" x2="12.01" y2="16" />
    </svg>
  );
}
