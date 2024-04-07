import React from "react";

export default function IconProfiler(props: { className?: string }) {
  return (
    <svg className={props.className} version="1.1" xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512" fill="#fff">
      <path d="M128,384v96H32v-96H128z M192,0v480h-96V0H192z M320,256v224h-96V256H320z M384,128v352h-96V128H384z M512,192v288h-96V192H512z" />
    </svg>
  );
}
