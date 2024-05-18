import { IconButton, IconButtonProps } from "../icon_button/view";
import React, { useState, useEffect } from "react";

export default function ThemeSwitcherButton(props: IconButtonProps) {
    const [isDark, setIsDark] = useState(
        window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches
    );

    useEffect(() => {
        if (isDark) {
            document.documentElement.classList.add('dark');
        } else {
            document.documentElement.classList.remove('dark');
        }
    }, [isDark]);

    const handleClick = () => {
        setIsDark(!isDark);
    };

    return (
        <IconButton {...props} onClick={handleClick}>
            {isDark ? 'dark_mode' : 'light_mode'}
        </IconButton>
    );
}
