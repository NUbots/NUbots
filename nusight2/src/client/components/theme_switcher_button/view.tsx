import { IconButton, IconButtonProps } from "../icon_button/view";
import React, { useState, useEffect, useMemo } from "react";

export default function ThemeSwitcherButton(props: IconButtonProps) {
    const [isDark, setIsDark] = useState(
        window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches
    );

    const mediaQuery = useMemo(() => window.matchMedia('(prefers-color-scheme: dark)'), []);

    useEffect(() => {
        const changeHandler = () => setIsDark(mediaQuery.matches);

        mediaQuery.addEventListener('change', changeHandler);

        return () => {
            mediaQuery.removeEventListener('change', changeHandler);
        };
    }, [mediaQuery]);

    useEffect(() => {
        if (isDark) {
            document.documentElement.classList.add('dark'); // Needed for styling Tailwind elements
            document.documentElement.classList.add('[color-scheme:dark]'); // Needed for styling default browser HTML elements
        } else {
            document.documentElement.classList.remove('dark');
            document.documentElement.classList.remove('[color-scheme:dark]');
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
