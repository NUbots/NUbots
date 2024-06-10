import React, { useEffect, useMemo, useState } from "react";
import { IconButton, IconButtonProps } from "../icon_button/view";

export default function ThemeSwitcherButton(props: IconButtonProps) {
    const [isDark, setIsDark] = useState(window.matchMedia && window.matchMedia("(prefers-color-scheme: dark)").matches);
    const mediaQuery = useMemo(() => window.matchMedia("(prefers-color-scheme: dark)"), []);

    useEffect(() => {
        const changeHandler = () => setIsDark(mediaQuery.matches);

        mediaQuery.addEventListener("change", changeHandler);

        return () => {
            mediaQuery.removeEventListener("change", changeHandler);
        };
    }, [mediaQuery]);

    useEffect(() => {
        const themeColorMetaTag = document.querySelector('meta[name="color-scheme"]');

        if (isDark) {
            document.documentElement.classList.add("dark"); // Needed for styling Tailwind elements
            if (themeColorMetaTag) {
                themeColorMetaTag.setAttribute('content', 'dark'); // Needed for default browser styling
            }
        } else {
            document.documentElement.classList.remove("dark");
            if (themeColorMetaTag) {
                themeColorMetaTag.setAttribute('content', 'light');
            }
        }
    }, [isDark]);

    const handleClick = () => {
        setIsDark(!isDark);
    };

    return (
        <IconButton {...props} onClick={handleClick}>
            {isDark ? "dark_mode" : "light_mode"}
        </IconButton>
    );
}
