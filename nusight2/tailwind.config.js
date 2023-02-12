const defaultTheme = require('tailwindcss/defaultTheme')
const colors = require('tailwindcss/colors')

/** @type {import('tailwindcss').Config} */
module.exports = {
  content: ['./index.html', './src/client/**/*.{js,jsx,ts,tsx}'],
  theme: {
    extend: {
      fontFamily: {
        sans: ['"Open Sans"', ...defaultTheme.fontFamily.sans],
      },
      colors: {
        nusight: {
          500: '#1197d3',
          600: '#0d76a6',
          700: '#004363',
        },
        gray: colors.neutral,
        icon: 'rgba(0, 0, 0, 0.54)',
        divider: 'rgba(0, 0, 0, 0.12)',
      },
    },
  },
  plugins: [],
}
