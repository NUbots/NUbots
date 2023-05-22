/* eslint-env node */
module.exports = {
  coverageDirectory: "coverage",
  coveragePathIgnorePatterns: ["/node_modules/"],
  collectCoverageFrom: [
    "**/*.{ts,tsx}",
    "!src/shared/messages.js",
    "!src/shared/messages.d.ts",
    "!**/node_modules/**",
    "!**/tests/**",
  ],
  globals: {
    "ts-jest": {
      tsConfig: "./tsconfig.test.json",
    },
  },
  moduleDirectories: ["node_modules", "<rootDir>/src"],
  transformIgnorePatterns: ["/node_modules/(?!mobx-utils/lib)"],
  moduleFileExtensions: ["js", "ts", "tsx"],
  moduleNameMapper: {
    "\\.(css)$": "identity-obj-proxy",
    "\\.(vert)$": "<rootDir>/__mocks__/mock.vert",
    "\\.(frag)$": "<rootDir>/__mocks__/mock.frag",
  },
  roots: ["<rootDir>/src"],
  modulePaths: ["<rootDir>/src"],
  testMatch: ["**/tests/**/*.tests.{ts,tsx}"],
  preset: "ts-jest/presets/js-with-ts",
  resolver: "<rootDir>/jest.resolver.js",
};
