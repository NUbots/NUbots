/* eslint-env node */
import { Config } from 'jest';

const config: Config = {
  coverageDirectory: 'coverage',
  coveragePathIgnorePatterns: [
    '/node_modules/',
  ],
  collectCoverageFrom: [
    '**/*.{ts,tsx}',
    '!src/shared/messages.js',
    '!src/shared/messages.d.ts',
    '!**/node_modules/**',
    '!**/tests/**',
  ],
  moduleDirectories: [
    'node_modules',
    '<rootDir>/src',
  ],
  transformIgnorePatterns: [
    '/node_modules/(?!mobx-utils/lib)',
  ],
  moduleFileExtensions: [
    'js',
    'ts',
    'tsx',
  ],
  moduleNameMapper: {
    '\\.(css)$': 'identity-obj-proxy',
    '\\.(vert)$': '<rootDir>/__mocks__/mock.vert',
    '\\.(frag)$': '<rootDir>/__mocks__/mock.frag',
  },
  roots: [
    '<rootDir>/src',
  ],
  modulePaths: [
    '<rootDir>/src',
  ],
  testMatch: [
    '**/tests/**/*.tests.{ts,tsx}',
  ],
  preset: 'ts-jest/presets/js-with-ts',
  resolver: "<rootDir>/jest.resolver.cjs",
}

export default config;
