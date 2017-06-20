module.exports = {
  coverageDirectory: 'coverage',
  coveragePathIgnorePatterns: ['/node_modules/', 'src/global.d.ts'],
  collectCoverageFrom: [
    '**/*.{ts,tsx}',
    '!src/shared/proto/**',
    '!**/node_modules/**',
    '!**/tests/**',
  ],
  globals: {
    __TS_CONFIG__: './tsconfig.test.json'
  },
  mapCoverage: true,
  moduleDirectories: [
    'node_modules',
    '<rootDir>/src'
  ],
  moduleFileExtensions: [
    'js',
    'ts',
    'tsx'
  ],
  moduleNameMapper: {
    '\\.(css)$': 'identity-obj-proxy',
    '\\.(vert)$': '<rootDir>/__mocks__/mock.vert',
    '\\.(frag)$': '<rootDir>/__mocks__/mock.frag'
  },
  roots: [
    '<rootDir>/src'
  ],
  modulePaths: [
    '<rootDir>/src'
  ],
  testMatch: [
    '**/tests/**/*.tests.{ts,tsx}'
  ],
  transform: {
    '.(ts|tsx)': '<rootDir>/node_modules/ts-jest/preprocessor.js'
  }
}

