import CopyWebpackPlugin from 'copy-webpack-plugin'
import ExtractTextPlugin from 'extract-text-webpack-plugin'
import HtmlWebpackPlugin from 'html-webpack-plugin'
import path from 'path'
import ProgressBarPlugin from 'progress-bar-webpack-plugin'
import webpack from 'webpack'
import nodeExternals from 'webpack-node-externals'

export type ConfigOptions = {
  mode: 'production' | 'development'
  context?: string
  sourceMap?: 'source-map' | 'eval-source-map' | false
  transpileOnly?: boolean
  rootDir?: string
}

export function getClientConfig({
  mode,
  context,
  sourceMap,
  transpileOnly,
  rootDir = __dirname,
}: ConfigOptions): webpack.Configuration {
  const isProduction = mode === 'production'
  return {
    mode: isProduction ? 'production' : 'development',
    devtool: sourceMap,
    context,
    entry: {
      main: './client/main.tsx',
    },
    output: {
      path: path.join(rootDir, 'dist', 'public'),
      filename: '[name].js',
      publicPath: '/',
      globalObject: 'this',
    },
    target: 'web',
    resolve: {
      extensions: ['.js', '.ts', '.tsx'],
      // Fix webpack's default behavior to not load packages with jsnext:main module
      // (jsnext:main directs not usually distributable es6 format, but es6 sources)
      mainFields: ['module', 'browser', 'main'],
    },
    module: {
      rules: [
        // webworkers
        { test: /\.worker\.ts$/, use: 'worker-loader' },
        // .ts, .tsx
        {
          test: /\.tsx?$/,
          exclude: [path.resolve(rootDir, 'node_modules')],
          use: {
            loader: 'ts-loader',
            options: {
              onlyCompileBundledFiles: true,
              transpileOnly,
            },
          },
        },
        // local css
        {
          test: /\.css$/,
          exclude: [path.resolve(rootDir, 'node_modules')],
          use: ExtractTextPlugin.extract({
            fallback: 'style-loader',
            use: [
              {
                loader: 'css-loader',
                options: {
                  modules: {
                    localIdentName: '[local]__[hash:base64:5]',
                  },
                  sourceMap: !isProduction,
                  importLoaders: 1,
                },
              },
              {
                loader: 'postcss-loader',
                options: {
                  ident: 'postcss',
                  plugins: [
                    require('postcss-import')({ addDependencyTo: webpack }),
                    require('postcss-url')(),
                    require('postcss-cssnext')(),
                    require('postcss-reporter')(),
                    require('postcss-browser-reporter')({ disabled: isProduction }),
                  ],
                },
              },
            ],
          }),
        },
        /*
        External libraries generally do not support css modules so the selector mangling will break external components.
        This separate simplified loader is used for anything within the node_modules folder instead.
        */
        {
          test: /\.css$/,
          include: [path.resolve(rootDir, 'node_modules')],
          use: ['style-loader', 'css-loader'],
        },
        { test: /\.file.svg$/, use: 'url-loader' },
        {
          test: /\.svg$/,
          exclude: /\.file.svg$/,
          use: {
            loader: 'react-svg-loader',
            options: {
              svgo: {
                // svgo options
                plugins: [{ removeTitle: true }, { removeViewBox: false }],
                floatPrecision: 2,
              },
            },
          },
        },
        // static assets
        { test: /\.html$/, use: 'html-loader' },
        { test: /\.png$/, use: 'url-loader?limit=10000' },
        { test: /\.(jpg|glb|bin)$/, use: 'file-loader' },
        { test: /\.(vert|frag)$/, use: path.resolve('build_scripts/glsl_loader.js') },
      ],
    },
    optimization: {
      splitChunks: {
        name: true,
        cacheGroups: {
          vendor: {
            test: /[\\/]node_modules[\\/]/,
            chunks: 'all',
            priority: -10,
          },
          proto: {
            test: /src[\\/]shared[\\/]proto/,
            chunks: 'all',
            priority: -10,
          },
        },
      },
    },
    plugins: ([
      new CopyWebpackPlugin({ patterns: [{ from: 'assets', context }] }),
      new ExtractTextPlugin({
        filename: 'styles.css',
        disable: !isProduction,
      }),
      new HtmlWebpackPlugin({
        template: 'assets/index.html',
        filename: 'index.html',
        chunks: ['main'],
      }),
      new ProgressBarPlugin(),
    ] as any) as webpack.Plugin[],
    node: {
      // workaround for webpack-dev-server issue
      // https://github.com/webpack/webpack-dev-server/issues/60#issuecomment-103411179
      fs: 'empty',
      net: 'empty',
    },
  }
}

export function getServerConfig({
  mode,
  context,
  sourceMap,
  transpileOnly,
  rootDir = __dirname,
}: ConfigOptions): webpack.Configuration {
  return {
    mode,
    devtool: sourceMap,
    context,
    entry: {
      ...(mode === 'production' ? { prod: './server/prod.ts' } : {}),
      ...(mode === 'development' ? { dev: './server/dev.ts' } : {}),
    },
    output: {
      path: path.join(rootDir, 'dist'),
      filename: '[name].js',
      globalObject: 'this',
    },
    target: 'node',
    resolve: {
      extensions: ['.js', '.ts'],
    },
    externals: [nodeExternals()],
    module: {
      rules: [
        {
          test: /\.ts$/,
          exclude: [path.resolve(rootDir, 'node_modules')],
          use: {
            loader: 'ts-loader',
            options: {
              onlyCompileBundledFiles: true,
              transpileOnly,
            },
          },
        },
        {
          test: /\.(jpg|ico)$/,
          use: {
            loader: 'file-loader',
            options: {
              outputPath: 'assets',
              publicPath: path.join('dist', 'assets'),
            },
          },
        },
      ],
    },
    optimization: {
      minimize: false,
    },
    plugins: [new ProgressBarPlugin()],
    node: {
      __dirname: false,
      __filename: false,
    },
  }
}
