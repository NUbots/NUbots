import * as CopyWebpackPlugin from 'copy-webpack-plugin'
import * as ExtractTextPlugin from 'extract-text-webpack-plugin'
import * as HtmlWebpackPlugin from 'html-webpack-plugin'
import * as path from 'path'
import * as webpack from 'webpack'

const isProduction = process.argv.indexOf('-p') >= 0
const transpileOnly = process.argv.indexOf('-t') >= 0
const sourcePath = path.join(__dirname, './src')
const outPath = path.join(__dirname, './dist')

const config: webpack.Configuration = {
  mode: isProduction ? 'production' : 'development',
  context: sourcePath,
  devtool: isProduction ? 'source-map' : 'inline-source-map',
  entry: {
    main: [
      './client/main.tsx',
      ...(isProduction ? [] : ['webpack-hot-middleware/client?reload=true']),
    ],
  },
  output: {
    path: outPath,
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
        exclude: [
          path.resolve(__dirname, 'node_modules'),
        ],
        use: [{
          loader: 'awesome-typescript-loader',
          options: {
            useBabel: true,
            transpileOnly,
            babelCore: '@babel/core',
          },
        }],
      },
      // local css
      {
        test: /\.css$/,
        exclude: [
          path.resolve(__dirname, 'node_modules'),
        ],
        use: ExtractTextPlugin.extract({
          fallback: 'style-loader',
          use: [
            {
              loader: 'css-loader',
              query: {
                modules: true,
                sourceMap: !isProduction,
                importLoaders: 1,
                localIdentName: '[local]__[hash:base64:5]',
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
        include: [
          path.resolve(__dirname, 'node_modules'),
        ],
        use: ['style-loader', 'css-loader'],
      },
      {
        test: /\.svg$/,
        use: [
          'babel-loader',
          {
            loader: 'react-svg-loader',
            query: {
              svgo: {
                // svgo options
                plugins: [{ removeTitle: true }],
                floatPrecision: 2,
              },
            },
          },
        ],
      },
      // static assets
      { test: /\.html$/, use: 'html-loader' },
      { test: /\.png$/, use: 'url-loader?limit=10000' },
      { test: /\.jpg$/, use: 'file-loader' },
      { test: /\.(vert|frag)$/, use: 'raw-loader' },
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
  plugins: [
    new CopyWebpackPlugin([{ from: 'assets/images', to: 'images' }]),
    new ExtractTextPlugin({
      filename: 'styles.css',
      disable: !isProduction,
    }),
    new HtmlWebpackPlugin({
      template: 'assets/index.html',
      filename: 'index.html',
      chunks: ['main'],
    }),
    ...(isProduction ? [] : [new webpack.HotModuleReplacementPlugin()]),
  ] as any as webpack.Plugin[],
  node: {
    // workaround for webpack-dev-server issue
    // https://github.com/webpack/webpack-dev-server/issues/60#issuecomment-103411179
    fs: 'empty',
    net: 'empty',
  },
}

export default config

