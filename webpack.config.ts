import * as CopyWebpackPlugin from 'copy-webpack-plugin'
import * as ExtractTextPlugin from 'extract-text-webpack-plugin'
import * as HtmlWebpackPlugin from 'html-webpack-plugin'
import * as path from 'path'
import * as webpack from 'webpack'
import Devtool = webpack.Options.Devtool

const isProduction = process.argv.indexOf('-p') >= 0
const sourcePath = path.join(__dirname, './src')
const outPath = path.join(__dirname, './build')

const devtool: Devtool = isProduction ? 'source-map' : 'inline-source-map'

export default [{
  context: sourcePath,
  devtool,
  entry: {
    main: [
      './client/main.tsx',
    ].concat(isProduction ? [] : [
      'webpack-hot-middleware/client',
    ]),
    vendor: [
      'classnames',
      'mobx',
      'mobx-react',
      'react',
      'react-dom',
      'react-router',
      'react-router-dom',
      'socket.io-client',
      'three',
    ],
  },
  output: {
    path: outPath,
    filename: 'bundle.js',
    publicPath: '/',
  },
  target: 'web' as 'web',
  resolve: {
    extensions: ['.js', '.ts', '.tsx'],
    // Fix webpack's default behavior to not load packages with jsnext:main module
    // (jsnext:main directs not usually distributable es6 format, but es6 sources)
    mainFields: ['module', 'browser', 'main'],
  },
  module: {
    rules: [
      // .ts, .tsx
      {
        test: /\.tsx?$/,
        use: isProduction
          ? 'awesome-typescript-loader?module=es6'
          : [
            'react-hot-loader/webpack',
            'awesome-typescript-loader',
          ],
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
                plugins: (loader: webpack.loader.LoaderContext) => [
                  require('postcss-import')({ root: loader.resourcePath }),
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
          {
            loader: 'babel-loader',
            query: {
              presets: ['env'],
            },
          },
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
      { test: /\.vert$/, use: 'raw-loader' },
      { test: /\.frag$/, use: 'raw-loader' },
    ],
  },
  plugins: [
    new webpack.NoEmitOnErrorsPlugin(),
    new CopyWebpackPlugin([{ from: 'assets/images', to: 'images' }]),
    new webpack.optimize.CommonsChunkPlugin({
      name: 'vendor',
      filename: 'vendor.bundle.js',
      minChunks: Infinity,
    }),
    new webpack.optimize.AggressiveMergingPlugin(),
    new ExtractTextPlugin({
      filename: 'styles.css',
      disable: !isProduction,
    }),
    new HtmlWebpackPlugin({
      template: 'assets/index.html',
    }),
  ].concat(isProduction ? [] : [
    new webpack.HotModuleReplacementPlugin(),
  ]),
  node: {
    // workaround for webpack-dev-server issue
    // https://github.com/webpack/webpack-dev-server/issues/60#issuecomment-103411179
    fs: 'empty',
    net: 'empty',
  },
}]
