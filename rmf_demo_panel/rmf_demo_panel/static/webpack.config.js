const webpack = require('webpack');
const resolve = require('path').resolve;

const config = {
    entry:  {
        app: resolve(__dirname, 'src/index.tsx'),
    },
    output: {
      path: resolve(__dirname, './dist'),
      filename: '[name].bundle.js'
    },
    plugins: [
    new webpack.ids.HashedModuleIdsPlugin(), // so that file hashes don't change unexpectedly
    ],
    resolve: {
        extensions: ['.js', '.jsx', '.css', '.tsx', '.ts']
    },
    module: {
        rules: [
            {
                test: /\.(js|jsx)?/,
                exclude: [/node_modules/, /\.(json)?/],
                loader: 'babel-loader'
            },
            { 
                test: /\.tsx?$/,
                loader: 'ts-loader'
            },
            {
                test: /\.css$/i,
                use: ["css-loader", "style-loader"]
            },
            {
                test: /\.json$/,
                use: [
                {
                    loader: "file-loader",
                    options: {
                    esModule: false,
                    },
                },
                ],
                type: "javascript/auto",
            },
        ]},
};

module.exports = config;
