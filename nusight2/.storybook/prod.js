/* eslint-env node */
const express = require("express");
const http = require("http");
const path = require("path");

const app = express();
const server = http.createServer(app);
const root = path.resolve(__dirname, "../dist/storybook");
app.use(express.static(root));
const port = process.env.PORT || 9002;

server.listen(port, () => {
  console.log(`Storybook server started at http://localhost:${port}`);
});
