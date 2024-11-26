/* eslint-env node */
import express from "express";
import * as http from "http";
import * as path from "path";

const app = express();
const server = http.createServer(app);
const root = path.resolve(import.meta.dirname, "../dist/storybook");
app.use(express.static(root));
const port = process.env.PORT || 9002;

server.listen(port, () => {
  console.log(`Storybook server started at http://localhost:${port}`);
});
