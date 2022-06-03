const udp = require("dgram");

const server = udp.createSocket("udp4");

server.on("error", function (error) {
  console.log("Server error:", error);
  server.close();
});

server.on("message", function (message, info) {
  console.log(
    `Received ${message.length} bytes from ${info.address}:${info.port}`
  );

  try {
    const parsed = JSON.parse(message);
    console.log("message:");
    console.dir(parsed);
  } catch (err) {
    console.error("Could not parse message as JSON", err);
  }
});

server.on("listening", function () {
  const address = server.address();
  const port = address.port;
  const family = address.family;
  const ipaddr = address.address;
  console.log(`UDP server is listening at ${family} address ${ipaddr}:${port}`);
});

server.on("close", function () {
  console.log("Socket closed!");
  process.exit();
});

server.bind(9870);
