var orchestrate = require('orchestrate');

var db = orchestrate(process.env.ORCHESTRATE_API_KEY);

console.log("going to call start()");

function start() {
  console.log("start was called");
}

start();
