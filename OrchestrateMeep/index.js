var orchestrate = require('orchestrate');

var collection = process.env.ORCHESTRATE_COLLECTION;
var key = process.env.ORCHESTRATE_API_KEY;

var db = orchestrate(key);

var step = 1;

console.log("going to call start()");

function start() {
  console.log("start was called");

  db.post(collection, {
    "name": "WOOOOT",
    "hometown": "MEEEEP",
    "twitter": "@robotgrrl"
  })
  .then(function (result) {
    console.log("success- POST the data");
  })
  .fail(function (err) {
    console.log("fail- did not POST the data");
  })


  db.post(collection, {
    "name": "YA",
    "hometown": "RA",
    "twitter": "BLERG"
  })
  .then(function (result) {
    console.log("success- POST the data");
  })
  .fail(function (err) {
    console.log("fail- did not POST the data");
  })

}


function getIt() {
  console.log("getIt was called");

  db.get(collection, key)
  .then(function (result) {
    console.log(result.body);
    next();
  })
  .fail(function (err) {
    console.log("fail- did not get the data");
  })

}


function updateIt() {
  console.log("updateIt was called");

  db.merge(collection, key, {
    "name": "BLEEEEP"
  })
  .then(function (result) {
    console.log("success- updated the data");
    next();
  })
  .fail(function (err) {
    console.log("fail- did not update the data");
  })


}




function next() {
  switch(step) {
    case 0:
      start();
    break;
    case 1:
      getIt();
    break;
    case 2:
      updateIt();
    break;
    case 3:
      getIt();
    break;
  }
  step++;
}


next();


