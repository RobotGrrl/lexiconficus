var orchestrate = require('orchestrate');

//var collection = process.env.ORCHESTRATE_COLLECTION;
var collection = 'users';
var key = process.env.ORCHESTRATE_API_KEY;

var db = orchestrate(key);

var step = -1;
var user_count = 0;


function start() {

  db.ping()
  .then(function () {
    console.log('connection is all good');
    next();
  })
  .fail(function (err) {
    console.log('FAIL - connection does not work');
    console.log(err);
  })
  
}


function newUserA() {
  console.log("newUserA");

  db.put(collection, ('user-'+user_count), {
    "username": "frankenmeep",
    "name": "Franken Meepen Stein",
    "hometown": "Zombieland",
    "pet": "Poodle"
  })
  .then(function (res) {
    console.log('done:' + res.statusCode);
    user_count++;
    next();
  })

}


function newUserB() {
  console.log("newUserB");

  db.put(collection, ('user-'+user_count), {
    "username": "hikingboots",
    "name": "Squished Catepillar",
    "hometown": "Forest",
    "pet": "Flea"
  })
  .then(function (res) {
    console.log('done:' + res.statusCode);
    user_count++;
    next();
  })

}


function getUserA() {
  console.log('getUserA');

  db.get(collection, 'user-0')
  .then(function (res) {
    console.log(res.body);
    next();
  })
  .fail(function (err) {});

}


function getUserB() {
  console.log('getUserB');

  db.get(collection, 'user-1')
  .then(function (res) {
    console.log(res.body);
    next();
  })
  .fail(function (err) {});

}


function updateUserA() {
  console.log('updateUserA');

  db.newPatchBuilder(collection, 'user-0')
    .add('favorite_food', "Brains")
    .replace('pet', "Dragon")
    .apply()
  .then(function (res) {
    console.log('done: ' + res.statusCode);
    next();
  })
  .fail(function (err) {
    console.log('update A error');
    console.log(err);
  });

}


function updateUserB() {
  console.log('updateUserB');

  db.newPatchBuilder(collection, 'user-1')
    .add('favorite_food', "Meow Mix")
    .replace('pet', "Rock")
    .apply()
  .then(function (res) {
    console.log('done: ' + res.statusCode);
    next();
  })
  .fail(function (err) {
    console.log('update B error');
    console.log(err);
  });

}


function next() {
  step++;
  switch(step) {
    case 0:
      start();
    break;
    case 1:
      newUserA();
    break;
    case 2:
      newUserB();
    break;
    case 3:
      getUserA();
    break;
    case 4:
      getUserB();
    break;
    case 5:
      updateUserA();
    break;
    case 6:
      updateUserB();
    break;
    case 7:
      getUserA();
    break;
    case 8:
      getUserB();
    break;
    case 9:
      console.log('c\'est fini! w00t!');
    break;
  }
}


next();


