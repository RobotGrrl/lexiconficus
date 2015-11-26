var express = require('express'),
    passport = require('passport'),
    GitHubStrategy = require('passport-github').Strategy;

var app = express();

// configure Express
app.use(express.logger());
app.use(express.cookieParser());
app.use(express.bodyParser());
app.use(express.methodOverride());
app.use(express.session({ secret: 'cookie monster' }));
app.use(passport.initialize());
app.use(passport.session());
app.use(app.router);

app.get('/', function(req, res){
    res.send('hello world');
});

app.listen(process.env.PORT || 5000); 
