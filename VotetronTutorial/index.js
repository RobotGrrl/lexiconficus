var express = require('express'),
    passport = require('passport'),
    GitHubStrategy = require('passport-github').Strategy;

var app = express();

passport.serializeUser(function(user, done) {
    done(null, user);
});

passport.deserializeUser(function(obj, done) {
    done(null, obj);
});

passport.use(new GitHubStrategy(
    {
        clientID: process.env.GITHUB_CLIENT_ID,
        clientSecret: process.env.GITHUB_CLIENT_SECRET,
        callbackURL: process.env.GITHUB_CALLBACK
    },
    function(accessToken, refreshToken, profile, done) {
        // on successful auth
    }
));

// configure Express
app.use(logger());
app.use(cookieParser());
app.use(bodyParser());
app.use(methodOverride());
app.use(session({ secret: 'cookie monster' }));
app.use(passport.initialize());
app.use(passport.session());
app.use(app.router);

app.get('/', function(req, res){
    res.send('hello world');
});

app.get('/auth/github', passport.authenticate('github'));

app.get('/auth/github/callback', passport.authenticate('github', { failureRedirect: '/' }), function(req, res) {
    // Successful authentication, redirect home.
    res.redirect('/');
});

app.listen(process.env.PORT || 5000); 
