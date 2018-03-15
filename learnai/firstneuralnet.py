# making a 2 layer neural net
# from this video by siraj raval
# https://www.youtube.com/watch?v=vcZub77WvFA&list=PL2-dafEMk2A5BoX3KyKu6ti5_Pytp91sk&index=2
# it's a simple perceptron - a feed-forward neural network
# with lots of annotations because i'm learning!

# for matrix math
import numpy as np

# for timing the training
import time

# the output will be ten 1's and 0's
n_hidden = 10 # number of hidden neurons
n_in = 10 # 10 inputs
n_out = 10 # and 10 outputs
n_sample = 300 # sample data

# hyperparameters
# these 'tuning knobs' are used to lower the loss function
# which is called cross entropy
learning_rate = 0.01 # how fast we want the network to learn
momentum = 0.9 # ???

# seeding it so it's the same random numbers each run
np.random.seed(0)

# now defining the activation function
# this is run every neuron in the network
# turns numbers into probabilities
# each weight is a set of probabilities (which way should it go)
# these probabilities are updated as we train the network, this is
# why it gets better every time. that's what the sigmoid function
# does!
# this happens each time a neuron hits one of the layers - it turns
# that number into a probability. 
def sigmoid(x):
  return 1.0/(1.0 + np.exp(-x))

# using a 2nd activation function, tanh helps because
# it makes the loss even less
def tanh_prime(x):
  return 1-np.tanh(x)**2

# now time to write our training function
# x is input data
# t is our transpose, helping to perform matrix multiplication
# V and W are the two layers to the network
# bv and bw are our biases - making us make a more accurate prediction
# one bias for each of the layers in our network
def train(x, t, V, W, bv, bw):
  
  # forward propogation
  # matrix multiply + biases
  # take the dot product of the input and the first layer,
  # and add in the biases
  A = np.dot(x, V) + bv

  # do the activation function on that delta
  Z = np.tanh(A)

  # using the result from the first operation, compute the second
  # layer using the dot product. and add the bias
  B = np.dot(Z, W) + bw

  # now for the next activation, which we will use sigmoid for
  Y = sigmoid(B)

  # this was the forward propogation. in a feed-forward neural network, 
  # use backward propogation for training to go back and forth.
  # this means the weights are updated one way, then update them backwards.
  # this keeps going while training.

  # backward propogation

  # the transpose is the matrix of weights, but flipped. it's flipped
  # because we're going backwards! this is used to calculate the 
  # backwards propogation.
  Ew = Y - t

  # the Ev value is used to predict the loss. we compare the predicted
  # with the actual loss, in order to minimize it. this is how training
  # happens, by minimizing our loss!
  Ev = tanh_prime(A) * np.dot(W, Ew)

  # predict our loss
  # these are two deltas. we compare it to the Z, which was the first
  # step, and compare the next one with the input
  # TODO: need more clarification on this. what does outer mean?
  # why is Z being used, and not Y?
  # why do we need to know dW, wouldn't we only be concerned about
  # dV since it's comparing the last step with the input?
  dW = np.outer(Z, Ew)
  dV = np.outer(x, Ev)

  # loss function. this is cross entropy!
  # another loss function could be mean squared error.
  # the reason why using cross entropy is because we're doing classification.
  # generally you want to use this function, because it tends to give us
  # a better result.
  loss = -np.mean( t * np.log(Y) + (1 - t) * np.log(1-Y) )

  # finally, we return the loss and the delta and error values
  return loss, (dV, dW, Ev, Ew)


# this is the prediction function
# we are going to do matrix multiplication to predict our value,
# which is the end result
def predict(x, V, W, bv, bw):
  
  # TODO: what exactly is happening here? why is the dot product between
  # the input, and first layer important?
  # why is the activation of this and the second layer important?
  # why wouldn't the training variables be used in this?
  A = np.dot(x, V) + bv
  B = np.dot(np.tanh(A), W) + bw

  # the prediction is going to be a 1 or 0, and it has to be greater
  # than 0.5 to be a 1
  return ((sigmoid(B) > 0.5).astype(int))


# create layers

# we're creating the layer here
# the size is the number of input values, and hidden values
V = np.random.normal(scale=0.1, size=(n_in, n_hidden))
W = np.random.normal(scale=0.1, size=(n_hidden, n_out))

# we haven't used the biases, let's initialise them
bv = np.zeros(n_hidden)
bw = np.zeros(n_out)

# this is so that it will be easier to pass into our training
# function.
params = [V, W, bv, bw]

# generate our data

# this is where the sample of variables comes in to play.
# we're making 300 samples! 
# TODO: what is the difference between normal (above) and
# binomial? assumption: distribution of numbers. just want to make sure
# this isn't refering to something else, like a matrix or something, that
# i'd be less familiar with
X = np.random.binomial(1, 0.5, (n_sample, n_in))
#  TODO: what is this line doing?!
T = X ^ 1

# TRAINING TIME

# training for 100 epochs
for epoch in range(100):
  
  # empty error array, we'll add data to it soon
  err = []

  # this is the update variable
  upd = [0]*len(params)

  # stopwatch!
  t0 = time.clock()

  # for each data point, update the weights of the network
  # neural net is to find the XOR value of 1's and 0's

  # for every value in our input data...
  for i in range(X.shape[0]):

    # ...calculate our loss and gradient. using the training function
    # we just wrote. note: *params lets us just pass all the params
    # (as defined above), instead of passing them all (more typing)
    loss,grad = train(X[i], T[i], *params)

    # now we will update the loss
    # apparently all this means is decrementing all of the parameters
    # with the update value.
    # this makes sense that it updates the biases, because this is
    # what needs to be changed during training!
    # TODO: for updating the layers, what exactly does this mean?
    # is the probabilities of the layers changing? or the values?
    for j in range(len(params)):
      params[j] -= upd[j]

    for j in range(len(params)):
      # remember the hyper parameters? this is where they are used
      # TODO: wondering what the possible numerical range of values
      # would be here. would it be 0 to 2, since we're adding the
      # gradient value and the update value?
      # how does this become normalised?
      upd[j] = learning_rate * grad[j] + momentum * upd[j]

    # now we want to append the loss to our error
    err.append(loss)

  print('Epoch: %d, Loss: %.8f, Time: %.4fs' % (
  epoch, np.mean(err), time.clock()-t0))

# now let's try to predict something
x = np.random.binomial(1, 0.5, n_in)
print('XOR prediction: ')
print(x)
print(predict(x, *params))

# what this shows is the input data and some randomly generated 1's
# and 0's, and then predict the XOR value and get better and better over time!
# TODO: if we were to change the activation function, would the 
# result be different?
# TODO: if there were to be more layers, would the result be different?
# TODO: how do you know how many layers are necessary?


