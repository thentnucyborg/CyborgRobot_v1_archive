# -*- coding: utf-8 -*-
"""
Created on Sun Oct 18 17:55:48 2015

@author: haavarws
"""


import pybrain
from pybrain.structure import FeedForwardNetwork
from pybrain.structure import RecurrentNetwork
from pybrain.structure import LinearLayer, SigmoidLayer
from pybrain.structure import FullConnection
from pybrain.datasets import SupervisedDataSet, SequentialDataSet, UnsupervisedDataSet

from pybrain.supervised.trainers import BackpropTrainer


from pybrain.tools.shortcuts import buildNetwork


import pickle

import Simulator
import GenerateDataSet

    
    
        

#Init:
#Number of neurons in layers
inputNeurons = 3
hiddenLayers = 3
hiddenLayerNeurons = 30
outputNeurons = 4


networkDim = [inputNeurons]

for i in range(hiddenLayers):
    networkDim.append(hiddenLayerNeurons)
networkDim.append(outputNeurons)


saveNetwork = True # Set to True to save the trained network
loadNetwork = True # Set to True to load a trained network from file
networkFilename = 'DemonstrationNet.dump'


# Generate dataset
approachDataSamples = 150 # number of samples generated for the training dataset
waitCloseDataSamples = 150
waitFarDataSamples  = 150
leaveDataSet = 150

validationTraining = False # If False, the network is trained on the full dataset
trainingTimes = 100


# Enviroment
xMax = 10
xMin = -10
yMax = 10
yMin = 0

robotHeight = 1 # Height of robot "face" from ground.

enviroment = [xMax,xMin,yMax,yMin]



#Preferences
slowSpeed = 0.5
prefSpeed = 1 # Absolute movement per update.
fastSpeed = 2

careDist = 8
goodDist = 2

robotHeight = 1


distPreferences = [careDist,goodDist]
speedPreferences = [slowSpeed,prefSpeed,fastSpeed]

#Tests to be done
tests = dict(lingerClose=0,lingerFar=0, approachLingerLeaveClose = 0,approachLingerLeaveFar= 1,randomMove=0)


#Simulation
updateTime = 0.25 # clock time between face updates

doFaceTest = True 





if loadNetwork:
    
    net = pickle.load(open(networkFilename,'rb'))
    net.sorted = False
    net.sortModules()
    
    
else:
    
    net = buildNetwork(*networkDim,  recurrent=True)
    
    for i in range(hiddenLayers):
        net.addRecurrentConnection(FullConnection(net["hidden" + str(i)],net["hidden"+ str(i)]))
    
    net.sortModules() #Sorts the network so it is ready to be activated
    
    
    
    
    # Creating dataset for training the network
    print("Generating dataset")
    
    
    sequenceDataSet = SequentialDataSet(inputNeurons, outputNeurons)

    sequenceDataSet = GenerateDataSet.approachDataSet(approachDataSamples, enviroment,robotHeight, prefSpeed, distPreferences, sequenceDataSet )
    
    sequenceDataSet = GenerateDataSet.leaveDataSet(leaveDataSet, enviroment,robotHeight, prefSpeed, distPreferences, sequenceDataSet )

    sequenceDataSet = GenerateDataSet.waitCloseDataSet(waitCloseDataSamples,robotHeight, sequenceDataSet)
    
    sequenceDataSet = GenerateDataSet.waitFarDataSet(waitFarDataSamples,distPreferences, robotHeight, sequenceDataSet)




    # Training neural network.
    print("Training network")
    trainer = BackpropTrainer(net,dataset=sequenceDataSet,learningrate=0.01,lrdecay=1, momentum=0,weightdecay=0, verbose = True)
    
    
    if validationTraining:
        trainer.trainUntilConvergence(verbose=True,maxEpochs=100, validationProportion=0.2)
    else:
        for i in range(trainingTimes):
            trainer.train()
            
            if not i %10:
               print("Iteration: " + str(i)+ " out of " + str(trainingTimes))
            
  
    
    print("Training finished")
    

"""
    Trained network is saved here.
"""
if saveNetwork:
    pickle.dump(net,open(networkFilename,'wb'))





#testNetwork(net)


if doFaceTest:
    Simulator.testFace(enviroment,tests, speedPreferences,distPreferences, updateTime, robotHeight, net)


