

This is the Face module.
it uses Python 3.4.3, and requires the Pybrain 3.3 package installed (https://github.com/pybrain/pybrain/archive/0.3.3.
zip). Pybrain requires Scipy and Matplotlib packages. The easiest way to get them 
is probably through a python distribution like Anaconda to set up a python enviroment. 

CreateDemoNetwork is the top module, set the init parameters if you want to create and train new net, load an old one, save the net, etc.
If you change the number of inputs or outputs the training functions will not work anymore and new ones need to be made. Changing number of neurons and layers work fine though.

