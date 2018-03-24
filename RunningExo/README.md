# Running Exoskeleton Sub-Project

Currently developed by [Albert Wu](wualbert@mit.edu) and [Sinba Yang](xingbang@mit.edu)

## Usage of Variables in the GUI
### General Varaiable(genVar)
* **genVar[0]**  
   State in `fsm1()`. 1 is stance phase, 2 is swing phase
* **genVar[1]**  
   Control action commanded by FlexSEA. Arbitrary unit
* **genVar[2]**  
   Number of `fsm1()` executions. Increases until overflow happens then wraps around. Used for timekeeping. Unit is 
   approximately 1 ms
* **genVar[3]**  
   Stance phase duration. Unit is approximately 1 ms
* **genVar[4]**  

* **genVar[5]**  

* **genVar[6]**  

* **genVar[7]**  

* **genVar[8]**  

* **genVar[9]**  
