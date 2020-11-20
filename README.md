# Model Predictive Control Project
##

### Description
<!--- Modify this section --->
This project's aim is to create predict good recommandation for films. Each user gives appreciations on films with grades that are integers between 0 and 5. One has to predict the remaining grades.




### Getting Started
<!--- Modify this section --->
This version was designed for Matlab 2020a or higher. 

<!---To run the model's calculation, it is only needed to execute the file `run.py`. On the terminal, the command is `python run.py`. The code should return a `results.csv` file with all its predictions, from the test data.--->

### Prerequisites

#### Libraries
<!--- Modify this section --->
The following librairies are used:
* [YALMIP](https://yalmip.github.io/) R20200930, available through the installation of MPT3
* [Casadi](https://web.casadi.org/get/) v3.5.5 for Matlab
* [MPT3](https://www.mpt3.org/Main/Installation): run [install_mpt3.m](https://www.mpt3.org/Main/Installation?action=download&upname=install_mpt3.m) in Matlab
* [Gurobi](https://www.gurobi.com/downloads/): download `Gurobi Optimizer`


#### Code 
<!--- Modify this section --->
To launch the code `run.py` use the following codes and pickle files:
* `helpers.py` : Deal with creation and loading of `.csv` files
* `models/modelNN.py` : Contains methods for the neural network computations
* `models/modelSurprise.py`: Contains surprise methods
* `models/modelBaseline.py`: Contains baseline methods
* `models/modelMatrixFact.py`: Contains matrix factorization methods


The `data` folder is also needed to store training data, the data for the final submission and the test set trained on 0.8 of the training set, which will be used for the ridge regression : `data_train.csv`, `sampleSubmission.csv` and `test_pred.pickle`.

### Additional content
<!--- Modify this section --->
The folder `models` contains python code that established our machine learning procedure,  contains the testing of the different methods implemented. Those files are run into the main code, which is `run.py`

The folder `litterature` contains scientific papers that inspired our project.

### Documentation
<!--- Modify this section --->
* [Class Project 2](https://https://github.com/epfml/ML_course/tree/master/projects/project2/project_recommender_system) : Description of the project.
* [Resources](https://www.https://www.aicrowd.com/challenges/epfl-ml-recommender-system-2019/dataset_files): Datas for the training and testing.

### Authors
* Balestrini Théophile : theophile.balestrini@epfl.ch
* Durand Mathilde : mathilde.durand@epfl.ch
* Hoggett Emma : emma.hoggett@epfl.ch

### Project Status
The project was submitted on the 12 January 2021, as part of the [Model Predictive Control](https://www.epfl.ch/labs/la/page-53049-en-html/teaching-mpc/) course.
