# OpenTDCRContactModel

## Intro
# TODO 

## Install
Clone the repo
```
git clone https://github.com/ContinuumRoboticsLab/OpenTDCRContactModel.git
cd OpenTDCRContactModel
```
Make sure matlab is installed prior to this step and that you have checked the "Optimization Toolbox" package to be installed with Matlab

Create a conda enviroment. This must be done with sudo as some of the python files
require it.
``` 
mkdir envs
sudo $(which conda) env create -f environment.yml --prefix ./envs
```
Activate enviroment
```
conda activate ./envs
```


## Install system packages
```
sudo apt update
```
```
sudo apt install cmake libopenblas-dev build-essential libnlopt-dev
```

nlopt must be built from source. Might have to run this code block twice if you encounter an error.
```
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

cd ../../
sudo rm -rf nlopt
```

## Build the pcca_solver file
```
cd utils_model/cpp
mkdir cmake-build-release
cmake -DCMAKE_BUILD_TYPE=Release -G 'CodeBlocks - Unix Makefiles' -B ./cmake-build-release
cd cmake-build-release
make pcca_solver
cd ../../../
```

## Usage

1. Import the built shared object (from parent of cpp folder):
```python
import importlib
pcca_solver_module = importlib.import_module("cpp.cmake-build-release.pcca_solver")
```
2. Instantiate the solver:
```
cpp_solver = pcca_solver_module.pcca_solver()
```
2. Use the solver with numpy objects:
```python
xSol,exitflag = cpp_solver.solve(...)
```