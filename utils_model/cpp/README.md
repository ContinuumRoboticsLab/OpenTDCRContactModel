# Usage Guide

## Prerequisites

### Build Tools & CMake

Mainly for the g++ compiler (along with other build tools):
```bash
sudo apt install build-essential
```
For the latest version of CMake we need to add an apt repository (I follow the instructions [here](https://askubuntu.com/a/865294)). In case you have an old version:
```bash
sudo apt remove --purge --auto-remove cmake
```
Prepare for installation:
```bash
sudo apt install -y software-properties-common lsb-release && \
sudo apt clean all
```

Obtain a copy of kitware's signing key:
```bash
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
```

Add the repository:
```bash
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt update
```

Install:
```bash
sudo apt install cmake
```


### BLAS/LAPACK 
a requirement by several libraries. You can use any implementation, but it does affect performance. I mainly used IntelMKL, you can follow installation guide [here](https://www.intel.com/content/www/us/en/developer/tools/oneapi/onemkl-download.html?operatingsystem=linux&distributions=aptpackagemanager).  
For debian based distributions:
```bash
wget -O- https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB \
| gpg --dearmor | sudo tee /usr/share/keyrings/oneapi-archive-keyring.gpg > /dev/null
echo "deb [signed-by=/usr/share/keyrings/oneapi-archive-keyring.gpg] https://apt.repos.intel.com/oneapi all main" | sudo tee /etc/apt/sources.list.d/oneAPI.list
sudo apt update
sudo apt install intel-oneapi-mkl-devel
```  
Or you can easily install OpenBLAS instead:
```bash
sudo apt install libopenblas-dev
```

### Python-dev, NumPy & Pybind

I used python 3.10, I remember having trouble with other versions for some reason (maybe unrelated, like matlab engine). Do:

```bash
sudo apt install python3.10-dev
sudo pip install numpy pybind11[global]
```

### Xtensor 
The "numpy equivalent" used in this project. **Make sure you have BLAS & python/pybind installed before installing xtensor**. Several installations are in order - "xtl", "xtensor", "xtensor-blas", "xtensor-python" and "xsimd". Something like this should work:
```bash
mkdir xtensor
cd xtensor

git clone https://github.com/xtensor-stack/xtl.git
cd xtl
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../../

git clone https://github.com/xtensor-stack/xtensor.git
cd xtensor
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../../

git clone https://github.com/xtensor-stack/xtensor-blas.git
cd xtensor-blas
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../../

git clone https://github.com/xtensor-stack/xtensor-python.git
cd xtensor-python
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

git clone https://github.com/xtensor-stack/xsimd.git
cd xsimd
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=/usr/local ..
sudo make install

cd ../../../

rm -rf xtensor
```

### NLopt

You can build & install the latest version from source:
```bash
git clone https://github.com/stevengj/nlopt.git
cd nlopt
mkdir build
cd build
cmake ..
make
sudo make install

cd ../../
```

Or just install with apt:
```bash
sudo apt install libnlopt-dev
```
## Build

1. Choose BLAS vendor by setting the BLA_VENDOR variable in [CMakeLists.txt](CMakeLists.txt) ("OpenBLAS" or "Intel10_64lp"). You only need to uncomment the relavant line.
2. **IntelMKL users only**: If not done so for current shell, initialize MKL:
```bash
. /opt/intel/oneapi/setvars.sh
. /opt/intel/oneapi/compiler/latest/env/vars.sh
```
3. Run (with the "cpp" folder containing the cpp source as your working directory):  
```bash
mkdir cmake-build-release
cmake -DCMAKE_BUILD_TYPE=Release -G 'CodeBlocks - Unix Makefiles' -B ./cmake-build-release
cd cmake-build-release
make pcca_solver
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