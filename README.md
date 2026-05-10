# TCompos: Checking correctness of real-time system models through compositional analysis

## Description
TCompos is a tool for verifying the correctness of timed systems modeled as networks of timed automata. Designed for timed safety properties, the tool introduces a new compositional framework based on assume-guarantee reasoning. 

TCompos builds upon the tool [TChecker](https://github.com/ticktac-project/tchecker) and leverages both forward and backward analysis techniques to analyze the system. The approach requires a decomposition of the network into an open-system component and an environment component. This modular view allows it to reason about each part (almost) in isolation before analyzing their combined behavior together. This enables considerable reductions in the size of the state space, allowing the algorithm not only to establish system correctness but also to identify property violations in faulty system models early in the analysis.

## Installation

**Software Dependencies**

> [!NOTE]
> To avoid manual dependency installation, you can use the provided Docker image containing all required dependencies.
   
The requirements are the same as those specified for [TChecker](https://github.com/ticktac-project/tchecker/wiki/Installation-of-TChecker). For convenience, we provide installation commands for Debian-based Linux distributions such as Ubuntu. Users of other operating systems or distributions may need to adapt accordingly.

The command below will install the majority of the required packages:
   ```
   sudo apt-get update && sudo apt-get install -y build-essential gcc g++ git cmake bison flex doxygen graphviz python3 libboost-all-dev
   ```
> [!WARNING]
> Boost version `1.81.0` or newer is required. Since some repositories may provide an older `libboost-all-dev` release, you may need to install Boost manually from the [official Boost repository](https://github.com/boostorg/boost).

The last required dependency is Catch2 version `3.0.0` or newer. The following commands install a tested version of the software.

   ```
   git clone --branch v3.4.0 https://github.com/catchorg/Catch2.git && \
   cd Catch2 && \
   cmake -B build && \
   cmake --build build -j$(nproc) && \
   sudo cmake --install build
   ```

**Installation**

To build and install, follow the steps below:

1. **Clone the Repository**

   ```
   git clone https://github.com/memoeini/TCompos.git
   ```

2. **Create and Enter the Build Directory**

   ```
   mkdir build
   cd build
   ```

3. **Configure the Build**

    * **For Linux:**

      ```
      cmake ../TCompos -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
      ```

    * **For macOS:**

      ```
      cmake ../TCompos -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install -DCMAKE_PREFIX_PATH=/path/to/bison
      ```

4. **Compile, Generate Documentation, and Install**

   ```
   make -j
   make doc
   make install
   ```

Further details on the installation are available on the official [TChecker GitHub repository](https://github.com/ticktac-project/tchecker).

## USAGE
To analyze your system using TCompos, follow the steps below to decompose it and run the verification algorithm.

1. **Decompose Your System**

You need to use the `system_decomposer.py` script to split your system into an **Open system** (includes the error property automaton) and an **Environment**, using the following command:

```
python system_decomposer.py \
  --model <model_file> \
  --os <open_system_component_1> <open_system_component_2> ... \
  --env <environment_component_1> <environment_component_2> ...
```

The input file must follow the [TChecker input language format](https://github.com/ticktac-project/tchecker/wiki/TChecker-file-format).

Properties should be modeled as **safety properties** using a special **error state** labeled `Pi`. This error property automaton should be included in the **open system** component, along with any automaton that has an invariant.

**Generated Output Files**

Running the system generator on an input model named `x` will produce the following output:

* `x_orig`: The original system with some modifications to names, used internally for reference
* `x_prop`: The open system (including the error property automaton)
* `x_env`: The environment

2. **Run the Algorithm**

Use the following command to verify the system:

```bash
tck-reach /path/to/x_orig -P /path/to/x_prop -E /path/to/x_env -a [algo] -l Pi
```

Replace `[algo]` with one of the following:

* `reach`: for the standard reachability algorithm
* `compos`: for the compositional verification approach

Both algorithms are configured to use the `EXTRA_LU_PLUS_GLOBAL` extrapolation with global-clock semantics, in accordance with the compositional algorithmic framework.

## License
MIT License

Copyright (c) 2019 Bordeaux INP, CNRS

Copyright (c) 2026 Mehran Moeini Jam, Hamed Kalantari

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
