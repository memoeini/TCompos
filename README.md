## Description
TCompos is a tool for compositional verification of timed automata. The tool, designed for timed safety properties, introduces a novel compositional framework that alleviates the limitations of the existing frameworks by leveraging both forward and backward analysis techniques, paving the way for more efficient verification of systems. The tool builds upon the tool TChecker by incorporating the new algorithm alongside the existing reachability algorithms, enabaling comparisons between them.

## Installation
The requirements are the same as those specified for TChecker and are outlined below for reference:

- a C++ compiler with C++17 support (Clang >= 3.6, GNU g++ >= 6. LLVM >= 10.0.0)
- CMake (>= 2.8.12)
- flex (>= 2.5.35)
- bison (>= 3.0.4)
- The Boost library (>= 1.81.0 -- boost::json is required)
- Doxygen (>= 1.8.15)
- Catch2 (>= 3.0.0)

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
     cmake ../tchecker -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install
     ```

   * **For macOS:**

     ```
     cmake ../tchecker -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/path/to/install -DCMAKE_PREFIX_PATH=/path/to/bison
     ```

4. **Compile and Install**

   ```
   make -j
   make doc
   make install
   ```

For additional information on the installation, please refer to the official [TChecker GitHub repository](https://github.com/ticktac-project/tchecker).

## USAGE
To analyze your system using TCompos, follow the steps below to decompose it and run the verification algorithm.

1. **Decompose Your System**

Use the `system-generator/system.py` script to split your system into two parts:

* **Open system** (includes the property automaton)
* **Environment**

The input file must follow the [TChecker input language format](https://github.com/ticktac-project/tchecker/wiki/TChecker-file-format).

When modeling properties, treat them as **safety properties** using a special **error state** labeled `Pi`. The property automaton should be included in the **open system** file.

2. **Generated Output Files**

Running the system generator on an input file `x` will produce the following:

* `x_orig`: The original system with some modifications, used internally for reference
* `x_prop`: The open system (with property automaton)
* `x_env`: The environment

3. **Run the Algorithm**

Use the following command to verify the system:

```bash
/path/to/x_orig -P /path/to/x_prop -E /path/to/x_env -a [algo] -l Pi
```

Replace `[algo]` with one of the following:

* `reach`: for the standard reachability algorithm
* `compos`: for the compositional verification approach

You may optionally add `-i` to enable **iterative execution**:

```bash
/path/to/x_orig -P /path/to/x_prop -E /path/to/x_env -a compos -l Pi -i
```
## License
MIT License

Copyright (c) 2024 Mehran Moeini Jam and Hamed Kalantari  

Copyright (c) 2019 Bordeaux INP, CNRS

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
