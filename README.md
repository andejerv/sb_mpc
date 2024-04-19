# SB_MPC
This is a C++ implementation of a simulation-based model predictive controller for object avoidance in autonomous vessels, based on the repository by ingerbha https://github.com/ingerbha/sb_mpc. It is primarily developed for use on the BlueBoat from Blue Robotics, https://bluerobotics.com/store/boat/blueboat/blueboat/. There is however small modifications necessary to adopt it for other vessels, this is further described in the usage section.

## Table Of Contents
- [Installation](#installation)
- [Usage](#usage)

## Installation
To install the SB_MPC library, follow these steps:
Please note that the above instructions assume that you have the necessary dependencies installed and configured correctly.

1. Clone the repository:
```bash
git clone https://github.com/your-username/SB_MPC.git
```

2. Navigate to the project build directory:
```bash
cd SB_MPC/build
```

3. Build the library:
```bash
make
```

4. Include the necessary header files in your C++ code:
```cpp
#include "SB_MPC.h"
```

5. Create an instance of the Agent class and implement the dxdt function and state initialization in your own class:
```cpp
class MyAgent : public Agent {
public:
    // Implement the dxdt function
    void dxdt(const Eigen::VectorXd& x, const Eigen::VectorXd& u, Eigen::VectorXd& dx) override {
        // Your implementation here
    }

    // Implement the state initialization
    void initializeState(Eigen::VectorXd& x) override {
        // Your implementation here
    }
};
```

Use the library in your code as needed.

Remember to link against the SB_MPC library when compiling your code.

## Usage