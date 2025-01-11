# Orbit Navigator

<p align="center">
    <img src="https://img.shields.io/github/v/release/OrbitNavigator/OrbitNavigator" alt="GitHub Release">
    <img src="https://img.shields.io/github/stars/OrbitNavigator/OrbitNavigator?style=flat" alt="GitHub Repo stars">
    <img alt="GitHub Issues or Pull Requests" src="https://img.shields.io/github/issues/OrbitNavigator/OrbitNavigator">
    <img alt="GitHub forks" src="https://img.shields.io/github/forks/OrbitNavigator/OrbitNavigator?style=flat">
    <img src="https://img.shields.io/github/license/OrbitNavigator/OrbitNavigator" alt="GitHub License">
</p>

## Development Guide

### Prerequisites

- **STM32CubeCLT**: Ensure you have STM32CubeCLT installed.
- **CMake**: Make sure CMake is installed on your system.

### Building the Project

1. **Clone the Repository**:

   ```sh
   git clone https://github.com/OrbitNavigator/OrbitNavigator.git
   cd OrbitNavigator
   ```

2. **Generate Build Files**:

   ```sh
   cmake -S . -B build
   ```

3. **Build the Project**:
   ```sh
   cmake --build build
   ```

### Flashing the Firmware

1. Connect your STM32Gxx device to your development machine.
2. Use STM32CubeCLT to flash the firmware:
   ```sh
   stmcubeclt -c port=SWD -d build/firmware.bin
   ```

### Directory Structure

```plaintext
OrbitNavigator/
├── src/
│   ├── main/
│   ├── linker/
│   └── startup/
└── lib/
    ├── CMSIS/
    ├── mavlink/
    └── stm32g4xx/
```

- **src**: Contains the source code for the project.

  - **main**: Includes the main application code.
  - **linker**: Contains linker scripts used for memory layout.
  - **startup**: Holds startup code and initialization routines.

- **lib**: Contains external libraries and dependencies.
  - **CMSIS**: ARM Cortex Microcontroller Software Interface Standard library.
  - **mavlink**: MAVLink communication protocol library.
  - **STM32G4xx**: STM32G4xx series specific peripheral libraries.

Each directory and subdirectory is organized to separate different aspects of the project, making it easier to manage and navigate the codebase.

### License

This project is licensed under the GPLv3 License. See the [LICENSE](LICENSE) file for more details.
