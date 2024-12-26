# drone_processing

This project controls a drone's motor and processes images using computer vision algorithms. It runs on a system that supports WSL (Windows Subsystem for Linux) and uses `g++` to compile C++ code while leveraging Python for image processing with OpenCV.

## Setup Instructions (test run on WSL)

### 1. **Install WSL (Windows Subsystem for Linux)**

If you haven't already installed WSL, follow these steps:

1. Open PowerShell as Administrator and run:
    ```bash
    wsl --install
    ```
2. Restart your machine after installation.
3. Set up a Linux distribution, such as Ubuntu, from the Microsoft Store.

### 2. **Install `g++` Compiler on WSL**

You will need the `g++` compiler to compile the C++ code. Follow these steps:

1. Open the WSL terminal (e.g., Ubuntu).
2. Update package lists and install `g++`:
    ```bash
    sudo apt update
    sudo apt install g++
    ```

### 3. **Install Python Dependencies**

The project uses Python for image processing, and OpenCV is required. To install it:

1. Ensure `pip3` is installed. If not, install it by running:
    ```bash
    sudo apt install python3-pip
    ```
2. Install OpenCV:
    ```bash
    pip3 install opencv-python
    ```

### 4. **Clone the Repository**

Clone the repository to your local machine:

```bash
git clone https://github.com/yourusername/drone_processing.git
cd drone_processing

```

```cpp
int main(int argc, char* argv[]) {
    cout << "Test commit" << endl;
    return EXIT_SUCCESS;
}

