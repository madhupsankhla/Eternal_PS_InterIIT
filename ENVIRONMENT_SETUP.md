# Environment Setup Guide

## Overview
This guide explains how to recreate the `eternal` conda environment on another machine.

**Target Platform:** ARM64 (aarch64) - Raspberry Pi 4  
**Python Version:** 3.10.19  
**Environment Name:** eternal

## Prerequisites
- **Hardware:** Raspberry Pi 4 (ARM64/aarch64 architecture)
- **OS:** Raspberry Pi OS 64-bit or compatible ARM64 Linux distribution
- **Miniconda or Anaconda:** Installed for ARM64 architecture
  - Download ARM64 version from: https://docs.conda.io/en/latest/miniconda.html
  - Look for "Linux aarch64" installer
- **Python:** 3.10.x (will be installed via conda)

## Method 1: Using environment.yml (Recommended)

This method recreates the exact environment with all dependencies:

```bash
# Navigate to the project directory
cd /path/to/Eternal_PS_InterIIT

# Create the environment from the YAML file
conda env create -f environment.yml

# Activate the environment
conda activate eternal
```

## Method 2: Using requirements.txt

If you prefer to use pip only:

```bash
# Create a new conda environment with Python 3.10
conda create -n eternal python=3.10

# Activate the environment
conda activate eternal

# Install packages from requirements.txt
pip install -r requirements.txt
```

## Additional Hardware-Specific Packages

Some packages are hardware-specific (e.g., for Raspberry Pi and Arduino) and may need to be installed separately:

### For Raspberry Pi GPIO Control:
```bash
pip install RPi.GPIO pigpio
```

### For Serial Communication (Arduino):
```bash
pip install pyserial
```

### For Keyboard Control:
```bash
pip install pynput
```

## Verification

Verify the installation:

```bash
# Check Python version
python --version
# Should output: Python 3.10.19

# Check installed packages
conda list

# Or for pip packages only
pip list
```

## Package Summary

### Core Dependencies:
- **Python**: 3.10.19
- **NumPy**: 2.2.5 (numerical computing)
- **SciPy**: 1.15.2 (scientific computing)
- **Matplotlib**: 3.10.1 (plotting)
- **OpenCV**: 4.12.0.88 (computer vision)
- **PyZbar**: 0.1.9 (QR code scanning)

### Development Tools:
- **IPython/Jupyter**: For interactive development
- **IPykernel**: 6.29.5 (Jupyter kernel)

### Hardware Interface (Optional):
- **PySerial**: Serial communication with Arduino
- **RPi.GPIO**: Raspberry Pi GPIO control
- **pigpio**: Alternative GPIO library for Raspberry Pi
- **pynput**: Keyboard input handling

## Updating the Environment

If you install new packages, update the environment files:

```bash
# Update environment.yml
conda env export --name eternal > environment.yml

# Update requirements.txt (for pip packages only)
pip list --format=freeze > requirements.txt
```

## Troubleshooting

### Issue: Environment creation fails
- Ensure you have the correct channels configured in conda
- Try updating conda: `conda update conda`
- **ARM64 specific:** Verify you installed the ARM64 version of conda:
  ```bash
  uname -m  # Should output: aarch64
  python -c "import platform; print(platform.machine())"  # Should output: aarch64
  ```

### Issue: OpenCV import errors
- Some systems may need additional libraries:
  ```bash
  sudo apt-get install libgl1-mesa-glx  # For Linux
  ```

### Issue: PyZbar not working
- Install zbar library (ARM64 compatible):
  ```bash
  sudo apt-get update
  sudo apt-get install libzbar0  # For Raspberry Pi OS/Ubuntu/Debian ARM64
  ```

### Issue: GPIO or pigpio not working
- Ensure pigpio daemon is running (for pigpio library):
  ```bash
  sudo pigpiod
  ```
- For RPi.GPIO, ensure user has GPIO permissions:
  ```bash
  sudo usermod -a -G gpio $USER
  # Log out and back in for changes to take effect
  ```

## Environment Name

The environment is named `eternal`. To use a different name, edit the first line of `environment.yml`:

```yaml
name: your_custom_name
```

Then create the environment with:
```bash
conda env create -f environment.yml
```
