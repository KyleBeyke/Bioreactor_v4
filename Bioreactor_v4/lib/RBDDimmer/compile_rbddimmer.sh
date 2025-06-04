#!/bin/bash
# RBDDimmer .mpy Compilation Script
# Converts your rbddimmer.py library to .mpy format for CircuitPython

echo "RBDDimmer .mpy Compilation Guide"
echo "================================"

# Step 1: Install mpy-cross if not already installed
echo "Step 1: Installing mpy-cross..."
pip install mpy-cross

# Step 2: Download your rbddimmer.py from GitHub
echo "Step 2: Downloading rbddimmer.py from GitHub..."
# Replace with your actual GitHub raw file URL
curl -o rbddimmer.py https://raw.githubusercontent.com/KyleBeyke/Bioreactor_v4/main/rbddimmer.py

# Step 3: Check CircuitPython version compatibility
echo "Step 3: Checking CircuitPython version..."
echo "Make sure your CircuitPython version matches the mpy-cross version"
echo "CircuitPython 9.x requires mpy-cross 9.x"
echo "CircuitPython 8.x requires mpy-cross 8.x"

# Step 4: Compile to .mpy
echo "Step 4: Compiling rbddimmer.py to rbddimmer.mpy..."
mpy-cross rbddimmer.py

# Step 5: Verify the compilation
if [ -f "rbddimmer.mpy" ]; then
    echo "✓ Successfully created rbddimmer.mpy"
    echo "File size comparison:"
    ls -la rbddimmer.py rbddimmer.mpy
    echo ""
    echo "✓ Copy rbddimmer.mpy to your Pico's /lib folder"
else
    echo "✗ Compilation failed"
    exit 1
fi

# Step 6: Optional - compile with specific compatibility
echo "Step 6: For specific CircuitPython versions:"
echo "# For CircuitPython 8.x:"
echo "mpy-cross --version 8 rbddimmer.py"
echo ""
echo "# For CircuitPython 9.x:"
echo "mpy-cross --version 9 rbddimmer.py"

echo ""
echo "Installation Instructions:"
echo "========================="
echo "1. Connect your Pico to computer"
echo "2. Copy rbddimmer.mpy to CIRCUITPY/lib/"
echo "3. In your code use: import rbddimmer"
echo ""
echo "Benefits of .mpy files:"
echo "- Smaller file size"
echo "- Faster loading"
echo "- Less RAM usage during import"
echo "- Pre-compiled bytecode"