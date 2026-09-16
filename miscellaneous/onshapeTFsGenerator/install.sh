#!/usr/bin/env bash
# Creates a virtual environment and installs all required dependencies
# for running onshapeTFs2yaml.py

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "Creating virtual environment at ${SCRIPT_DIR}/.venv..."
python3 -m venv .venv
source .venv/bin/activate

echo "Upgrading pip..."
pip install --upgrade pip

echo "Installing dependencies..."
pip install onshape-robotics-toolkit
pip install scipy numpy matplotlib pyyaml

echo "Installation complete. Virtual environment created at ${SCRIPT_DIR}/.venv"
