#!/bin/bash

echo "Running autoformatting."
isort . && black .
echo "Autoformatting complete."

echo "Running linting."
flake8
if [ $? -eq 0 ]; then
    echo "Linting passed."
else
    echo "Linting failed! Terminating check script early."
    exit
fi

echo "Running type checking."
mypy . --config-file mypy.ini
if [ $? -eq 0 ]; then
    echo "Type checking passed."
else
    echo "Type checking failed! Terminating check script early."
    exit
fi

echo "Running unit tests."
# Note: tests/ directory is not included in coverage
pytest -s tests/ --cov-config=.coveragerc --cov=lisdf/ --cov-fail-under=75 --cov-report=term-missing:skip-covered --durations=10
if [ $? -eq 0 ]; then
    echo "Unit tests passed."
else
    echo "Unit tests failed!"
    exit
fi

echo "All checks passed!"
