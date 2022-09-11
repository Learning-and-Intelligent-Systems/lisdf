# LISdf
![Build Status](https://github.com/Learning-and-Intelligent-Systems/lisdf/actions/workflows/ci_checks.yml/badge.svg)
![License](https://img.shields.io/badge/license-MIT-blue.svg)

**Note:** this repository is under active development and is not yet ready for use. We will update this README
once LISdf is ready for use.

A repository for a universal I/O spec for Task and Motion Planning (TAMP), along with scripts to convert from 
popular specs to our spec.

## Installation

### Dependencies

This repository requires Python 3.8+. We recommend you create a 
[conda env](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) or 
[virtual env](https://docs.python.org/3.8/library/venv.html) with an appropriate Python version before installing
the other dependencies.

If you are creating a virtual environment within the project directory, then you might want to call it one of
`.env, env, .venv, venv` as the code checks have been configured to exclude those directories.

### Installing with `pip`

Run `pip install lisdf@git+https://github.com/Learning-and-Intelligent-Systems/lisdf.git` to install.

### Installing from Source

1. Clone the repository using `git clone`. 
2. Run `pip install -e .` to install.

## Instructions For Contributing

### Dev Dependencies
Clone the repository using `git clone`. Then run `pip install -e '.[develop]'` to install all dependencies for development/contribution.

Warning: the dev dependencies include the [`lisdf-models` repository](https://github.com/Learning-and-Intelligent-Systems/lisdf-models) which is ~700MB big as of 10th September 2022.

### Pushing your Changes
You can't directly push to the `main` branch.  All contributions will require review to ensure code is understandable 
and can be built upon. 

1. To contribute new code, make a new Git branch in this repository (don't use a fork, since that will not properly trigger 
   the checks when you make a PR). When your code is ready for review, [make a PR](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request)
   and request reviews from the appropriate people (if you're not sure who to request a review from, default to Nishanth
   and/or Will).
2. To merge a PR, you need at least one approval, and you have to pass the 4 checks defined in 
   `.github/workflows/ci_checks.yml` (these will be run automatically on GitHub whenever you push or open a PR).
   You can run these checks locally in one line via `./scripts/run_checks.sh`. For more details, refer to the next section.

### Formatting and Checks
We use `isort` and `black` for autoformatting, `flake8` and `mypy` for linting and type checking, and `pytest` 
for unit and integration testing. You can run all these commands in one simple script via `./scripts/run_checks.sh`.

If you wish to run these commands individually, you can run these commands from the repo directory:
1. `isort . && black .`
2. `flake8 .`
3. `mypy . --config-file mypy.ini`
4. `pytest -s tests/ --cov-config=.coveragerc --cov=lisdf/ --cov-fail-under=75 --cov-report=term-missing:skip-covered --durations=10`

* The 1st command is the autoformatting check, which runs `black` and `isort`.
* The 2nd command is the linter check, which runs `flake8`
* The 3rd command is the static typing check, which uses `mypy` to verify type annotations. 
* The 4th command is the unit testing check, which verifies that unit tests pass and that code is adequately covered. 
  * The "75" means that 75% of all lines in every file in `lisdf/` must be covered (excludes `tests/`).
