# LISdf
A repository for a universal I/O spec for TAMP, along with scripts to convert from popular specs to our spec


## Installation
### Pip
* This repository uses Python versions 3.8+. We recommend you create a [conda env](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html) or [virtual env](https://docs.python.org/3.8/library/venv.html) with an appropriate Python version before beginning to install.
* Run `pip install -r requirements.txt` to install dependencies.

## Instructions For Contributing
* Run `pip install -r requirements-dev.txt` to install all dependencies for development/contribution to the codebase.
* You can't push directly to main. All contributions will require review to ensure code is understandable and can be built upon. 
    * To contribute new code, make a new branch in this repository (don't use a fork, since that will not properly trigger the checks when you make a PR). When your code is ready for review, [make a PR](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) and request reviews from the appropriate people (if you're not sure who to request a review from, default to Nishanth and/or Will).
* To merge a PR, you need at least one approval, and you have to pass the 4 checks defined in `.github/workflows/predicators.yml` (these will be run automatically on GitHub whenever you push or open a PR). You can run these checks locally in one line via `./scripts/run_checks.sh`, or individually as follows:
    * `pytest -s tests/ --cov-config=.coveragerc --cov=src/ --cov=tests/ --cov-fail-under=100 --cov-report=term-missing:skip-covered --durations=0`
    * `mypy . --config-file mypy.ini`
    * `pytest . --pylint -m pylint --pylint-rcfile=.pylintrc`
    * `yapf -i -r --style .style.yapf . && docformatter -i -r . && isort .`
* The first one is the unit testing check, which verifies that unit tests pass and that code is adequately covered. The "100" means that all lines in every file must be covered.
* The second one is the static typing check, which uses Mypy to verify type annotations. If it doesn't work due to import errors, try `mypy -p predicators --config-file predicators/mypy.ini` from one directory up.
* The third one is the linter check, which runs Pylint with the custom config file `.pylintrc` in the root of this repository. Feel free to edit this file as necessary.
* The fourth one is the autoformatting check, which uses the custom config files `.style.yapf` and `.isort.cfg` in the root of this repository.
