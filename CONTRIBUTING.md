# Contributing to LISdf
Follow the instructions in the [README](README.md) if you haven't already setup your development environment.

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

### Publishing to PyPI
Right now, the plan is to manually publish to PyPI when required. We can automate the release process later if needed.

We use [Hatch](https://hatch.pypa.io/latest/) as our build system because it is lightweight, easy to use yet powerful
in comparison to other tools like `poetry`. 

1. Increment `__version__` in `lisdf/__init__.py` to the new version number. 
    - We try to follow the [semantic versioning](https://semver.org/) convention.
2. Install `hatch` via `pip install hatch` if you haven't already.
3. Build the package via `hatch build`
4. Inspect the build artifacts in `dist/` to ensure that everything looks good.
    - Note: the `.whl` file is just a zip file, so you can unzip it to inspect the contents.
5. Publish to PyPI via `hatch publish`
    - Pre-requisite: you will need to have a PyPI account and be added as a collaborator of the package. Ask Will to 
    add you if you don't have access yet.
