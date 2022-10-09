# LISdf

<div align="center">
<img src="https://raw.githubusercontent.com/Learning-and-Intelligent-Systems/lisdf/master/docs/assets/lisdf_logo.png" alt="LISdf logo" width="300" role="img">

[![PyPI - Version](https://img.shields.io/pypi/v/lisdf.svg?label=Version)](https://pypi.org/project/lisdf)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/lisdf.svg?label=Python)](https://pypi.org/project/lisdf)
[![PyPI - License](https://img.shields.io/pypi/l/lisdf.svg?label=License)](https://pypi.org/project/lisdf)
[![PyPI - Downloads](https://img.shields.io/pypi/dm/lisdf.svg?label=Downloads)](https://pypistats.org/packages/lisdf)
![Contributors](https://img.shields.io/github/contributors/Learning-and-Intelligent-Systems/lisdf?label=Contributors)
![Build Status](https://github.com/Learning-and-Intelligent-Systems/lisdf/actions/workflows/ci_checks.yml/badge.svg)
</div>

A repository for a universal I/O spec for Task and Motion Planning (TAMP), along with scripts to convert from 
popular specs to our spec. Includes:

- LISdf specification for specifying scenes for TAMP.
- A sophisticated parser for reading `.lisdf` scene files.
- The LISdf Plan Output format, with helpers to read, write, validate and run plans.

**Note:** this repository is under active development and is not intended for general use. 
Please contact `willshen at mit.edu` and `jiayuanm at mit.edu` if you are interested in using this package.

-----

**Table of Contents**


- [Installation](#installation)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)
- [Authors](#authors)
- [Change Log](#change-log)

## Installation

**Dependencies:** this package required Python 3.8 or higher. Although the dependencies within `lisdf` are minimal, 
we recommend you use a [conda env](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html)
or [virtual env](https://docs.python.org/3.8/library/venv.html) with an appropriate Python version.

### Installing with `pip`

```
pip install lisdf
```

### Installing from Source

Clone the repository and install the dependencies with `pip`:

```console
> git clone git@github.com:Learning-and-Intelligent-Systems/lisdf.git
> pip install -e .
```

## Documentation
- LISdf Input Spec: coming soon...
- [LISdf Plan Output Spec (Google Doc)](https://docs.google.com/document/d/15V7K-ljLGx-4hJJaihaDM4-MXGuTXvXhEdNQgRum75A/edit#heading=h.2m2ax6udwea4)

## Contributing

### Dev Dependencies
Follow the instructions below:

1. Clone the repository using `git clone`. 
    - If you are creating a virtual environment within the project directory, then you might want to call it one of
    `.env, env, .venv, venv` as the code checks have been configured to exclude those directories.
2. Run `pip install -e '.[develop]'` to install all dependencies for development/contribution.
3. (Optional, required for unit tests) Install the `lisdf-models` model files by running
   ```
   pip install lisdf_models@git+https://github.com/Learning-and-Intelligent-Systems/lisdf-models.git
   ```
    **WARNING:** the [`lisdf-models` repository](https://github.com/Learning-and-Intelligent-Systems/lisdf-models)
    is ~700MB big as of 10th September 2022.
4. Check [CONTRIBUTING.md](CONTRIBUTING.md) for more information on how to contribute to this repository, including 
    how to run the tests, code checks, and publishing to PyPI.


## License
This repository is licensed under the MIT License. See [LICENSE](LICENSE) for more details.


## Authors
LISdf is an initiative within the [Learning and Intelligent Systems (LIS) group](https://lis.csail.mit.edu) at 
[MIT CSAIL](https://www.csail.mit.edu/).

Contributors and Programmers (alphabetical order):
- Aidan Curtis
- Jiayuan Mao
- Nishanth Kumar
- Sahit Chintalapudi
- Tom Silver
- William Shen
- Zhutian Yang

Other contributors who helped with general discussions, design, feedback (alphabetical order):
- Leslie Kaelbling
- Michael Noseworthy
- Nicholas Roy
- Rachel Holladay
- Tomás Lozano-Pérez
- Yilun Du


## Change Log

### 0.1.0
Initial release to PyPI.

-----

<sub>LISdf = Learning and Intelligent Systems Description Format</sub>
