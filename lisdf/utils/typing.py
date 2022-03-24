#! /usr/bin/env python3
# -*- coding: utf-8 -*-
# File   : typing.py
# Author : Jiayuan Mao
# Email  : maojiayuan@gmail.com
# Date   : 03/23/2022
#
# This file is part of lisdf.
# Distributed under terms of the MIT license.

# from typing import Generic, TypeVar

import numpy as np

"""
There are a few more options to do fine-grained numpy typing.

1. The official numpy.typing package. Cons: does not support shape typing.
2. https://github.com/ramonhagenaars/nptyping. Cons: needs an additioanl pacakge.
3. Below I used a very simple version from https://stackoverflow.com/a/64032593
"""


# Shape = TypeVar("Shape")
# DType = TypeVar("DType")
#
#
# class NDArray(np.ndarray, Generic[Shape, DType]):
#     pass
#
#
# Vector2f = NDArray[3, np.float32]
# Vector3f = NDArray[3, np.float32]
# Vector4f = NDArray[4, np.float32]
# Vector6f = NDArray[6, np.float32]

Vector2f = np.ndarray
Vector3f = np.ndarray
Vector4f = np.ndarray
Vector6f = np.ndarray
