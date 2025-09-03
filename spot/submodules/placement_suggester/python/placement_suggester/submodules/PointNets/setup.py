#!/usr/bin/env python

from distutils.core import setup

from setuptools import find_packages

setup(
    name="pointnets",
    version="0.1dev",
    author="Ben Eisner",
    packages=find_packages("python"),
    package_dir={"": "python"},
    description="Point network implementations using pytorch geometric",
    long_description=open("README.md").read(),
)
