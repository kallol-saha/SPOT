#!/usr/bin/env python

from distutils.core import setup
from setuptools import find_packages

setup(
    name='placement_suggester',
    version='0.1dev',
    author='Brian Okorn',
    packages=find_packages('python'),
    package_dir={'': 'python'},
    description='Test classes for training self-supervised posegraphs',
    long_description=open('README.md').read(),
    package_data = {'': ['*.mat', '*.npy']},
)

