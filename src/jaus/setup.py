from setuptools import setup, find_packages

setup(
    name="jaus",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "pytest",
        "bitstring",
        "pytest-asyncio",
        "pytest-catchlog",
    ],
    entry_points={}
)
