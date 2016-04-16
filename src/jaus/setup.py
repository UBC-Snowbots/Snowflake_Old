from setuptools import setup, find_packages

setup(
    name="jaus",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        "pytest",
        "bitstring",
        "git+git://github.com/olivierverdier/dispatch.git@ae6a88633183a7ba3830d06fd0ce46f7e314bdd5",
        "pytest-asyncio",
    ],
    entry_points={}
)
