from setuptools import setup, find_packages

setup(
	name="jaus",
	version="0.1",
	packages=find_packages(),
	install_requires=["twisted", "pytest"],
	entry_points={
		"console_scripts": [
			"test_twisted = jaus.run:test_twisted"
		]
	}
)
