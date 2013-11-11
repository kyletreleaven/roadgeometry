
from setuptools import setup, find_packages

setup(
	name = "roadgeometry",
	description = "A small library for road network geometry queries",
	author = "Kyle Treleaven",
	author_email = "ktreleav@gmail.com",
	version = "0.0.0",
	packages = find_packages(),
	namespace_packages = [ 'setiptah', 'setiptah.roadgeometry', ],
)

