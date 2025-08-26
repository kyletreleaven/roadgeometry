
from setuptools import setup, find_packages

setup(
	name = "roadbm",
	description = "",
	author = "Kyle Treleaven",
	author_email = "ktreleav@gmail.com",
	version = "0.0.0",
	packages = find_packages(),
	namespace_packages = [ 'setiptah', 'setiptah.roadbm', 'setiptah.nxopt', 'setiptah.vehrouting', ],
	install_requires = [
		'numpy',
		#'scipy',
		'networkx',
		'bintrees',
		# setiptah dependencies
		'roadgeometry'
	],
	dependency_links = [
		'git+https://github.com/kyletreleaven/roadgeometry.git#egg=roadgeometry',
	],
)
