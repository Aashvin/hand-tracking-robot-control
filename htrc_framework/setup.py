from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


setup_params = generate_distutils_setup(
    packages=['htrc_framework'],
    package_dir={'': 'src'}
)

setup(**setup_params)