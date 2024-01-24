from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['nav_module'],
    package_dir={'': 'src_nav'}
)
setup(**d)