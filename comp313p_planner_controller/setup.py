from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['comp313p_planner_controller'],
    packages_dir = {'': 'src'}
    )

setup(**d)
