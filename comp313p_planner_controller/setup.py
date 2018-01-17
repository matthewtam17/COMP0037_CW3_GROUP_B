from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['comp313p_planner_controller.controller',
                    'comp313p_planner_controller.planner.algorithms',
                    'comp313p_planner_controller.planner.base',
                    'comp313p_planner_controller.planner.graphics'],
    packages_dir = {'': 'src'}
    )

setup(**d)
