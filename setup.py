from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=[
        "drc",
        "drc.manipulator",
        "drc.mobile",
        "drc.mobile_manipulator",
    ],
    package_dir={"": "."},
)

setup(**setup_args)