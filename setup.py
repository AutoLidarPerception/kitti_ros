# from setuptools import setup
#
# setup(
#     name='pykitti',
#     version='0.2.4',
#     description='A minimal set of tools for working with the KITTI dataset in Python',
#     author='Lee Clement',
#     author_email='lee.clement@robotics.utias.utoronto.ca',
#     url='https://github.com/utiasSTARS/pykitti',
#     download_url='https://github.com/utiasSTARS/pykitti/tarball/0.2.4',
#     license='MIT',
#     packages=['pykitti'],
#     package_dir={'': '3rdparty'},
#     install_requires=['numpy', 'matplotlib']
# )
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # name='pykitti',
    packages=['pykitti'],
    # scripts=['3rdparty/pykitti'],
    package_dir={'': '3rdparty/pykitti'},
)

setup(**d)
