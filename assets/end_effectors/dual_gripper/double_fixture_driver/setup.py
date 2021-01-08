from setuptools import setup
from shutil import copyfile
import os

setup(
  name='DoubleFixtureDriver',
  version='1.0',
  author='Willem de Graaf',
  author_email='w_degraaf95@live.nl',
  packages=['double_fixture_driver'],
  # scripts=[
          # 'scripts/activate_script.script',
          # 'scripts/close_script.script'
          # 'scripts/open_script.script'],
  description='Driver to control hardware for ROS-I AP Workshop in June 2019 (Robotiq gripper, suction cup and convey belt)',
)

# path = os.path.dirname(__file__)
# copyfile('./scripts/*', path+'scripts/')

