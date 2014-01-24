from setuptools import setup, find_packages
import sys,os

setup(name='motmot.fview_draw_geom',
      description='draw geom targets on fview window and publish over ROS',
      version='0.0.2',
      packages = find_packages(),
      author='John Stowers',
      author_email='john.stowers@gmail.com',
      namespace_packages = ['motmot'],
      entry_points = {
    'motmot.fview.plugins':'fview_draw_geom = motmot.fview_draw_geom.fview_draw_geom:FviewDrawGeom',
    },
      )
