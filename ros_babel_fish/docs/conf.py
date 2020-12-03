# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import subprocess
# import sys
# sys.path.insert(0, os.path.abspath('.'))

on_rtd = os.environ.get('READTHEDOCS', None) == 'True'

if not on_rtd:
  import sphinx_rtd_theme
  html_theme = 'sphinx_rtd_theme'
  html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]

# -- Project information -----------------------------------------------------

project = 'ROS BabelFish'
copyright = '2019, Stefan Fabian'
author = 'Stefan Fabian'


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [ "breathe", "exhale" ]

# Breathe Configuration
breathe_projects = { "rbf" : os.path.abspath("xml") }
breathe_default_project = "rbf"

# Exhale Configuration
exhale_args = {
  "verboseBuild": False,
  "containmentFolder": "api",
  "rootFileName": "index.rst",
  "rootFileTitle": "Library API",
  "doxygenStripFromPath": "..",
  "createTreeView": True,
  "exhaleExecutesDoxygen": True,
  "exhaleUseDoxyfile": True,
  "pageLevelConfigMeta": ":github_url: https://github.com/StefanFabian/ros_babel_fish/"
}

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = []


# -- Options for HTML output -------------------------------------------------

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Tell sphinx what the primary language being documented is.
primary_domain = 'cpp'

# Tell sphinx what the pygments highlight language should be.
highlight_language = 'cpp'
