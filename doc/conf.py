# Documentation build configuration file.
# Reference: https://www.sphinx-doc.org/en/master/usage/configuration.html

import subprocess

subprocess.call("doxygen Doxyfile.in", shell=True)

# -- Project --------------------------------------------------------------

project = "NTURT Rear Box"
copyright = "2024, NTU Racing Team"
author = "NTU Racing Team members"

# -- General configuration ------------------------------------------------

extensions = [
    "breathe",
    "sphinx.ext.autodoc",
    "sphinx.ext.ifconfig",
    "sphinx.ext.mathjax",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx_rtd_theme",
]

templates_path = ["_templates"]

exclude_patterns = ["_build"]

# -- Options for HTML output ----------------------------------------------

html_theme = "sphinx_rtd_theme"
html_theme_options = {
    "logo_only": True,
    "prev_next_buttons_location": "bottom",
}
# html_baseurl = ""
html_title = "NTURT Rear Box Documentation"
html_logo = "images/logo.png"
# github_url = ""
html_static_path = ["_static"]

# -- Options for Breathe plugin -------------------------------------------

breathe_projects = {
	"NTURT Rear Box": "_build/xml/"
}
breathe_default_project = "NTURT Rear Box"
breathe_domain_by_extension = {
    "h": "c",
    "c": "c",
}
breathe_show_enumvalue_initializer = True
breathe_default_members = ("members", )
