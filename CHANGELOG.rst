^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package bob_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2026-02-03)
------------------
* Fix linter and style issues for ROS2 compliance
* Standardize docstrings and copyright headers
* Refactor launch parameter
* Add support for 'prefix' parameter in configuration
* Add support for inline 'parameters' in node configuration
* Add support for '//PKGSHARE' and '//PKGSHARE:pkg' placeholders for dynamic path resolution
* Add support for environment variable substitution (${VAR} and ${VAR:-default}) in configuration files
* Enable dynamic placeholder resolution for 'config' and 'config_nodes' launch arguments
* Standardize placeholder resolution across all configuration fields including 'launch_ns'
* Contributors: Bob Ros

1.0.0 (2026-02-02)
------------------
* Initial release
* Contributors: Bob Ros