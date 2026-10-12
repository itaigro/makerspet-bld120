"""Gazebo search paths from ROS package exports.

Same rule as ``GazeboRosPaths`` in ros_gz_sim's ``gz_sim.launch.py``: every
package whose ``package.xml`` has
``<export><gazebo_ros gazebo_model_path=... gazebo_media_path=... plugin_path=.../>``
contributes those directories (``${prefix}`` = the package's share directory).
Upstream relies on this, e.g. ``makerspet_mini`` exports ``${prefix}/sdf``,
which is where Gazebo finds ``model://makerspet_mini/mesh/hemisphere.stl``.

``sim.launch.py`` starts ``gz sim`` itself (see ``gazebo()`` there) and so has
to build these paths itself.
"""
import os

from ament_index_python.packages import get_package_share_directory, get_packages_with_prefixes
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package


def gazebo_ros_paths():
    """Return ``(resource_paths, plugin_paths)`` exported by installed packages."""
    model, media, plugin = [], [], []
    for name in sorted(get_packages_with_prefixes()):
        share = get_package_share_directory(name)
        manifest = os.path.join(share, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(manifest):
            continue
        try:
            package = parse_package(manifest)
        except InvalidPackage:
            continue
        for export in package.exports:
            if export.tagname != 'gazebo_ros':
                continue
            for key, bucket in (('gazebo_model_path', model),
                                ('gazebo_media_path', media),
                                ('plugin_path', plugin)):
                if key in export.attributes:
                    bucket.append(export.attributes[key].replace('${prefix}', share))
    return model + media, plugin


def gazebo_env(environ=os.environ):
    """Environment additions for ``gz sim``, extending any existing values."""
    resource, plugin = gazebo_ros_paths()

    def join(*parts):
        return os.pathsep.join(p for p in parts if p)

    return {
        'GZ_SIM_RESOURCE_PATH': join(environ.get('GZ_SIM_RESOURCE_PATH', ''), *resource),
        'GZ_SIM_SYSTEM_PLUGIN_PATH': join(environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                                          environ.get('LD_LIBRARY_PATH', ''), *plugin),
    }
