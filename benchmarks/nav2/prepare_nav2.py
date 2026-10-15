#!/usr/bin/env python3
"""Fetch hash-pinned upstream Smac and add the benchmark-only observation hook."""
import hashlib
import io
from pathlib import Path
import shutil
import tarfile
import urllib.request

COMMIT = "a097086719c88f781aa59788eca29ac6ca5e56db"
SHA256 = "8b0fdd2aa41947d48d543ad24a6e5dc7fe885f1014a2bd43e5410985c5ff31af"
data = urllib.request.urlopen(
    f"https://codeload.github.com/ros-navigation/navigation2/tar.gz/{COMMIT}"
).read()
if hashlib.sha256(data).hexdigest() != SHA256:
    raise RuntimeError("Nav2 source archive hash mismatch")
destination = Path("/opt/nav2_source")
prefix = f"navigation2-{COMMIT}/nav2_smac_planner/"
with tarfile.open(fileobj=io.BytesIO(data)) as archive:
    for member in archive.getmembers():
        if member.name.startswith(prefix) and member.isfile():
            relative = Path(member.name[len(prefix):])
            if ".." in relative.parts or relative.is_absolute():
                raise RuntimeError("unsafe archive path")
            output = destination / relative
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_bytes(archive.extractfile(member).read())

source = destination / "src/analytic_expansion.cpp"
text = source.read_text()
old = '#include "nav2_smac_planner/analytic_expansion.hpp"'
assert text.count(old) == 1
text = text.replace(old, old + '\n#include "nav2_smac_planner/prefix_observer.hpp"')
old = "  node->setMotionPrimitive(nullptr);"
assert text.count(old) == 1
text = text.replace(old, "  rs_benchmark_observer::capture(node);\n" + old)
source.write_text(text)
shutil.copy("prefix_observer.hpp", destination / "include/nav2_smac_planner")
shutil.copy("prefix_observer.cpp", destination / "src")
cmake = destination / "CMakeLists.txt"
text = cmake.read_text()
old = "ament_package()"
assert text.count(old) == 1
text = text.replace(old, """
# One shared observer store for all three upstream plugin libraries.
add_library(rs_benchmark_prefix_observer SHARED src/prefix_observer.cpp)
ament_target_dependencies(rs_benchmark_prefix_observer ${dependencies})
target_link_libraries(rs_benchmark_prefix_observer ${OMPL_LIBRARIES})
foreach(planner ${library_name} ${library_name}_2d ${library_name}_lattice)
  target_link_libraries(${planner} rs_benchmark_prefix_observer)
endforeach()
install(TARGETS rs_benchmark_prefix_observer LIBRARY DESTINATION lib)
ament_package()
""")
cmake.write_text(text)
