FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/i90_position/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/i90_position/msg/__init__.py"
  "../src/i90_position/msg/_pos.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
