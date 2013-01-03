FILE(REMOVE_RECURSE
  "msg_gen"
  "msg_gen"
  "src/razor_imu_9dof/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "msg_gen/lisp/RazorImu.lisp"
  "msg_gen/lisp/_package.lisp"
  "msg_gen/lisp/_package_RazorImu.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
