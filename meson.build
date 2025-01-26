project('motion_planner', 'cpp',
  version : '0.1',
  default_options : ['warning_level=3', 'cpp_std=c++20'])

include_dirs = ['src']
sources = ['src/MotionProfile.cpp', 'test/test.cpp']
executable('motion_planner',
           'test/test.cpp',
           install : true,
           include_directories: include_dirs,
           sources: sources,
           )
