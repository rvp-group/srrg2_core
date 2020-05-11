 add_library(srrg2_config_library SHARED
   configurable.h
   configurable.cpp
)

add_library(srrg2_configurable_shell_library SHARED
  configurable_shell.h
  configurable_shell.cpp

  configurable_visual_shell.h
  configurable_visual_shell.cpp

  linenoise.h
  linenoise.c
  )

target_link_libraries(
  srrg2_configurable_shell_library
  srrg2_property_library
  srrg2_messages_library
  srrg2_viewer_library
  )
