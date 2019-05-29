FILE(GLOB sources *.cpp)
FILE(GLOB common_sources ../*.cpp)

add_library(valkyrie_dynamic_model SHARED ${sources} ${common_sources})
target_link_libraries (valkyrie_dynamic_model sejong_utils sejong_rbdl sejong_urdf ${TinyXML_LIBRARIES})

