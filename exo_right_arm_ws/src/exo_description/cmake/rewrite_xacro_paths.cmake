# Replaces $(find exo_description)/urdf with an absolute path so xacro can run
# before this package is registered in the ament index.
file(READ "${IN}" _content)
string(REPLACE "$(find exo_description)/urdf" "${URDF_DIR}" _content "${_content}")
file(WRITE "${OUT}" "${_content}")
