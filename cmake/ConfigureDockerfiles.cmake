set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include(JaiaVersions)
include(ReadCommonVersions)

if(PROJECT_GIT_BUILD)
  set(JAIABOT_BRANCH_FOR_DOCKER "${PROJECT_VERSION_GITBRANCH}")
else()
  set(JAIABOT_BRANCH_FOR_DOCKER "${JAIA_VERSION_RELEASE_BRANCH}")
endif()

set(DOCKERFILES
  ${CMAKE_SOURCE_DIR}/.docker/focal/amd64/Dockerfile.in
  ${CMAKE_SOURCE_DIR}/.docker/focal/arm64/Dockerfile.in
  ${CMAKE_SOURCE_DIR}/scripts/sim-docker/Dockerfile.in
  ${CMAKE_SOURCE_DIR}/scripts/test-setup-build/Dockerfile.in
  )

foreach(I ${DOCKERFILES})
  string(REPLACE ".in" "" OUT ${I})
  configure_file(${I} ${OUT} @ONLY)

  # make Dockerfile same owner as Dockerfile.in
  execute_process(
    COMMAND chown --reference=${I} ${OUT}
    )
  
endforeach()
