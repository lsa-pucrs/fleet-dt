# fdt-sim (emu/sim/build.sh copies this directory into scratch/fdt-sim of the ns-3 tree): the simulator
# with the regulator core of the vessel (libfdtreg.a, built by build.sh from the C sources of the repository)
build_exec(
  EXECNAME fdt-sim
  EXECNAME_PREFIX scratch_fdt-sim_
  SOURCE_FILES fdt-sim.cc
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}" ${CMAKE_CURRENT_SOURCE_DIR}/libfdtreg.a m
  EXECUTABLE_DIRECTORY_PATH ${CMAKE_OUTPUT_DIRECTORY}/scratch/fdt-sim/
)
target_include_directories(scratch_fdt-sim_fdt-sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
