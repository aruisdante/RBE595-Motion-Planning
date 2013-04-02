add_executable(system_benchmark SystemBenchmark.cpp)
target_link_libraries(system_benchmark pendsys carsys rgrrt ${OMPL_LIBRARIES} ${OMPLAPP_LIBRARY})
