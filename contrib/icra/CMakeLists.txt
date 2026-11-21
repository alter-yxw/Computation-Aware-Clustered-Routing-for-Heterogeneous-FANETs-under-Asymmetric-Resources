check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/icra-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME icra
    SOURCE_FILES # model/icra-clustermanager.cc
                # model/icra-neighbor.cc
                # model/icra-node.cc
                 model/icra-packet.cc
                 model/icra-rqueue.cc
                 model/icra-rtable.cc
                # model/performance-metrics.cc
                 model/icra-Qweight.cc
                 model/icra-routing.cc                 
                 helper/icra-helper.cc
    HEADER_FILES # model/icra-clustermanager.h
                 # model/icra-neighbor.h
                 # model/icra-node.h
                 model/icra-packet.h
                 model/icra-rqueue.h
                 model/icra-rtable.h
                # model/performance-metrics.h
                 model/icra-Qweight.h
                 model/icra-routing.h
                 helper/icra-helper.h
                 helper/icra-helper.h
    LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libinternet} ${libwifi}
                      ${libmobility} ${libenergy} ${libfanet}
    TEST_SOURCES test/icra-test-suite.cc
                 ${examples_as_tests_sources}
)
set(CMAKE_CXX_STANDARD 23)

if(ENABLE_TESTS)
  build_exec(
    EXECNAME icra-tests
    SOURCE_FILES test/icra-test-runner.cc
    # FANET's existing dispatch references these protocol classes. Resolve them
    # in this test executable without changing the shared FANET module.
    LIBRARIES_TO_LINK ${LIB_AS_NEEDED_PRE} ${testicra} ${libcafbcr}
                      ${LIB_AS_NEEDED_POST} ${libcore}
    EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/utils/
  )
endif()
