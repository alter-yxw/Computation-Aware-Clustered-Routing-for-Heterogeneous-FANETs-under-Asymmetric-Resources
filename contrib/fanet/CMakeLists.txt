check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/fanet-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME fanet
    SOURCE_FILES model/fanet-node.cc
                 model/fanet-neighbor.cc
                 model/fanet-clustermanager.cc
                 model/performance-metrics.cc
                 helper/fanet-helper.cc
    HEADER_FILES model/fanet-node.h
                 model/fanet.h
                 model/fanet-neighbor.h
                 model/fanet-clustermanager.h
                 model/performance-metrics.h
                 helper/fanet-helper.h
                 helper/fanet-attack-layer.h
                 helper/fanet-radio-accounting.h
                 helper/secure-routing-metrics.h
                 helper/secure-benchmark-results.h
    LIBRARIES_TO_LINK ${libcore}
                      ${libnetwork}
                      ${libwifi}
                      ${libinternet}
                      ${libtraffic-control}
    TEST_SOURCES test/fanet-test-suite.cc
                 test/fanet-behavior-test-suite.cc
                 test/fanet-neighbor-update-test-suite.cc
                 ${examples_as_tests_sources}
)
