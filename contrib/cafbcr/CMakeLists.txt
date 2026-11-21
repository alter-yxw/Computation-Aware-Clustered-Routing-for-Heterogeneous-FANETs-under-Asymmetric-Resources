check_include_file_cxx(stdint.h HAVE_STDINT_H)
if(HAVE_STDINT_H)
    add_definitions(-DHAVE_STDINT_H)
endif()

set(examples_as_tests_sources)
if(${ENABLE_EXAMPLES})
    set(examples_as_tests_sources
        #test/cafbcr-examples-test-suite.cc
        )
endif()

build_lib(
    LIBNAME cafbcr
    SOURCE_FILES model/cafbcr-election.cc
                 model/cafbcr-packet.cc
                 model/cafbcr-routing.cc
                 model/cafbcr-forwarding-history.cc
                 model/cafbcr-revisit-recovery.cc
                 model/cafbcr-fuzzy-logic.cc
                 model/cafbcr-fuzzy-hello.cc
                 model/cafbcr-qhello.cc
                 model/cafbcr-compute-task-tag.cc
                 model/cafbcr-compute-task-header.cc
                 model/cafbcr-compute-task-app.cc
                 model/cafbcr-cluster-hop-tag.cc
                 model/cafbcr-scenario-config.cc
                 helper/cafbcr-helper.cc
    HEADER_FILES model/cafbcr-election.h
                 model/cafbcr-packet.h
                 model/cafbcr-routing.h
                 model/cafbcr-forwarding-history.h
                 model/cafbcr-revisit-recovery.h
                 model/cafbcr-fuzzy-logic.h
                 model/cafbcr-fuzzy-hello.h
                 model/cafbcr-qhello.h
                 model/cafbcr-compute-task-tag.h
                 model/cafbcr-compute-task-header.h
                 model/cafbcr-compute-task-app.h
                 model/cafbcr-cluster-hop-tag.h
                 model/cafbcr-scenario-config.h
                 helper/cafbcr-helper.h
    LIBRARIES_TO_LINK ${libcore}
    TEST_SOURCES test/cafbcr-test-suite.cc
                 test/cafbcr-qhello-test-suite.cc
                 test/cafbcr-link-probe-test-suite.cc
                 test/cafbcr-task-observation-test-suite.cc
                 test/cafbcr-common-demand-test-suite.cc
                 test/cafbcr-queue-load-test-suite.cc
                 test/cafbcr-neighbor-compute-test-suite.cc
                 test/cafbcr-selection-policy-test-suite.cc
                 test/cafbcr-join-mechanism-test-suite.cc
                 test/cafbcr-direct-maintenance-test-suite.cc
                 test/cafbcr-feedback-covered-cm-test-suite.cc
                 test/cafbcr-control-unicast-test-suite.cc
                 test/cafbcr-udp-observer-test-suite.cc
                 test/cafbcr-task-wire-test-suite.cc
                 test/cafbcr-network-observer-test-suite.cc
                 test/cafbcr-failure-summary-test-suite.cc
                 test/cafbcr-run-diagnostics-test-suite.cc
                 test/cafbcr-forwarding-cache-test-suite.cc
                 test/cafbcr-forwarding-history-test-suite.cc
                 test/cafbcr-revisit-recovery-test-suite.cc
                 ${examples_as_tests_sources}
)
set(CMAKE_CXX_STANDARD 23)
