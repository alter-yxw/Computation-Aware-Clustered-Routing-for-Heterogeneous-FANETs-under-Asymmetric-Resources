build_lib(
  LIBNAME qrcf
  SOURCE_FILES model/qrcf-model.cc model/qrcf-packet.cc model/qrcf-neighbor-table.cc
               model/qrcf-learning.cc model/qrcf-diagnostics.cc model/qrcf-settings.cc
               model/qrcf-discovery.cc model/qrcf-route-planner.cc
               model/qrcf-routing.cc helper/qrcf-helper.cc
  HEADER_FILES model/qrcf-model.h model/qrcf-packet.h model/qrcf-neighbor-table.h
               model/qrcf-learning.h model/qrcf-diagnostics.h model/qrcf-settings.h
               model/qrcf-discovery.h model/qrcf-route-planner.h
               model/qrcf-routing.h helper/qrcf-helper.h
  LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libinternet} ${libmobility} ${libenergy} ${libwifi}
  TEST_SOURCES test/qrcf-test-suite.cc test/qrcf-routing-test-suite.cc
               test/qrcf-value-wire-test-suite.cc test/qrcf-priority-test-suite.cc
)

if(ENABLE_TESTS)
  build_exec(
    EXECNAME qrcf-tests
    SOURCE_FILES test/qrcf-test-runner.cc
    LIBRARIES_TO_LINK ${LIB_AS_NEEDED_PRE} ${testqrcf} ${LIB_AS_NEEDED_POST} ${libcore}
    EXECUTABLE_DIRECTORY_PATH ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/utils/
  )
endif()
