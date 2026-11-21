build_lib(
  LIBNAME cfr
  SOURCE_FILES model/cfr-policy.cc model/cfr-packet.cc model/cfr-routing.cc
               model/cfr-task-application.cc helper/cfr-helper.cc
               engineering/cfr-eng-routing.cc engineering/cfr-eng-task-application.cc
               engineering/cfr-eng-helper.cc
               helper/cfr-eng-local-adapter.cc
  HEADER_FILES model/cfr-policy.h model/cfr-packet.h model/cfr-routing.h
               model/cfr-task-application.h helper/cfr-helper.h
               engineering/cfr-eng-routing.h engineering/cfr-eng-task-application.h
               engineering/cfr-eng-helper.h
               helper/cfr-eng-local-adapter.h
  LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libinternet} ${libmobility} ${libwifi} ${libcafbcr}
)
