build_lib(
  LIBNAME mognn
  SOURCE_FILES model/mo-policy.cc model/mo-routing.cc model/mo-relay-batch.cc helper/mo-helper.cc
  HEADER_FILES model/mo-policy.h model/mo-routing.h model/mo-relay-batch.h helper/mo-helper.h
  LIBRARIES_TO_LINK ${libcore} ${libnetwork} ${libinternet}
)
