build_lib(
  LIBNAME sbdp
  SOURCE_FILES
    model/sbdp-header.cc
  HEADER_FILES
    model/sbdp-header.h
  LIBRARIES_TO_LINK
    ${libnetwork}
    ${libcore}
)
