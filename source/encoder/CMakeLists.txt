# vim: syntax=cmake

if(GCC)
    add_definitions(-Wno-uninitialized)
    if(CC_HAS_NO_STRICT_OVERFLOW)
        # GCC 4.9.2 gives warnings we know we can ignore in this file
        set_source_files_properties(slicetype.cpp PROPERTIES COMPILE_FLAGS -Wno-strict-overflow)
    endif(CC_HAS_NO_STRICT_OVERFLOW)
endif()
if(MSVC)
   add_definitions(/wd4701) # potentially uninitialized local variable 'foo' used
endif()

if(EXTRA_LIB)
    if(LINKED_8BIT)
        list(APPEND APIFLAGS "-DLINKED_8BIT=1")
    endif(LINKED_8BIT)
    if(LINKED_10BIT)
        list(APPEND APIFLAGS "-DLINKED_10BIT=1")
    endif(LINKED_10BIT)
    if(LINKED_12BIT)
        list(APPEND APIFLAGS "-DLINKED_12BIT=1")
    endif(LINKED_12BIT)
    string(REPLACE ";" " " APIFLAGSTR "${APIFLAGS}")
    set_source_files_properties(api.cpp PROPERTIES COMPILE_FLAGS ${APIFLAGSTR})
endif(EXTRA_LIB)

add_library(encoder OBJECT ../x265.h
	slicetype.cpp slicetype.h
    bitcost.cpp bitcost.h rdcost.h
    motion.cpp motion.h
    level.cpp level.h
    dpb.cpp dpb.h
    reference.cpp reference.h
    encoder.cpp encoder.h
    api.cpp)
