if (NOT TARGET sofa)
    add_subdirectory(sofa)
endif ()

if (NOT TARGET eigen)
    add_subdirectory(eigen)
endif ()

if (NOT TARGET gtest)
    add_subdirectory(googletest)
endif ()

if (NOT TARGET json)
    add_subdirectory(json)
endif ()