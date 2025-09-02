find_package(Boost 1.72 QUIET CONFIG COMPONENTS chrono thread filesystem atomic iostreams serialization)

if(NOT Boost_FOUND)
    file(DOWNLOAD https://archives.boost.io/release/1.87.0/source/boost_1_72_0.tar.gz ${CMAKE_BINARY_DIR}/_deps/boost_1_72_0.tar.gz)
    file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/_deps/boost_1_72_0.tar.gz DESTINATION ${CMAKE_BINARY_DIR}/_deps)
    set(BOOST_BUILD_ROOT ${CMAKE_BINARY_DIR}/_deps/Boost-for-Android-ndk_21_boost_1.72.0)
    file(DOWNLOAD https://github.com/dec1/Boost-for-Android/archive/refs/tags/ndk_21_boost_1.72.0.zip ${CMAKE_BINARY_DIR}/_deps/ndk_21_boost_1.72.0.zip)
    file(ARCHIVE_EXTRACT INPUT ${CMAKE_BINARY_DIR}/_deps/ndk_21_boost_1.72.0.zip DESTINATION ${CMAKE_BINARY_DIR}/_deps)
    set(BOOST_CONFIG_FILE ${BOOST_BUILD_ROOT}/patch_and_build.sh)
    file(WRITE ${BOOST_CONFIG_FILE} "#!/usr/bin/env sh\n")
    file(APPEND ${BOOST_CONFIG_FILE} "export BOOST_DIR=${CMAKE_BINARY_DIR}/_deps/boost_1_72_0\n")
    file(APPEND ${BOOST_CONFIG_FILE} "export NDK_DIR=\${HOME}/Android/Sdk/ndk/21.4.7075529\n")
    file(APPEND ${BOOST_CONFIG_FILE} "export ABI_NAMES=\"arm64-v8a\"\n")
    file(APPEND ${BOOST_CONFIG_FILE} "export LINKAGES=\"shared static\"\n")
    file(APPEND ${BOOST_CONFIG_FILE} "sed -i -e 's/unary_function/__unary_function/' ${CMAKE_BINARY_DIR}/_deps/boost_1_72_0/boost/container_hash/hash.hpp\n")
    file(APPEND ${BOOST_CONFIG_FILE} "sh build.sh\n")
    file(CHMOD ${BOOST_CONFIG_FILE} FILE_PERMISSIONS OWNER_EXECUTE)
    file(MAKE_DIRECTORY ${THIRD_PARTY_DIR}/include)
    file(MAKE_DIRECTORY ${THIRD_PARTY_DIR}/lib)
    add_custom_target(Boost_Build
            ALL
            COMMAND ${BOOST_CONFIG_FILE}
            WORKING_DIRECTORY ${BOOST_BUILD_ROOT}
    )

    add_custom_target(Boost_Install
            ALL
            COMMAND mv ${BOOST_BUILD_ROOT}/build/install/include/* ${THIRD_PARTY_DIR}/include/.
            COMMAND mv ${BOOST_BUILD_ROOT}/build/install/lib/* ${THIRD_PARTY_DIR}/lib/.
            WORKING_DIRECTORY ${BOOST_BUILD_ROOT}
    )
    add_dependencies(Boost_Install Boost_Build)
    set(Boost_INCLUDE_DIRS ${THIRD_PARTY_DIR}/include)

    # Boost Headers -----------------------------------------------------
    add_library(Boost::headers INTERFACE IMPORTED)
    set_target_properties(Boost::headers PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
            INTERFACE_COMPILE_DEFINITIONS "BOOST_ALL_NO_LIB"
    )
    add_dependencies(Boost::headers Boost_Install)

    set(_BOOST_COMPONENTS "serialization;chrono;thread;filesystem;atomic;iostreams")

    foreach(boost_comp IN LISTS _BOOST_COMPONENTS)
        add_library(Boost::${boost_comp} UNKNOWN IMPORTED)
        set_target_properties(Boost::${boost_comp} PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}"
                INTERFACE_COMPILE_DEFINITIONS "BOOST_ALL_NO_LIB"
        )
        set_property(TARGET Boost::${boost_comp} APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)

        set_target_properties(Boost::${boost_comp} PROPERTIES
                IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE CXX
                IMPORTED_LOCATION_RELEASE "${THIRD_PARTY_DIR}/lib/libboost_${boost_comp}.a"
        )

        set_target_properties(Boost::${boost_comp} PROPERTIES
                MAP_IMPORTED_CONFIG_MINSIZEREL Release
                MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
        )
        set_property(TARGET Boost::${boost_comp} APPEND PROPERTY INTERFACE_LINK_LIBRARIES Boost::headers)
        add_dependencies(Boost::${boost_comp} Boost_Install)

    endforeach ()
    unset(_BOOST_COMPONENTS)


    # Boost thread -----------------------------------------------------
    include(CMakeFindDependencyMacro)
    find_dependency(Threads)
    set_property(TARGET Boost::thread APPEND PROPERTY INTERFACE_LINK_LIBRARIES Threads::Threads)
    set(BUILDING_BOOST ON)
endif()