cmake_minimum_required(VERSION 2.8.12)

project("libOpenFPGA")

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
include(FilesToDirs)

#
#
# Determine compiler configuration
#
#

#Compiler flag configuration checks
include(CheckCXXCompilerFlag)

#
# We require c++14 support
#
CHECK_CXX_COMPILER_FLAG("-std=c++14" CXX_COMPILER_SUPPORTS_CXX14_FLAG)
if(CXX_COMPILER_SUPPORTS_CXX14_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
else()
    message(FATAL_ERROR "Could not determine C++14 support for compiler ${CMAKE_CXX_COMPILER}. C++14 support is required.")
endif()

#
# Warning flags
#
set(WARN_FLAGS "") #The actual warning flags to be applied

if(MSVC)
    #Visual studio warnings
    # Note that we don't use /Wall since it generates warnings about standard library headers
    set(WARN_FLAGS_TO_CHECK  #The flags to check if the compiler supports
        "/W4" #Most warnings
        )
else()
    set(WARN_FLAGS_TO_CHECK  #The flags to check if the compiler supports
        #GCC-like
        "-Wall"                         #Most warnings, typically good
        "-Wextra"                       #Extra warning, usually good
        "-Wpedantic"                    #Ensure ISO compliance (i.e. no non-standard extensions)
        "-Wcast-qual"                   #Warn if cast removes qualifier (e.g. const char* -> char*)
        "-Wcast-align"                  #Warn if a cast causes memory alignment changes
        "-Wshadow"                      #Warn if local variable shadows another variable
        "-Wformat=2"                    #Sanity checks for printf-like formatting
        "-Wno-format-nonliteral"        # But don't worry about non-literal formtting (i.e. run-time printf format strings)
        "-Wlogical-op"                  #Checks for logical op when bit-wise expected
        "-Wmissing-declarations"        #Warn if a global function is defined with no declaration
        "-Wmissing-include-dirs"        #Warn if a user include directory is missing
        "-Wredundant-decls"             #Warn if there are overlapping declarations
        "-Wswitch-default"              #Warn if a switch has no default
        "-Wundef"                       #Warn if #if() preprocessor refers to an undefined directive
        "-Wunused"                      #Warn about unused variables/parameters
        "-Wunused-variable"             #Warn about variables that are not used
        "-Wunused-parameter"            #Warn about function parameters which are unused
        "-Wdisabled-optimization"       #Warn when optimizations are skipped (usually due to large/complex code)
        "-Wnoexcept"                    #Warn when functions should be noexcept (i.e. compiler know it doesn't throw)
        "-Woverloaded-virtual"          #Warn when a function declaration overrides a virtual method
        "-Wctor-dtor-privacy"           #Warn about inaccessible constructors/destructors
        "-Wnon-virtual-dtor"            #Warn about missing virtual destructors
        "-Wduplicated-cond"             #Warn about identical conditions in if-else chains
        "-Wduplicated-branches"         #Warn when different branches of an if-else chain are equivalent
        "-Wnull-dereference"            #Warn about null pointer dereference execution paths
        "-Wuninitialized"               #Warn about unitialized values
        "-Winit-self"                   #Warn about self-initialization
        "-Wcatch-value=3"               #Warn when catch statements don't catch by reference
        "-Wextra-semi"                  #Warn about redudnant semicolons
        #GCC-like optional
        #"-Wsuggest-final-types"         #Suggest where 'final' would help if specified on a type methods
        #"-Wsuggest-final-methods"       #Suggest where 'final' would help if specified on methods
        #"-Wsuggest-override"            #Suggest where 'override' should be specified
        #"-Wold-style-cast"              #Warn about using c-style casts
        #"-Wconversion"                  #Warn when type conversions may change value
        #"-Wsign-conversion"             #Warn if a conversion may change the sign
        #"-Wpadded"                      #Will warn if additional padding is introduced to a struct/class. Turn on if optimizing class memory layouts
        #"-Wstrict-overflow=2"           #Warn if the compiler optimizes assuming signed overflow does not occur
        #"-Wfloat-equal"                 #Warn about using direct floating point equality
        #"-Wunsafe-loop-optimizations"   #Warn when loops can't be optimized
        #"-Wswitch-enum"                 #Warn about uncovered enumeration values in a switch (even if there is a default)
        #"-Wsign-promo"                  #Warn when overload resolution converts an unsigned type to signed when an unsigned overload exists
        #"-Wdouble-promotion"            #Warn when float is implicitly propted to double
        #"-Wuseless-cast"                #Warn about casts to the same type
        #"-Wzero-as-null-pointer-constant" #Warn about using '0' instead of nullptr
        )
endif()

#Check and see if the compiler supports the various warning flags,
#and add valid flags
foreach(flag ${WARN_FLAGS_TO_CHECK})
    CHECK_CXX_COMPILER_FLAG(${flag} CXX_COMPILER_SUPPORTS_${flag})
    if(CXX_COMPILER_SUPPORTS_${flag})
        #Flag supported, so enable it
        set(WARN_FLAGS "${WARN_FLAGS} ${flag}")
    endif()
endforeach()

#The flex/bison code is not warning clean so we need to suppress some warnings
set(FLEX_BISON_WARN_SUPPRESS_FLAGS "")
set(FLEX_BISON_WARN_SUPPRESS_FLAGS_TO_CHECK
    "-Wno-redundant-decls"  #Flex/bison generate code with redundant declarations
    "-Wno-switch-default"   #Flex/bison generate switch statments w/o default cases
    "-Wno-unused-parameter" #Flex produces functions with unused params in re-entrant mode
    "-Wno-missing-declarations" #Flex misses some declarations in re-entrant mode
    "-Wimplicit-fallthrough=0" #Bison produces some cases with explicit
    "-Wno-sign-compare" #Flex generates code which performs some signed/unsigned comparison
    "-Wno-null-dereference" #Bison produces some cases wiwth potenetial null derefs
    )
foreach(flag ${FLEX_BISON_WARN_SUPPRESS_FLAGS_TO_CHECK})
    CHECK_CXX_COMPILER_FLAG(${flag} CXX_COMPILER_SUPPORTS_${flag})
    if(CXX_COMPILER_SUPPORTS_${flag})
        #Flag supported, so enable it
        set(FLEX_BISON_WARN_SUPPRESS_FLAGS "${FLEX_BISON_WARN_SUPPRESS_FLAGS} ${flag}")
    endif()
endforeach()

#
# Unit Testing
#
enable_testing()

#
# Sub-projects
#

#Explicitly specify that libreadline has not been found
# This must be explicitly set for ABC to not depend on
# libreadline. Note that because the ABC CMakeLists uses MATCHES
# in its check the value must be explicitly 'FALSE' (in upper case).
#
# TODO: Ideally ABC's CMakeLists.txt should handle this itself,
#       or we should really search for readline correctly.
#       For now just disable readline.
set(READLINE_FOUND FALSE)

#Add the various sub-projects
add_subdirectory(libs)


#PugiXml has some deliberate switch fallthrough cases (as indicated by comments), but they
#are tagged as warnings with g++-7 (the comments don't match g++-7's suppression regexes). 
#Since we don't want to change PugiXml (it is developed externally), we relax the warning 
#level so no fallthrough warnings are generated
CHECK_CXX_COMPILER_FLAG("-Wimplicit-fallthrough=0" CXX_COMPILER_SUPPORTS_-Wimplicit-fallthrough=0)
if(CXX_COMPILER_SUPPORTS_-Wimplicit-fallthrough=0)
    target_compile_options(libpugixml PRIVATE "-Wimplicit-fallthrough=0")
endif()


#Set output locations to be in the main source tree under the relevant folder
set_target_properties(liblog test_log
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/liblog"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/liblog"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/liblog")
set_target_properties(libpugixml
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libpugixml"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libpugixml"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libpugixml")
set_target_properties(libpugiutil
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libpugiutil"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libpugiutil"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libpugiutil")
set_target_properties(libvtrutil
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libvtrutil"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libvtrutil"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libvtrutil")
set_target_properties(libarchfpga read_arch
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libarchfpga"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libarchfpga"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libarchfpga")
set_target_properties(librrgraph test_rrgraph 
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librrgraph"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librrgraph"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librrgraph")
set_target_properties(libsdcparse sdcparse_test
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libsdcparse"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libsdcparse"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libsdcparse")
set_target_properties(libblifparse blifparse_test
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libblifparse"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libblifparse"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libblifparse")
set_target_properties(libeasygl
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libeasygl"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libeasygl"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/libeasygl")
set_target_properties(libtatum
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libtatum"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libtatum"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/EXTERNAL/libtatum")
if(WITH_LIBRTLNUMBER)
    set_target_properties(librtlnumber rtl_number
        PROPERTIES
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librtlnumber"
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librtlnumber"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/libs/librtlnumber")
endif()

