# Copyright (c) 2022, NXP.
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

macro(get_all_targets_recursive targets dir)
    get_property(subdirectories DIRECTORY ${dir} PROPERTY SUBDIRECTORIES)
    foreach(subdir ${subdirectories})
        get_all_targets_recursive(${targets} ${subdir})
    endforeach()

    get_property(current_targets DIRECTORY ${dir} PROPERTY BUILDSYSTEM_TARGETS)
    list(APPEND ${targets} ${current_targets})
endmacro()

function(get_all_targets var)
    set(targets)
    get_all_targets_recursive(targets ${CMAKE_CURRENT_SOURCE_DIR})
    set(${var} ${targets} PARENT_SCOPE)
endfunction()

function(export_target_to_bin target)
    get_target_property(type ${target} TYPE)
    if(type MATCHES "EXECUTABLE")
        # message(STATUS "Target ${target} will be exported to raw binary format")
        set(target_bin_filename ${target}.bin)
        add_custom_command(
            OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_bin_filename}
            COMMAND ${CMAKE_OBJCOPY} ARGS -v -O binary ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}${CMAKE_EXECUTABLE_SUFFIX_C} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_bin_filename}
            DEPENDS ${target}
        )
        add_custom_target(export_${target}_to_bin ALL
            DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_bin_filename}
        )
    endif()
endfunction()

function(export_target_to_srec target)
    get_target_property(type ${target} TYPE)
    if(type MATCHES "EXECUTABLE")
        # message(STATUS "Target ${target} will be exported to srec format")
        set(target_srec_filename ${target}.srec)
        add_custom_command(
            OUTPUT ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_srec_filename}
            COMMAND ${CMAKE_OBJCOPY} ARGS -v -O srec ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target}${CMAKE_EXECUTABLE_SUFFIX_C} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_srec_filename}
            DEPENDS ${target}
        )
        add_custom_target(export_${target}_to_srec ALL
            DEPENDS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${target_srec_filename}
        )
    endif()
endfunction()

function(export_all_targets_to_bin)
    get_all_targets(all_targets)
    foreach(target ${all_targets})
        export_target_to_bin(${target})
    endforeach()
endfunction()

function(export_all_targets_to_srec)
    get_all_targets(all_targets)
    foreach(target ${all_targets})
        export_target_to_srec(${target})
    endforeach()
endfunction()

function(run_post_build_command)
    get_all_targets(all_targets)
    foreach(target ${all_targets})
        get_target_property(type ${target} TYPE)
        if(type MATCHES "EXECUTABLE")
            list(APPEND executable_targets ${target})
        endif()
    endforeach()
    add_custom_command(
        OUTPUT post_build.output
        COMMAND ${OT_NXP_POST_BUILD_COMMAND} ARGS ${OT_NXP_POST_BUILD_COMMAND_ARGS}
        DEPENDS ${executable_targets}
        COMMENT "Running post build command: ${OT_NXP_POST_BUILD_COMMAND} ${OT_NXP_POST_BUILD_COMMAND_ARGS}"
    )
    add_custom_target(post_build_command ALL
        DEPENDS post_build.output
    )
endfunction()

function(otnxp_git_version git_version)
    execute_process(
        COMMAND git describe --dirty --always --exclude "*"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        OUTPUT_VARIABLE GIT_REV_OTNXP OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND git describe --dirty --always --exclude "*"
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/openthread
        OUTPUT_VARIABLE GIT_REV_OT OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    set(${git_version} "${GIT_REV_OT} OT-NXP/${GIT_REV_OTNXP}" PARENT_SCOPE)
endfunction()