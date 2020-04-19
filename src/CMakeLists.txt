##====================================================================================================================##
##                                                                                                                    ##
##      froggy      Copyright ©2020 by  Malik Kirchner <kirchner@xelonic.com>                                          ##
##                                                                                                                    ##
##      This program is free software: you can redistribute it and/or modify                                          ##
##      it under the terms of the GNU General Public License as published by                                          ##
##      the Free Software Foundation, either version 3 of the License, or                                             ##
##      (at your option) any later version.                                                                           ##
##                                                                                                                    ##
##      This program is distributed in the hope that it will be useful,                                               ##
##      but WITHOUT ANY WARRANTY; without even the implied warranty of                                                ##
##      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                                 ##
##      GNU General Public License for more details.                                                                  ##
##                                                                                                                    ##
##      You should have received a copy of the GNU General Public License                                             ##
##      along with this program.  If not, see <https://www.gnu.org/licenses/>.                                        ##
##                                                                                                                    ##
##====================================================================================================================##

add_executable(froggy froggy.cpp)
target_include_directories( froggy PRIVATE $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}> $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>)
target_link_libraries(froggy ${CONAN_LIBS})

set_property(TARGET froggy PROPERTY CXX_STANDARD 17)
target_compile_features(froggy PUBLIC cxx_std_17)

#install(DIRECTORY "${CMAKE_SOURCE_DIR}/include/" DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}" )
install(TARGETS froggy EXPORT froggy-export RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
#install(TARGETS libfroggy EXPORT froggy-export LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}")
install(EXPORT froggy-export FILE froggy-targets.cmake NAMESPACE xelonic:: DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/froggy")

configure_package_config_file(froggy-config.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/froggy-config.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/froggy)

write_basic_package_version_file( ${CMAKE_CURRENT_BINARY_DIR}/froggy-config-version.cmake VERSION ${PROJECT_VERSION} COMPATIBILITY SameMinorVersion )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/froggy-config.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/froggy-config-version.cmake DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/froggy )
