cmake_minimum_required( VERSION 2.8 )
project( Robot-Control-LQG-Estimation )

set( CMAKE_C_STANDARD 99 )
set( CMAKE_C_STANDARD_REQUIRED ON )

find_package( BLAS REQUIRED )
find_package( LAPACK REQUIRED )

include_directories( ${CMAKE_CURRENT_LIST_DIR}/kalman/ )
include_directories( ${CMAKE_CURRENT_LIST_DIR}/linearizer/ )
include_directories( ${CMAKE_CURRENT_LIST_DIR}/lqr/ )

add_library( LQGControl MODULE ${CMAKE_CURRENT_LIST_DIR}/lqg.c ${CMAKE_CURRENT_LIST_DIR}/kalman/kalman_filters.c ${CMAKE_CURRENT_LIST_DIR}/linearizer/system_linearizer.c ${CMAKE_CURRENT_LIST_DIR}/lqr/ilq_regulator.c ${CMAKE_CURRENT_LIST_DIR}/kalman/matrix/matrix.c )
set_target_properties( LQGControl PROPERTIES PREFIX "" )
target_link_libraries( LQGControl -lm ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ) 

add_library( LQGEstimationControl MODULE ${CMAKE_CURRENT_LIST_DIR}/lqg_estimation.c ${CMAKE_CURRENT_LIST_DIR}/kalman/kalman_filters.c ${CMAKE_CURRENT_LIST_DIR}/linearizer/system_linearizer.c ${CMAKE_CURRENT_LIST_DIR}/lqr/ilq_regulator.c ${CMAKE_CURRENT_LIST_DIR}/kalman/matrix/matrix.c )
set_target_properties( LQGEstimationControl PROPERTIES PREFIX "" )
target_link_libraries( LQGEstimationControl -lm ${BLAS_LIBRARIES} ${LAPACK_LIBRARIES} ) 
