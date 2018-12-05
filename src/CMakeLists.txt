add_library( Slam2D SHARED
        bundle_adjustment.cpp
        model.cpp
        common.cpp)

target_link_libraries( Slam2D
        ${THIRD_PARTY_LIBS}
        )