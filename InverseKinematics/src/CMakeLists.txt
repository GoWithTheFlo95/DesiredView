
FIND_PACKAGE( PCL 1.8.1 REQUIRED )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${PCL_LIBRARY_DIRS} )

ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)

FILE( GLOB InverseKinematic_SRC
	"*.h"
	"*.cpp"
)

ADD_EXECUTABLE( InverseKinematic ${InverseKinematic_SRC} )
TARGET_LINK_LIBRARIES( InverseKinematic PUBLIC ${PCL_LIBRARIES} )