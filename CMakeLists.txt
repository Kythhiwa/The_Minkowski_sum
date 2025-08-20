cmake_minimum_required(VERSION 3.10)
project(The_Minkowski_sum)
# Более явное указание на необходимость C++11

# Настройка поиска OpenGL и GLUT
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)

# Для GLUT в разных системах могут быть разные названия
find_package(GLUT REQUIRED)
if(NOT GLUT_FOUND)
    # Альтернативные имена для GLUT
    find_package(FreeGLUT REQUIRED)
endif()

# Список исходных файлов
set(SOURCES
    src/main.cpp
    src/dcel.h
    src/dcel.cpp
    src/config.h
    src/halfEdge.cpp
    src/halfEdge.h 
    src/vertex.cpp
    src/vertex.h 
    src/face.h
    src/sweepline.cpp
    src/sweepline.h
    src/eventqueue.h
    src/eventI.h 
    src/eventI.cpp 
    src/render.h 
    src/render.cpp
    src/genran.h 
    src/genran.cpp 
    src/eventT.h 
    src/eventT.cpp
)

# Создание исполняемого файла
add_executable(the_minkowski_sum ${SOURCES})
# Подключение библиотек
target_link_libraries(the_minkowski_sum
    OpenGL::OpenGL
    OpenGL::GLU
    GLUT::GLUT
)

