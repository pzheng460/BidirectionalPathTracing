cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

add_executable(RayTracing main.cpp PathVertex.cpp PathVertex.hpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp
        Camera.hpp
        Camera.cpp)
