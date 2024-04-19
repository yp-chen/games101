#include "Scene.hpp"
#include "Sphere.hpp"
#include "Triangle.hpp"
#include "Light.hpp"
#include "Renderer.hpp"

// In the main function of the program, we create the scene (create objects and lights)
// as well as set the options for the render (image width and height, maximum recursion
// depth, field-of-view, etc.). We then call the render function().
int main()
{
    Scene scene(1280, 960);

    auto sph1 = std::make_unique<Sphere>(Vector3f(-1, 0, -12), 2);//圆心是(-1, 0, -12)，半径是2
    sph1->materialType = DIFFUSE_AND_GLOSSY;//材质为DIFFUSE_AND_GLOSSY，可以看作在这个物体上的反射是粗糙反射（介于镜面反射和漫反射之间）与漫反射的叠加
    sph1->diffuseColor = Vector3f(0.6, 0.7, 0.8);//漫反射底色为(0.6, 0.7, 0.8)

    auto sph2 = std::make_unique<Sphere>(Vector3f(0.5, -0.5, -8), 1.5);//圆心是(0.5, -0.5, -8)，半径是1.5
    sph2->ior = 1.5;//折射率为1.5
    sph2->materialType = REFLECTION_AND_REFRACTION;//材质为REFLECTION_AND_REFRACTION，可以看作在这个物体上的反射是镜面反射和折射的叠加

    scene.Add(std::move(sph1));
    scene.Add(std::move(sph2));//将两个球体加入场景

    Vector3f verts[4] = {{-5,-3,-6}, {5,-3,-6}, {5,-3,-16}, {-5,-3,-16}};//网格的四个顶点
    uint32_t vertIndex[6] = {0, 1, 3, 1, 2, 3};//两个三角形的顶点序号
    Vector2f st[4] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
    auto mesh = std::make_unique<MeshTriangle>(verts, vertIndex, 2, st);//2指的就是由两个三角形组成
    mesh->materialType = DIFFUSE_AND_GLOSSY;//设置网格材质为DIFFUSE_AND_GLOSSY

    scene.Add(std::move(mesh));//网格加入场景
    scene.Add(std::make_unique<Light>(Vector3f(-20, 70, 20), 0.5));//点光源坐标是(-20, 70, 20)，intensity是0.5
    scene.Add(std::make_unique<Light>(Vector3f(30, 50, -12), 0.5));    

    Renderer r;
    r.Render(scene);//将场景加入渲染器进行渲染

    return 0;
}