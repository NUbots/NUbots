#include <array>
#include <iostream>

#include "Timer.hpp"

#include "ArrayPrint.hpp"
#include "Circle.hpp"
#include "Cylinder.hpp"
#include "Sphere.hpp"
#include "VisualMesh.hpp"

using Scalar = float;

std::array<std::array<Scalar, 4>, 4> generateHoc(const Scalar& theta,
                                                 const Scalar& phi,
                                                 const Scalar& lambda,
                                                 const Scalar& height) {


    Scalar ct = std::cos(theta);
    Scalar st = std::sin(theta);
    Scalar cp = std::cos(phi);
    Scalar sp = std::sin(phi);
    Scalar cl = std::cos(lambda);
    Scalar sl = std::sin(lambda);


    std::array<std::array<Scalar, 4>, 4> Hoc;

    // Rotation matrix
    Hoc[0][0] = ct * cp;
    Hoc[0][1] = ct * sp;
    Hoc[0][2] = -st;
    Hoc[1][0] = sl * st * cp - cl * sp;
    Hoc[1][1] = sl * st * sp + cl * cp;
    Hoc[1][2] = ct * sl;
    Hoc[2][0] = cl * st * cp + sl * sp;
    Hoc[2][1] = cl * st * sp - sl * cp;
    Hoc[2][2] = ct * cl;

    // Lower row
    Hoc[3][0] = 0;
    Hoc[3][1] = 0;
    Hoc[3][2] = 0;
    Hoc[3][3] = 1;

    // Translation
    Hoc[0][3] = 0;
    Hoc[1][3] = 0;
    Hoc[2][3] = height;

    return Hoc;
}

int main() {

    mesh::Cylinder<Scalar> cylinder(0, 2.0, 0.075, 15, 20);
    mesh::Sphere<Scalar> sphere(0, 0.075, 1, 10);
    mesh::Circle<Scalar> circle(0, 0.075, 1, 10);

    Timer t;
    mesh::VisualMesh<Scalar> mesh(cylinder, 1.0, 1.1, 1, M_PI / 1024.0);
    t.measure("Generated visual mesh");

    mesh::VisualMesh<Scalar>::Lens lens;
    lens.type                                = mesh::VisualMesh<Scalar>::Lens::EQUIRECTANGULAR;
    lens.dimensions                          = {{1280, 1024}};
    lens.equirectangular.fov                 = {{1.0472, 0.785398}};
    lens.equirectangular.focal_length_pixels = (lens.dimensions[0] * 0.5) / std::tan(lens.equirectangular.fov[0] * 0.5);

    // lens.type       = mesh::VisualMesh<Scalar>::Lens::RADIAL;
    // lens.radial.fov = 1.0472;

    for (int i = 0; i < 10000; ++i) {
        auto Hoc = generateHoc((Scalar(rand()) / Scalar(RAND_MAX)) * (M_PI * 2.0),
                               (Scalar(rand()) / Scalar(RAND_MAX)) * (M_PI * 2.0),
                               (Scalar(rand()) / Scalar(RAND_MAX)) * (M_PI * 2.0),
                               0.5);
        t.measure("Generated Hoc");
        try {
            mesh.classify(nullptr, 0, mesh::VisualMesh<Scalar>::FOURCC::YUYV, Hoc, lens);
        }
        catch (const std::exception& ex) {
            std::cout << ex.what() << std::endl;
        }

        t.measure("Classified");
    }
}
