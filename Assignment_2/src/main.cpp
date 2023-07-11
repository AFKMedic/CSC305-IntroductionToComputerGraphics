/*
Chris Wong
V00780634
Assignment 2
*/
// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

//Take supplied input and put it into the form Ax = b and return x
Vector3d solveEquation(Vector3d b, Vector3d A1, Vector3d A2, Vector3d A3) {
    Matrix3d A;
    A.col(0) = A1.transpose();
    A.col(1) = A2.transpose();
    A.col(2) = A3.transpose();
    return A.inverse()*b;
}

//Solve the quadratic for calculating insterection of a sphere
Vector2d solveSphere(Vector3d e, Vector3d d, Vector3d c, double discriminant) {
    Vector2d t;
    t.x() = ((-d).dot(e - c) - sqrt(discriminant)) / (d.dot(d));
    t.y() = ((-d).dot(e - c) + sqrt(discriminant)) / (d.dot(d));
    return t;
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            //Return missing variables
            Vector3d uvt = solveEquation(pgram_origin - ray_origin, -pgram_u, -pgram_v, ray_direction);

            // Check if intersects with parallelogram
            if (uvt.z() >= 0 && uvt.x() >= 0 && uvt.y() >= 0 && uvt.x() <= 1 && uvt.y() <= 1)
            {

                Vector3d ray_intersection = ray_origin + (uvt.z() * ray_direction);

                //Assign the normal of the plane of the parallelogram
                Vector3d ray_normal = (pgram_v.cross(pgram_u)).normalized();
                
                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            const Vector3d ray_origin = pixel_center;
            //const Vector3d ray_direction = camera_view_direction;
            Vector3d ray_direction = pixel_center - camera_origin;
            Vector3d pgram_uvt = solveEquation(pgram_origin - ray_origin, -pgram_u, -pgram_v, ray_direction);
           
            if (pgram_uvt.z() >= 0 && pgram_uvt.x() >= 0 && pgram_uvt.y() >= 0 && pgram_uvt.x() <= 1 && pgram_uvt.y() <= 1)
            {

                Vector3d ray_intersection = ray_origin + (pgram_uvt.z() * ray_direction);
                Vector3d ray_normal = (pgram_v.cross(pgram_u)).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            } 
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    //Store values for R, G, B
    MatrixXd R = MatrixXd::Zero(800, 800);
    MatrixXd G = MatrixXd::Zero(800, 800);
    MatrixXd B = MatrixXd::Zero(800, 800);
    
    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    // Add intensity for light
    const double lightIntesity = 1;

    for (unsigned i = 0; i < A.cols(); ++i)
    {
        for (unsigned j = 0; j < A.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            //const Vector3d ray_direction = camera_view_direction;

            //Change to perspective camera
            Vector3d ray_direction = pixel_center - camera_origin;

            bool sph_intersect = false;
            Vector2d sph_t_interval;
            double discriminant = pow((ray_direction.dot(ray_origin - sphere_center)), 2) -
                ((ray_direction.dot(ray_direction)) * ((ray_origin - sphere_center).dot(ray_origin - sphere_center) - pow(sphere_radius, 2)));

            if (discriminant >= 0) {
                sph_t_interval = solveSphere(ray_origin, ray_direction, sphere_center, discriminant);
                sph_intersect = true;
            }

            if (sph_intersect)
            {
                Vector3d ray_intersection = ray_origin + (sph_t_interval.x() * ray_direction);

                // Compute normal at the intersection point
                //Vector3d ray_normal = ray_intersection.normalized();
                Vector3d ray_normal = (2 * (ray_intersection - sphere_center)).normalized();

                //Calculate the bisector for specular shading
                Vector3d bisector = ((camera_origin - ray_intersection).normalized()+(light_position - ray_intersection).normalized()).normalized();
                
                //Calculate colour based on material params and light
                double diffuseR = diffuse_color.x() * lightIntesity * std::max(0.0, (light_position - ray_intersection).normalized().dot(ray_normal));
                double specularR = specular_color.x() * lightIntesity * std::max(0.0, pow((ray_normal.dot(bisector)), specular_exponent));
                double diffuseG = diffuse_color.y() * lightIntesity * std::max(0.0, (light_position - ray_intersection).normalized().dot(ray_normal));
                double specularG = specular_color.y() * lightIntesity * std::max(0.0, pow((ray_normal.dot(bisector)), specular_exponent));
                double diffuseB = diffuse_color.z() * lightIntesity * std::max(0.0, (light_position - ray_intersection).normalized().dot(ray_normal));
                double specularB = specular_color.z() * lightIntesity * std::max(0.0, pow((ray_normal.dot(bisector)), specular_exponent));

                // Simple diffuse model
                //C(i, j) = ambient + diffuse + specular;
                R(i, j) = ambient + diffuseR + specularR;
                G(i, j) = ambient + diffuseG + specularG;
                B(i, j) = ambient + diffuseB + specularB;

                // Clamp to zero
                //C(i, j) = std::max(C(i, j), 0.);
                R(i, j) = std::max(R(i, j), 0.);
                G(i, j) = std::max(G(i, j), 0.);
                B(i, j) = std::max(B(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }
    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
