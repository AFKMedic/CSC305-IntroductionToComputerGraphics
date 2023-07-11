// Added support for spheres and parallelogram
// Added support for shadows and reflections
// Building tree is not implemented but code for box intersection is
// written but commeneted out so that the program still runs

////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "bunny.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

//Maximum number of recursive calls
const int max_bounce = 5;

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

// Objects
std::vector<Vector3d> sphere_centers;
std::vector<double> sphere_radii;
std::vector<Matrix3d> parallelograms;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Spheres
    sphere_centers.emplace_back(1, 0.2, -2);
    sphere_radii.emplace_back(1);
  
    //parallelograms
    parallelograms.emplace_back();
    parallelograms.back() << -100, 100, -100,
        -1.25, 0, -1.2,
        -100, -100, 100;

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
    MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    // TODO

    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.

}


////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    // Triangle Origin is a
    // Other verticies are b and c
    const Vector3d a_to_b = b - a;
    const Vector3d a_to_c = c - a;

    Vector3d uvt;
    Matrix3d AA;
    AA.col(0) = -(a_to_b).transpose();
    AA.col(1) = -(a_to_c).transpose();
    AA.col(2) = ray_direction.transpose();

    // Solve system
    uvt = AA.inverse() * (a - ray_origin);

    if (uvt.z() >= 0 && uvt.x() >= 0 && uvt.y() >= 0 && (uvt.x() + uvt.y()) <= 1) {
        p = ray_origin + (uvt.z() * ray_direction);
        N = (a_to_b.cross(a_to_c)).normalized();
        return uvt.z();
    }
    return -1;
}

double ray_sphere_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    const Vector3d sphere_center = sphere_centers[index];
    const double sphere_radius = sphere_radii[index];

    double t = -1;

    double AA = ray_direction.dot(ray_direction);
    double BB = 2 * ray_origin.dot(ray_direction) - 2 * ray_direction.dot(sphere_center);
    double CC = (sphere_center - ray_origin).dot(sphere_center - ray_origin) - sphere_radius * sphere_radius;

    double delta = BB * BB - 4 * AA * CC;

    if (delta < 0)
    {
        return t;
    }
    else
    {
        double t1 = (-BB + sqrt(delta)) / (2 * AA);
        double t2 = (-BB - sqrt(delta)) / (2 * AA);
        t = std::min(t1, t2);
        if (t < 0) {
            t = std::max(t1, t2);
        }
        p = ray_origin + t * ray_direction;
        N = (2 * (p - sphere_center)).normalized();
        return t;
    }
}

//Compute the intersection between a ray and a paralleogram, return -1 if no intersection
double ray_parallelogram_intersection(const Vector3d& ray_origin, const Vector3d& ray_direction, int index, Vector3d& p, Vector3d& N)
{
    const Vector3d pgram_origin = parallelograms[index].col(0);
    const Vector3d A = parallelograms[index].col(1);
    const Vector3d B = parallelograms[index].col(2);
    const Vector3d pgram_u = A - pgram_origin;
    const Vector3d pgram_v = B - pgram_origin;


    Vector3d uvt;
    Matrix3d AA;
    AA.col(0) = -(pgram_u).transpose();
    AA.col(1) = -(pgram_v).transpose();
    AA.col(2) = ray_direction.transpose();

    uvt = AA.inverse() * (pgram_origin - ray_origin);

    if (uvt.z() >= 0 && uvt.x() >= 0 && uvt.y() >= 0 && uvt.x() <= 1 && uvt.y() <= 1) {
        p = ray_origin + (uvt.z() * ray_direction);
        N = (pgram_v.cross(pgram_u)).normalized();
        return uvt.z();
    }

    return -1;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    /*
    Vector3f boxMin = box.min();
    Vector3f boxMax = box.max();
    float tMinX;
    float tMaxX;
    float tMinY;
    float tMaxY;
    float tMinZ;
    float tMaxZ;
    float invRayX = 1 / ray_direction.x();
    float invRayY = 1 / ray_direction.y();
    float invRayZ = 1 / ray_direction.z();


    if (invRayX >= 0) {
        tMinX = (boxMin.x() - ray_origin.x()) * invRayX;
        tMaxX = (boxMax.x() - ray_origin.x()) * invRayX;
    }
    else {
        tMinX = (boxMax.x() - ray_origin.x()) * invRayX;
        tMaxX = (boxMin.x() - ray_origin.x()) * invRayX;
    }

    if (invRayY >= 0) {
        tMinY = (boxMin.x() - ray_origin.y()) * invRayY;
        tMaxY = (boxMax.x() - ray_origin.y()) * invRayY;
    }
    else {
        tMinY = (boxMax.x() - ray_origin.y()) * invRayY;
        tMaxY = (boxMin.x() - ray_origin.y()) * invRayY;
    }

    if (tMinX > tMaxY || tMinY > tMaxX) {
        return false;
    }

    if (invRayZ >= 0) {
        tMinZ = (boxMin.z() - ray_origin.z()) * invRayZ;
        tMaxZ = (boxMax.z() - ray_origin.z()) * invRayZ;
    }
    else {
        tMinZ = (boxMax.z() - ray_origin.z()) * invRayZ;
        tMaxZ = (boxMin.z() - ray_origin.z()) * invRayZ;
    }

    if (tMinX > tMaxZ || tMinZ > tMaxX) {
        return false;
    }
    */

    //return true;
    return false;
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    double closest_t = std::numeric_limits<double>::max();
    bool intersect = false;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    for (int i = 0; i < facets.rows(); ++i)
    {
        int vertexALocation = facets(i, 0);
        int vertexBLocation = facets(i, 1);
        int vertexCLocation = facets(i, 2);

        Vector3d vertexA = vertices.row(vertexALocation);
        Vector3d vertexB = vertices.row(vertexBLocation);
        Vector3d vertexC = vertices.row(vertexCLocation);

        const double t = ray_triangle_intersection(ray_origin, ray_direction, vertexA, vertexB, vertexC, tmp_p, tmp_N);
        if (t >= 0)
        {
            if (t < closest_t)
            {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
            intersect = true;
        }
    }

    for (int i = 0; i < sphere_centers.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_sphere_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
            intersect = true;
        }
    }

    for (int i = 0; i < parallelograms.size(); ++i)
    {
        //returns t and writes on tmp_p and tmp_N
        const double t = ray_parallelogram_intersection(ray_origin, ray_direction, i, tmp_p, tmp_N);
        //We have intersection
        if (t >= 0)
        {
            //The point is before our current closest t
            if (t < closest_t)
            {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
            }
            intersect = true;
        }
    }
    return intersect;
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

//Checks if the light is visible
bool is_light_visible(const Vector3d& ray_origin, const Vector3d& ray_direction, const Vector3d& light_position)
{
    Vector3d tmp_p;
    Vector3d tmp_N;
    bool intersection = find_nearest_object(ray_origin, ray_direction, tmp_p, tmp_N);

    if (!intersection) {
        return true;
    }
    else {
        if ((tmp_p - ray_origin).norm() < (light_position - ray_origin).norm()) {
            return false;
        }
        else {
            return true;
        }
    }
}

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction, int max_bounce)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;
    const double episilon = 0.0001;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        const Vector3d Li = (light_position - p).normalized();

        bool visible = is_light_visible((p + (Li * episilon)), Li, light_position);
        if (!visible) continue;

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        //const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    Vector4d refl_color = obj_reflection_color;
    Vector4d reflection_color(0, 0, 0, 0);
    Vector3d refl_ray = ((p - ray_origin) - (2 * ((p - ray_origin).dot(N)) * N)).normalized();
    if (max_bounce > 0) {
        reflection_color += refl_color.cwiseProduct(shoot_ray(p + episilon * refl_ray, refl_ray, max_bounce - 1));
    }


    // Rendering equation
    Vector4d C = ambient_color + lights_color + reflection_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);

    double image_y = (focal_length / (sin((3.145 / 2) - (field_of_view / 2)))) * sin((field_of_view / 2));
    double image_x = image_y * aspect_ratio;


    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction, max_bounce);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
