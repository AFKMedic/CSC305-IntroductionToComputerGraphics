// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5; //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    //TODO: setup uniform

    uniform.ambient_light = ambient_light;

    //TODO: setup camera, compute w, u, v

	Vector3d w = -camera_gaze.normalized();
    Vector3d u = (camera_top.cross(w)).normalized();
    Vector3d v = w.cross(u);

    //TODO: compute the camera transformation

	Matrix4d camera_transform;
    
    camera_transform << u.x(), v.x(), w.x(), camera_position.x(),
        u.y(), v.y(), w.y(), camera_position.y(),
        u.z(), v.z(), w.z(), camera_position.z(),
        0, 0, 0, 1;
    
    camera_transform = camera_transform.inverse().eval();

    //TODO: setup projection matrix

	double image_y = tan(field_of_view/2) * near_plane;
    double image_x = image_y * aspect_ratio;
    double right = image_x;
    double left = -right;
    double top = image_y;
    double bottom = -top;
    double near = -near_plane;
    double far = -far_plane;

    Matrix4d ortho;
    Matrix4d project;
    ortho <<
        2 / (right - left), 0,                      0,                      -(right + left) / (right - left),
        0,                  2 / (top - bottom),     0,                      -(top + bottom) / (top - bottom),
        0,                  0,                      2 / (near - far),       -(near + far) / (near - far),
        0,                  0,                      0,                      1;

    project <<
        near, 0, 0, 0,
        0, near, 0, 0,
        0, 0, near + far, -far * near,
        0, 0, 1, 0;
	

    Matrix4d P;
    if (is_perspective)
    {
        //TODO setup prespective camera
        uniform.matrix_final = ortho * project  * camera_transform;
    }
    else
    {
        uniform.matrix_final = ortho * camera_transform;
    }
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        //return va;
		VertexAttributes out;
        out.position = uniform.matrix_final.cast <float> () * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets

    for (int i = 0; i < facets.rows(); ++i) {
        int vertexLocationA = facets(i, 0);
        int vertexLocationB = facets(i, 1);
        int vertexLocationC = facets(i, 2);

        VertexAttributes p1 = VertexAttributes(vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2));
        VertexAttributes p2 = VertexAttributes(vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2));
        VertexAttributes p3 = VertexAttributes(vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2));

        vertex_attributes.push_back(p1);
        vertex_attributes.push_back(p2);
        vertex_attributes.push_back(p3);
    }
	
    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;
        res << cos(alpha), 0, sin(alpha), 0,
        0, 1, 0, 0,
        -sin(alpha), 0, cos(alpha), 0,
        0, 0, 0, 1;
    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        //return va;
        VertexAttributes out;
        out.position = uniform.matrix_final.cast <float>() * uniform.rotate.cast <float>() * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    //TODO: use the transformation matrix
    
    uniform.rotate << trafo;
    for (int i = 0; i < facets.rows(); ++i) {
        int vertexLocationA = facets(i, 0);
        int vertexLocationB = facets(i, 1);
        int vertexLocationC = facets(i, 2);
        VertexAttributes p1 = VertexAttributes(vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2));
        VertexAttributes p2 = VertexAttributes(vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2));
        VertexAttributes p3 = VertexAttributes(vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2));
        vertex_attributes.push_back(p1);
        vertex_attributes.push_back(p2);
        vertex_attributes.push_back(p1);
        vertex_attributes.push_back(p3);
        vertex_attributes.push_back(p2);
        vertex_attributes.push_back(p3);
    }
    
    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        //TODO: compute the correct lighting
        VertexAttributes out;
        out.position = uniform.matrix_final.cast <float>() * uniform.rotate.cast <float>() * va.position;
        Matrix3d rotateNonHom;
        rotateNonHom << uniform.rotate(0, 0), uniform.rotate(1, 0), uniform.rotate(2, 0),
                        uniform.rotate(0, 1), uniform.rotate(1, 1), uniform.rotate(2, 1),
                        uniform.rotate(0, 2), uniform.rotate(1, 2), uniform.rotate(2, 2);
        out.normal = rotateNonHom * va.normal;

        // Transform normal
        // Not working
        /*
        Vector4d n;
        n << va.normal.x(), va.normal.y(), va.normal.z(), 0;
        n = uniform.matrix_final.inverse().transpose() * n;
        n = n.normalized();

        out.normal = Vector3d(n.x(), n.y(), n.z());
        */

        Vector3d pos(out.position[0], out.position[1], out.position[2]);

        Vector3d lights_color(0, 0, 0);
        Vector3d ambient_color = ambient_light.array() * ambient_light.array();

        for (int i = 0; i < light_positions.size(); ++i)
        {
            const Vector3d& light_position = light_positions[i];
            const Vector3d& light_color = light_colors[i];

            const Vector3d Li = (light_position - pos).normalized();

            Vector3d diff_color = va.obj_diffuse_color;
            Vector3d spec_color = va.obj_specular_color;
            Vector3d diffuse = diff_color * std::max(Li.dot(out.normal), 0.0);
            Vector3d bisector = ((camera_position - pos).normalized() + (light_position - pos).normalized()).normalized();
            Vector3d specular = spec_color * std::max(0.0, pow((out.normal.dot(bisector)), va.obj_specular_exponent));
            const Vector3d D = light_position - pos;
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
        }
        

        Vector3d C = ambient_color + lights_color;
        out.color << C[0], C[1], C[2], 1;
        

        return out;
        //return va;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        //return FragmentAttributes(1, 0, 0);
        FragmentAttributes out(va.color[0], va.color[1], va.color[2], va.color[3]);
        if (is_perspective) {
            out.depth = va.position[2];
        }
        else {
            out.depth = -va.position[2];
        }
        return out;
        //return FragmentAttributes (va.color[0], va.color[1], va.color[2], va.color[3]);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        
        if(fa.depth < previous.depth){
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.depth;
            return out;
        }
        else {
            return previous;
        }
        
        //return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);
    uniform.rotate << trafo;

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: compute the normals
    //TODO: set material colors
    
   
    for (int i = 0; i < facets.rows(); ++i) {
        int vertexLocationA = facets(i, 0);
        int vertexLocationB = facets(i, 1);
        int vertexLocationC = facets(i, 2);

        VertexAttributes p1 = VertexAttributes(vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2));
        VertexAttributes p2 = VertexAttributes(vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2));
        VertexAttributes p3 = VertexAttributes(vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2));

        Vector3d a;
        a << vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2);

        Vector3d b;
        b << vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2);

        Vector3d c;
        c << vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2);

        Vector3d ba = b - a;
        Vector3d ca = c - a;

        Vector3d n = ca.cross(ba).normalized();

        p1.normal << n;
        p2.normal << n;
        p3.normal << n;

        p1.obj_specular_color << obj_specular_color;
        p2.obj_specular_color << obj_specular_color;
        p3.obj_specular_color << obj_specular_color;

        p1.obj_diffuse_color << obj_diffuse_color;
        p2.obj_diffuse_color << obj_diffuse_color;
        p3.obj_diffuse_color << obj_diffuse_color;

        p1.obj_specular_exponent = obj_specular_exponent;
        p2.obj_specular_exponent = obj_specular_exponent;
        p3.obj_specular_exponent = obj_specular_exponent;

        vertex_attributes.push_back(p1);
        vertex_attributes.push_back(p2);
        vertex_attributes.push_back(p3);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);

    uniform.rotate << trafo;

    //TODO: compute the vertex normals as vertex normal average

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes
    //TODO: set material colors

    std::vector<Vector3d> normalAvg(vertices.rows(), Vector3d(0,0,0));
    for (int i = 0; i < facets.rows(); i++) {
        int vertexLocationA = facets(i, 0);
        int vertexLocationB = facets(i, 1);
        int vertexLocationC = facets(i, 2);

        VertexAttributes p1 = VertexAttributes(vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2));
        VertexAttributes p2 = VertexAttributes(vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2));
        VertexAttributes p3 = VertexAttributes(vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2));

        Vector3d a;
        a << vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2);

        Vector3d b;
        b << vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2);

        Vector3d c;
        c << vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2);

        Vector3d ba = b - a;
        Vector3d ca = c - a;

        Vector3d n = ca.cross(ba).normalized();

        normalAvg[vertexLocationA] << (normalAvg[vertexLocationA] + n).normalized();
        normalAvg[vertexLocationB] << (normalAvg[vertexLocationA] + n).normalized();
        normalAvg[vertexLocationC] << (normalAvg[vertexLocationA] + n).normalized();
    }


    for (int i = 0; i < facets.rows(); ++i) {
        int vertexLocationA = facets(i, 0);
        int vertexLocationB = facets(i, 1);
        int vertexLocationC = facets(i, 2);

        VertexAttributes p1 = VertexAttributes(vertices(vertexLocationA, 0), vertices(vertexLocationA, 1), vertices(vertexLocationA, 2));
        VertexAttributes p2 = VertexAttributes(vertices(vertexLocationB, 0), vertices(vertexLocationB, 1), vertices(vertexLocationB, 2));
        VertexAttributes p3 = VertexAttributes(vertices(vertexLocationC, 0), vertices(vertexLocationC, 1), vertices(vertexLocationC, 2));
        
        p1.obj_specular_color << obj_specular_color;
        p2.obj_specular_color << obj_specular_color;
        p3.obj_specular_color << obj_specular_color;

        p1.obj_diffuse_color << obj_diffuse_color;
        p2.obj_diffuse_color << obj_diffuse_color;
        p3.obj_diffuse_color << obj_diffuse_color;

        p1.obj_specular_exponent = obj_specular_exponent;
        p2.obj_specular_exponent = obj_specular_exponent;
        p3.obj_specular_exponent = obj_specular_exponent;

        p1.normal << normalAvg[vertexLocationA];
        p2.normal << normalAvg[vertexLocationB];
        p3.normal << normalAvg[vertexLocationC];

        vertex_attributes.push_back(p1);
        vertex_attributes.push_back(p2);
        vertex_attributes.push_back(p3);
    }

    

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    
    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer.setConstant(FrameBufferAttributes());

    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);
    
    frameBuffer.setConstant(FrameBufferAttributes());

    //TODO: add the animation
    
    int delay = 20;
    float angle = 0;
    GifWriter g;
    GifBegin(&g, "wireframe.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for (float i = 0; i < 24; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(angle, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
        angle += (3.14) / 12;
    }

    GifEnd(&g);

    GifBegin(&g, "flat_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    angle = 0;
    for (float i = 0; i < 24; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(angle, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
        angle += (3.14) / 12;
    }

    GifEnd(&g);

    GifBegin(&g, "pv_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    angle = 0;
    for (float i = 0; i < 24; i++)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(angle, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
        angle += (3.14) / 12;
    }

    GifEnd(&g);
    

    return 0;
}
