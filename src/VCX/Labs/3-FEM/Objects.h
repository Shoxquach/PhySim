#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace VCX::Labs::FEM {
    namespace Tet {
        const std::vector<int> tet_index = {
            0, 1, 2, 3
        };
        const std::vector<std::uint32_t> tri_index = {
            0, 1, 2,
            0, 2, 3,
            0, 3, 1,
            3, 2, 1
        };
        const std::vector<std::uint32_t> line_index = {
            0, 1, 0, 2, 0, 3, 1, 2, 2, 3, 1, 3
        };
        const std::vector<glm::vec3> tet_vertex_positions = {
            glm::vec3(-1.f, -1.f, -1.f),
            glm::vec3(1.f, 1.f, -1.f),
            glm::vec3(1.f, -1.f, 1.f),
            glm::vec3(-1.f, 1.f, 1.f)
        };
    }
    namespace Cube {
        const std::vector<std::uint32_t> tri_index = {
            0, 1, 2, 0, 2, 3,
            5, 1, 0, 5, 0, 4,
            1, 5, 2, 5, 6, 2,
            2, 3, 7, 2, 7, 6,
            0, 3, 7, 0, 7, 4,
            4, 7, 5, 7, 6, 5
        };
        const std::vector<std::uint32_t> line_index = {
            0, 1, 1, 2, 2, 3, 3, 0,
            4, 5, 5, 6, 6, 7, 7, 4,
            0, 4, 1, 5, 2, 6, 3, 7,
            0, 2, 0, 5, 2, 5, 2, 7, 0, 7, 5, 7
        };
        const std::vector<int> tet_index = {
            0, 1, 2, 5,
            0, 2, 3, 7,
            0, 4, 5, 7,
            2, 5, 6, 7,
            1, 3, 4, 6
        };
        const std::vector<glm::vec3> tet_vertex_positions = {
            glm::vec3(0.f, 0.f, 0.f),
            glm::vec3(1.f, 0.f, 0.f),
            glm::vec3(1.f, 1.f, 0.f),
            glm::vec3(0.f, 1.f, 0.f),
            glm::vec3(0.f, 0.f, 1.f),
            glm::vec3(1.f, 0.f, 1.f),
            glm::vec3(1.f, 1.f, 1.f),
            glm::vec3(0.f, 1.f, 1.f)
        };
    }
    namespace Object1 {
        const std::vector<std::uint32_t> tri_index = {
            0, 1, 8, 1, 9, 8, 1, 10, 9, 1, 2, 10, 2, 3, 10, 3, 11, 10, 3, 12, 11, 3, 4, 12,
            4, 5, 12, 5, 13, 12, 5, 14, 13, 5, 6, 14, 6, 7, 14, 7, 15, 14, 7, 8, 15, 7, 0, 8,
            1, 0, 7, 1, 7, 6, 2, 1, 3, 3, 1, 6, 3, 5, 4, 5, 3, 6, 8, 9, 14, 8, 14, 15,
            9, 10, 14, 10, 11, 14, 11, 12, 14, 12, 13, 14
        };
        const std::vector<std::uint32_t> line_index = {
            0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 0,
            1, 7, 1, 6, 1, 3, 3, 6, 3, 5,
            0, 8, 1, 9, 2, 10, 3, 11, 4, 12, 5, 13, 6, 14, 7, 15,
            1, 8, 1, 10, 3, 10, 3, 12, 5, 12, 5, 14, 7, 14, 7, 8,
            8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 14, 14, 15, 15, 8,
            8, 14, 9, 14, 10, 14, 11, 14, 12, 14
        };
        const std::vector<int> tet_index = {
            0, 1, 7, 9,
            1, 6, 7, 14,
            1, 7, 8, 14,
            1, 8, 9, 14,
            7, 8, 14, 15,

            1, 2, 3, 10,
            1, 3, 6, 14,
            1, 3, 10, 14,
            1, 9, 10, 14,
            3, 10, 11, 14,

            3, 4, 5, 12,
            3, 5, 6, 14,
            3, 5, 12, 14,
            3, 11, 12, 14,
            5, 12, 13, 14
        };
        const std::vector<glm::vec3> tet_vertex_positions = {
            glm::vec3(0.f, 0.f, 0.f),
            glm::vec3(1.f, 0.f, 0.f),
            glm::vec3(2.f, 0.f, 0.f),
            glm::vec3(2.f, 1.f, 0.f),
            glm::vec3(2.f, 2.f, 0.f),
            glm::vec3(1.f, 2.f, 0.f),
            glm::vec3(1.f, 1.f, 0.f),
            glm::vec3(0.f, 1.f, 0.f),
            glm::vec3(0.f, 0.f, 1.f),
            glm::vec3(1.f, 0.f, 1.f),
            glm::vec3(2.f, 0.f, 1.f),
            glm::vec3(2.f, 1.f, 1.f),
            glm::vec3(2.f, 2.f, 1.f),
            glm::vec3(1.f, 2.f, 1.f),
            glm::vec3(1.f, 1.f, 1.f),
            glm::vec3(0.f, 1.f, 1.f)
        };
    }
    namespace Icosahedron {
        const std::vector<int> tet_index = {
            0, 1, 2, 3,
            0, 1, 2, 5,
            0, 1, 3, 9,
            0, 1, 4, 5,
            0, 1, 4, 9,
            0, 2, 3, 7,
            0, 2, 5, 6,
            0, 2, 6, 7,
            0, 3, 7, 8,
            0, 3, 8, 9,
            0, 4, 5, 12,
            0, 4, 9, 11,
            0, 4, 11, 12,
            0, 5, 6, 12,
            0, 6, 7, 10,
            0, 6, 10, 12,
            0, 7, 8, 10,
            0, 8, 9, 11,
            0, 8, 10, 11,
            0, 10, 11, 12
        };
        const std::vector<std::uint32_t> tri_index = {
            1, 2, 3,
            1, 2, 5,
            1, 3, 9,
            1, 4, 5,
            1, 4, 9,
            2, 3, 7,
            2, 5, 6,
            2, 6, 7,
            3, 7, 8,
            3, 8, 9,
            4, 5, 12,
            4, 9, 11,
            4, 11, 12,
            5, 6, 12,
            6, 7, 10,
            6, 10, 12,
            7, 8, 10,
            8, 9, 11,
            8, 10, 11,
            10, 11, 12
        };
        const std::vector<std::uint32_t> line_index = {
            1, 2, 1, 3, 1, 4, 1, 5, 1, 9,
            2, 3, 2, 5, 2, 6, 2, 7,
            3, 7, 3, 8, 3, 9,
            4, 5, 4, 9, 4, 11, 4, 12,
            5, 6, 5, 12,
            6, 7, 6, 10, 6, 12,
            7, 8, 7, 10,
            8, 9, 8, 10, 8, 11,
            9, 11,
            10, 11, 10, 12,
            11, 12
        };
        const float phi = 1.618f;
        const std::vector<glm::vec3> tet_vertex_positions = {
            glm::vec3(0.f, 0.f, 0.f),
            glm::vec3(0.0f, 1.0f, phi),
            glm::vec3(phi, 0.0f, 1.0f),
            glm::vec3(0.0f, -1.0f, phi),
            glm::vec3(-1.0f, phi, 0.0f),
            glm::vec3(1.0f, phi, 0.0f),
            glm::vec3(phi, 0.0f, -1.0f),
            glm::vec3(1.0f, -phi, 0.0f),
            glm::vec3(-1.0f, -phi, 0.0f),
            glm::vec3(-phi, 0.0f, 1.0f),
            glm::vec3(0.0f, -1.0f, -phi),
            glm::vec3(-phi, 0.0f, -1.0f),
            glm::vec3(0.0f, 1.0f, -phi)
        };
    }
    namespace Object2 {
        const std::vector<int> tet_index = {
            0, 1, 2, 3,
            1, 2, 3, 4,
            2, 3, 4, 6,
            2, 4, 5, 6,
            4, 5, 6, 7,
            5, 6, 7, 9,
            5, 7, 8, 9,
            7, 8, 9, 10,
            8, 9, 10, 12,
            8, 10, 11, 12,
            10, 11, 12, 13,
            11, 12, 13, 14
        };
        const std::vector<std::uint32_t> tri_index = {
            0, 1, 2, 0, 1, 3, 1, 3, 4, 3, 4, 6, 4, 6, 7, 6, 7, 9, 7, 9, 10, 9, 10, 12, 10, 12, 13, 12, 13, 14,
            1, 2, 4, 2, 4, 5, 4, 5, 7, 5, 7, 8, 7, 8, 10, 8, 10, 11, 10, 11, 13,
            0, 2, 3, 2, 3, 6, 2, 5, 6, 5, 6, 9, 5, 8, 9, 8, 9, 12, 8, 11, 12, 11, 12, 14, 11, 13, 14
        };
        const std::vector<std::uint32_t> line_index = {
            0, 1, 1, 3, 3, 4, 4, 6, 6, 7, 7, 9, 9, 10, 10, 12, 12, 13, 13, 14,
            0, 3, 3, 6, 6, 9, 9, 12, 12, 14,
            1, 4, 4, 7, 7, 10, 10, 13,
            1, 2, 2, 4, 4, 5, 5, 7, 7, 8, 8, 10, 10, 11, 11, 13,
            2, 5, 5, 8, 8, 11,
            0, 2, 2, 3, 2, 6, 5, 6, 5, 9, 8, 9, 8, 12, 11, 12, 11, 14
        };
        const std::vector<glm::vec3> tet_vertex_positions = {
            glm::vec3(0.f, 0.f, 0.f),
            glm::vec3(-0.5f, 1.f, 0.5f),
            glm::vec3(0.5f, 1.f, 0.5f),
            glm::vec3(0.f, 0.f, 1.f),
            glm::vec3(-0.5f, 1.f, 1.5f),
            glm::vec3(0.5f, 1.f, 1.5f),
            glm::vec3(0.f, 0.f, 2.f),
            glm::vec3(-0.5f, 1.f, 2.5f),
            glm::vec3(0.5f, 1.f, 2.5f),
            glm::vec3(0.f, 0.f, 3.f),
            glm::vec3(-0.5f, 1.f, 3.5f),
            glm::vec3(0.5f, 1.f, 3.5f),
            glm::vec3(0.f, 0.f, 4.f),
            glm::vec3(-0.5f, 1.f, 4.5f),
            glm::vec3(0.5f, 1.f, 4.5f),
            glm::vec3(0.f, 0.f, 5.f),
        };
    }

    class Block {
        int x, y, z;
        int getId(int i, int j, int k) {
            return i * (y + 1) * (z + 1) + j * (z + 1) + k;
        }
        void addTet(int p1, int p2, int p3, int p4) {
            tet_index.push_back(p1);
            tet_index.push_back(p2);
            tet_index.push_back(p3);
            tet_index.push_back(p4);
        }
        void addTri(int p1, int p2, int p3) {
            tri_index.push_back(p1);
            tri_index.push_back(p2);
            tri_index.push_back(p3);
        }
        void addLine(int p1, int p2) {
            line_index.push_back(p1);
            line_index.push_back(p2);
        }

    public:
        std::vector<std::uint32_t> tri_index, line_index;
        std::vector<int> tet_index;
        std::vector<glm::vec3> tet_vertex_positions;
        Block(int _x = 2, int _y = 2, int _z = 8):
            x(_x), y(_y), z(_z) {
            int n_Vertices = (x + 1) * (y + 1) * (z + 1);
            tet_vertex_positions.resize(n_Vertices);
            for (int i = 0; i <= x; i ++) {
                for (int j = 0; j <= y; j ++) {
                    for (int k = 0; k <= z; k ++) {
                        tet_vertex_positions[getId(i, j, k)] = glm::vec3(i, j, k);
                    }
                }
            }
            for (int i = 0; i < x; i ++) {
                for (int j = 0; j < y; j ++) {
                    for (int k = 0; k < z; k ++) {
                        int v1 = getId(i, j, k);
                        int v2 = getId(i, j + 1, k);
                        int v3 = getId(i, j + 1, k + 1);
                        int v4 = getId(i, j, k + 1);
                        int v5 = getId(i + 1, j, k);
                        int v6 = getId(i + 1, j + 1, k);
                        int v7 = getId(i + 1, j + 1, k + 1);
                        int v8 = getId(i + 1, j, k + 1);
                        addTet(v1, v2, v4, v5);
                        addTet(v2, v5, v6, v7);
                        addTet(v2, v3, v4, v7);
                        addTet(v4, v5, v7, v8);
                        addTet(v2, v4, v5, v7);
                    }
                }
            }
            for (int i = 0; i < x; i ++) {
                for (int j = 0; j < y; j ++) {
                    int v1 = getId(i, j, 0);
                    int v2 = getId(i, j + 1, 0);
                    int v3 = getId(i + 1, j + 1, 0);
                    int v4 = getId(i + 1, j, 0);
                    addTri(v1, v2, v4);
                    addTri(v2, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v4);
                    addLine(v2, v4);
                    if (i == x - 1) {
                        addLine(v3, v4);
                    }
                    if (j == y - 1) {
                        addLine(v2, v3);
                    }
                    v1 = getId(i, j, z);
                    v2 = getId(i, j + 1, z);
                    v3 = getId(i + 1, j + 1, z);
                    v4 = getId(i + 1, j, z);
                    addTri(v1, v2, v3);
                    addTri(v1, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v3);
                    addLine(v1, v4);
                    if (i == x - 1) {
                        addLine(v3, v4);
                    }
                    if (j == y - 1) {
                        addLine(v2, v3);
                    }
                }
            }
            for (int i = 0; i < x; i ++) {
                for (int k = 0; k < z; k ++) {
                    int v1 = getId(i, 0, k);
                    int v2 = getId(i, 0, k + 1);
                    int v3 = getId(i + 1, 0, k + 1);
                    int v4 = getId(i + 1, 0, k);
                    addTri(v1, v2, v4);
                    addTri(v2, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v4);
                    addLine(v2, v4);
                    if (i == x - 1) {
                        addLine(v3, v4);
                    }
                    if (k == z - 1) {
                        addLine(v2, v3);
                    }
                    v1 = getId(i, y, k);
                    v2 = getId(i, y, k + 1);
                    v3 = getId(i + 1, y, k + 1);
                    v4 = getId(i + 1, y, k);
                    addTri(v1, v2, v3);
                    addTri(v1, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v3);
                    addLine(v1, v4);
                    if (i == x - 1) {
                        addLine(v3, v4);
                    }
                    if (k == z - 1) {
                        addLine(v2, v3);
                    }
                }
            }
            for (int j = 0; j < y; j ++) {
                for (int k = 0; k < z; k ++) {
                    int v1 = getId(0, j, k);
                    int v2 = getId(0, j, k + 1);
                    int v3 = getId(0, j + 1, k + 1);
                    int v4 = getId(0, j + 1, k);
                    addTri(v1, v2, v4);
                    addTri(v2, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v4);
                    addLine(v2, v4);
                    if (j == y - 1) {
                        addLine(v3, v4);
                    }
                    if (k == z - 1) {
                        addLine(v2, v3);
                    }
                    v1 = getId(x, j, k);
                    v2 = getId(x, j, k + 1);
                    v3 = getId(x, j + 1, k + 1);
                    v4 = getId(x, j + 1, k);
                    addTri(v1, v2, v3);
                    addTri(v1, v3, v4);
                    addLine(v1, v2);
                    addLine(v1, v3);
                    addLine(v1, v4);
                    if (j == y - 1) {
                        addLine(v3, v4);
                    }
                    if (k == z - 1) {
                        addLine(v2, v3);
                    }
                }
            }
        }
    };
    enum Object { tet, cube, object1, icosahedron, object2, block };
    // namespace Object = Icosahedron;
}