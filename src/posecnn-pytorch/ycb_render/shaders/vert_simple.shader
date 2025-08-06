// #version 460
#version 140
uniform mat4 V;
uniform mat4 P;

// layout (location=0) in vec3 position;
// layout (location=1) in vec3 normal;
// layout (location=2) in vec2 texCoords;
in vec3 position;
in vec3 normal;
in vec2 texCoords;

void main() {
    gl_Position = P * V * vec4(position,1);
}