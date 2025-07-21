#version 140
uniform mat4 V;
uniform mat4 P;

in vec3 position;
in vec3 normal;
in vec2 texCoords;

void main() {
    gl_Position = P * V * vec4(position,1);
}