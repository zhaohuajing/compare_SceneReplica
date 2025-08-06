// #version 460
#version 140
// layout (location = 0) out vec4 outputColour;
// layout (location = 1) out vec4 NormalColour;
// layout (location = 2) out vec4 InstanceColour;
// layout (location = 3) out vec4 PCColour;
out vec4 outputColour;
out vec4 NormalColour;
out vec4 InstanceColour;
out vec4 PCColour;
void main() {
    outputColour = vec4(0.2, 0.2, 0.2, 1.0);
    NormalColour = vec4(0,0,0,0);
    InstanceColour = vec4(0,0,0,0);
    PCColour = vec4(0,0,0,0);

}