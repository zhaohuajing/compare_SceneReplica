#version 140
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