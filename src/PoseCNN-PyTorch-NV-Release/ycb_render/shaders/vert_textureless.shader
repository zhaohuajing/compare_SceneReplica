#version 140
uniform mat4 V;
uniform mat4 P;
uniform mat4 pose_rot;
uniform mat4 pose_trans;
uniform vec3 instance_color; 
        
in vec3 position;
in vec3 normal;
in vec3 color;
out vec3 theColor;
out vec3 Normal;
out vec3 FragPos;
out vec3 Normal_cam;
out vec3 Instance_color;
out vec3 Pos_cam;
out vec3 Pos_obj;
out float inverse_normal;
void main() {
    gl_Position = P * V * pose_trans * pose_rot * vec4(position, 1);
    vec4 world_position4 = pose_trans * pose_rot * vec4(position, 1);
    FragPos = vec3(world_position4.xyz / world_position4.w); // in world coordinate
    Normal = normalize(mat3(pose_rot) * normal); // in world coordinate
    Normal_cam = normalize(mat3(V) * mat3(pose_rot) * normal); // in camera coordinate
    
    vec4 pos_cam4 = V * pose_trans * pose_rot * vec4(position, 1);
    Pos_cam = pos_cam4.xyz / pos_cam4.w;
    Pos_obj = position;
    float normalDir = dot(Normal_cam, Pos_cam);           
    theColor = color;
    Instance_color = instance_color;
    inverse_normal = normalDir;
}