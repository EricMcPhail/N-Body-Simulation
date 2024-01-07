#version 460 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in mat4 aInstanceMatrix;

out vec3 fColor;

uniform mat4 view;
uniform mat4 projection;

void main() {
    fColor = aColor;
    gl_Position =  projection * view * aInstanceMatrix * vec4(aPos, 1.0);
}
