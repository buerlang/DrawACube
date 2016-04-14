#version 410

layout (location = 0) in vec3 vp;
layout (location = 1) in vec4 color;

out vec4 v2fColor;

uniform vec3 pickColor;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main(void){
	gl_Position = projection * view * model * vec4(vp, 1.0);
	v2fColor = vec4(pickColor, 1.0);
}