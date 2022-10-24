#version 150

uniform mat4 modelViewProjectionMatrix;
in vec4 position;

// texture coordinates are sent to fragment shader
in vec2 texcoord;
out vec2 texCoordVarying;

void main(){
	texCoordVarying = texcoord;
    gl_Position = modelViewProjectionMatrix * position;
}