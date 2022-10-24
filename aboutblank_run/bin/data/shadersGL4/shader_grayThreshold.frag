#version 150

//
uniform sampler2DRect tex0;
uniform float minX;
uniform float maxX;
uniform float minY;
uniform float maxY;
uniform float grayThresh;

//
in vec2 texCoordVarying;

// this is the output of the fragment shader
out vec4 outputColor;

void main(){
	// get texte
	vec4 texel0 = texture(tex0, texCoordVarying);

	if (texCoordVarying.x < minX || texCoordVarying.x > maxX || texCoordVarying.y < minY || texCoordVarying.y > maxY) {
		outputColor = vec4(0.0, 0.0, 0.0, 1.0);
		return;
	}

	// threshold
	float g = texel0.r * 255;
	vec4 c = vec4(1.0, 1.0, 1.0, 1.0);
	if(g <= grayThresh) {
		c = vec4(0.0, 0.0, 0.0, 1.0);
	}

	outputColor = c;
}